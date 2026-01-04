# Project 1. Computational Loco-Manipulation of Humanoid Robots {#project-1.-computational-loco-manipulation-of-humanoid-robots .标题1z}

### I. Introduction {#i.-introduction .标题3z}

When humanoid robots perform tasks such as moving goods, opening doors
and so on, upper-limb manipulation will significantly affect overall
balance. Therefore, the project introduces **Full Loco-Manipulation
Model** (**FLM)** as Sleiman et al. proposed, including **centroidal
dynamics, whole-body kinematics and operation dynamics** \[1\]. A single
multi-contact OCP is formulated and a **real-time transcription and
solver** are being developed based on **sequential convex programming**.

The project's infrastructure mainly refers to wb_humanoid_mpc \[2\] and
OCS2 \[4\]. However, the "full centroidal model" in OCS2 **directly
performs automatic differentiation on FLM**, which would lead to
**increased computational complexity and larger tape size** from CppAD,
due to the mixing of matrix inversions, CMM recursive algorithm and CoM
momentum's partial derivative.

Moreover, wb_humanoid_mpc adopts OCS2's "SQP" method with HPIPM solver,
in which Riccati Recursion algorithm is highly efficient through ZOH
discretization's sparse pattern. In my opinion, too many discrete
nodes - up to 60 with 1.2s horizon and 20ms step size -- lead to **a
huge convex subproblem, especially with many constraints**, in order to
meet ZOH's assumption that variables in each interval remain constant.
Besides, HPIPM **can't handle second-order cone (SOC)** constraints like
friction cone, and several **hard constraints** are **converted into
soft costs** (not sure reason).

In contrast, the project gives derivatives of centroidal dynamics and
whole-body kinematics through **theoretical derivation for hand-parser**
and will implement **SCP with a first-order primal-dual solver**
supports FOH discretization and SOC, resulting in **fewer nodes** (20
nodes in 1s horizon) and **smoother trajectory** (piecewise continuous).
Meanwhile a **customized solver** of fixed sparse pattern could be
comparable in speed to HPIPM. Though this requires a **fixed-structure
problem**, the engineering design of **static pre-allocation** for
maximum size is ideal for loco-manipulation tasks, where all constraints
and objectives are pre-modeled, then selectively activated by specific
problem or states.

### II Model of Full Loco-Manipulation Model {#ii-model-of-full-loco-manipulation-model .标题3z}

Loco-Manipulation requires simultaneous planning of the center of mass,
gait, and arm movements; therefore, its dynamic equation must
incorporate centroidal dynamics, whole-body kinematics, and task
dynamics.

Locomotion needs to **meet various constraints** required for **lower
limb gait tracking and dynamic balance**, and mainly consists of the
following components:

1)  **Foot contact constraints**, such as the friction cone of the
    standing foot, normal or zero velocity, contact moment, and zero
    wrench of the swinging foot;

2)  **Terminal state constraints**, strictly satisfying the terminal
    foot placement and corresponding whole-body pose determined by
    upper-level perception and Gait Schedule.

3)  **Actuator constraints**: strictly adhere to the range of position
    and speed amplitudes, torque amplitudes, and slopes of the joint
    servo motor.

Locomotion uses a **quadratic cost to track gait trajectories**
generated offline and fitted online, **penalize deviations** from the
default humanoid posture, and **optimize the impact** of contact forces
and execution costs. Therefore, it typically consists of the following
components:

1\) **Foot swing tracking** $p_{f} - p^{ref}\ $，

2\) **Quadratic Penalty of** $q - q^{ref}$

3\) **Secondary cost items for control inputs** $u^{T}Ru\ $ and
**Jerk**,

4\) **Approaching a target set of terminal state** with non-strict
constraints.

Similar to Locomotion, Manipulation also requires the addition of
constraints and costs to satisfy the physical laws of hand contact and
to complete upper limb manipulation tasks.

2.1 Full Loco-Manipulation Model

**State vector is**
$\left\lbrack h_{com},\ q_{b},q_{j} \right\rbrack \in R^{12 + n_{a}}$,
CoM momentum
$h_{com} = \left\lbrack {lm}_{com},{am}_{com} \right\rbrack \in R^{6}$
at Frame $G_{LWA}$, floating base position
$q_{b} = \left\lbrack r_{IB},\Phi_{IB} \right\rbrack \in R^{6}$ at Frame
$B_{LWA}$，Joint position $q_{j} \in R^{n_{a}}$. **Control input is**
$\lbrack f_{fl},f_{fr},f_{hl},f_{hr},v_{j}\rbrack$，Contact force
$f = \left\lbrack f_{c},\tau_{c} \right\rbrack \in R^{6}$, and joint
velocity $v_{j} \in R^{n_{a}}$。Other variables include the location of
the contact point $p_{c_{i}}(q)\ $and CoM position
$p_{com}(q)$，floating base speed
$v_{b} = {\dot{q}}_{b} = \lbrack lv_{IB},av_{IB}\rbrack$ and contacts'
speed
$v_{c_{i}}(v),\ v = \left\lbrack v_{b},v_{j} \right\rbrack = \lbrack{\dot{q}}_{b},{\dot{q}}_{j}\rbrack$.

It should be noted that the whole-body motion planning does not
incorporate floating base speed$\ v_{b}\ $.Instead of using it as a
control input, the nonlinear relationship is directly modeled with
first-order derivative$\ {\dot{q}}_{b}\ $and
$(h_{com},\ {\dot{q}}_{j})$. Therefore, when considering the contact
point velocity $v_{c_{i}}(v)\ $'s first-order linearization relatively
to $\delta(v_{b},v_{j})$, the differential chain rule needs to be used
to indirectly obtain the Jacobian coefficient relatively to
$\delta(h_{com},\ {\dot{q}}_{j})$.

2.1.A CoM Dynamics

$${\dot{\mathbf{h}}}_{\mathbf{com}}\mathbf{=}\begin{bmatrix}
\sum_{\mathbf{i = 1}}^{\mathbf{n}_{\mathbf{c}}}\mathbf{f}_{\mathbf{c}_{\mathbf{i}}}\mathbf{+ mg} \\
\sum_{\mathbf{i = 1}}^{\mathbf{n}_{\mathbf{c}}}{\left( \mathbf{p}_{\mathbf{c}_{\mathbf{i}}}\left( \mathbf{q} \right)\mathbf{-}\mathbf{p}_{\mathbf{com}}\mathbf{(q)} \right)\mathbf{\times}\mathbf{f}_{\mathbf{ci}}\mathbf{+}\mathbf{\tau}_{\mathbf{c}_{\mathbf{i}}}}
\end{bmatrix}$$

The CoM dynamics equations, in the WORLD coordinate system, describe
relationship between external contact forces and the center-of-mass
momentum. They neglect the robot\'s internal pose and velocity, focusing
only on the external force state.
$r_{c_{i}} = p_{c_{i}}(q) - p_{com}(q)\ $, the location of the contact
point and CoM are nonlinear relationship with
$q = \left\lbrack q_{b},q_{j} \right\rbrack\ $described by the SE(3)
transformation.

2.1.B Whole-body kinematics

Full-body motion planning requires not only planning contact forces and
center-of-mass momentum, but also the position and velocity of each limb
and floating base that forms CoM position and momentum. Therefore, it is
necessary to introduce the kinematic equations of the center of mass,

$$h_{com} = \left\lbrack A_{b}(q),\ A_{j}(q) \right\rbrack\begin{bmatrix}
{\dot{q}}_{b} \\
{\dot{q}}_{j}
\end{bmatrix}$$

$${\dot{q}}_{b} = A_{b}^{- 1}(q)(h_{com} - A_{j}(q){\dot{q}}_{j})$$
$A(q) \in R^{6 \times (6 + n_{a})}$ is Centroidal Momentum Matrix (CMM),
obtained recursively by the CCRBA algorithm.

2.1.C Task dynamics

在操作任务(Manipulation
Task)中，被操作对象的动力学千差万别且较重的操作任务会显著反作用于本机，导致失稳或任务失败，如搬运重物、推拉弹簧门等体力任务，因此Loco-Manipulation问题建模必须包含操作任务的动力学及其规划

$${\dot{x}}_{t} = \begin{bmatrix}
v_{t} \\
M_{t}^{- 1}( - J_{t}^{T}f_{t} - b_{t})
\end{bmatrix}$$
其中，状态$v_{t}$为箱子质心速度，$M_{t}$为惯量矩阵，$J_{t}$是，$b_{t}$，$f_{t}$为双手作用于对象的力和扭矩。

### III. Transcription of Full Loco-Manipulation Model {#iii.-transcription-of-full-loco-manipulation-model .标题3z}

3.1 Transcription of Full Loco-Manipulation Model

3.1.A Transcription of CoM Dynamics

Differentiate the angular momentum and linear momentum separately,
noting that the differential of position is the same as the Jacobian
matrix of velocity,

$$\delta\dot{am} = = \left\lbrack f_{c_{i}} \right\rbrack_{\times}^{T}\left( J_{c_{i}} - J_{com} \right)\begin{bmatrix}
\delta q_{b} \\
\delta q_{j}
\end{bmatrix} + \lbrack r_{c \times},\ I\rbrack\begin{bmatrix}
\delta f_{c_{i}} \\
\delta\tau_{c_{i}}
\end{bmatrix}$$

$$\delta\dot{lm} = I_{fl}\begin{bmatrix}
\delta f_{c} \\
\delta\tau_{c}
\end{bmatrix} + I_{fr}\begin{bmatrix}
\delta f_{c} \\
\delta\tau_{c}
\end{bmatrix} + \ldots$$
Considering only the relevant state and control inputs, the linearized
state equation is

$$\begin{bmatrix}
\delta\dot{lm} \\
\delta\dot{am}
\end{bmatrix} = \begin{bmatrix}
0 \\
A_{G}
\end{bmatrix}\begin{bmatrix}
\delta q_{b} \\
\delta q_{j}
\end{bmatrix} + B_{G}\begin{bmatrix}
\begin{matrix}
(\delta f_{c,fl},\ \delta\tau_{c,fl}) & \delta f_{fr}
\end{matrix} & \begin{matrix}
\delta f_{hl} & \delta f_{hr}
\end{matrix}
\end{bmatrix}^{T}$$

3.1.B Transcription of Whole-body kinematics

Differentiating the equation,

$$\delta{\dot{q}}_{b} = \frac{\partial v_{b}}{\partial h}\delta h_{com} + \frac{\partial v_{b}}{\partial{\dot{q}}_{j}}\delta{\dot{q}}_{j} + \frac{\partial v_{b}}{\partial q}\delta q = A_{b}^{- 1}\delta h_{com} - A_{b}^{- 1}A_{j}\delta{\dot{q}}_{j} - A_{b}^{- 1}\frac{\partial h_{com}}{\partial q}\delta q$$

$$\frac{\partial v_{b}}{\partial q} = - A_{b}^{- 1}\frac{\partial h_{com}}{\partial q}$$
Therefore, kinematic linearization requires first calculating the
partial derivative of CoM momentum with respect to joint position
$dhdq\ $, which recursive algorithms can achieve
$O\left( N^{2} \right)\ $complexity.

Although the automatic differential can be calculated directly,
combining above theoretical formulas and analytical methods is more
effective. Considering only the relevant states and control inputs, the
linearized state equation is

$$\left\lbrack \delta{\dot{q}}_{b} \right\rbrack = \begin{bmatrix}
A_{b}^{- 1} & - A_{b}^{- 1}dhdq\ 
\end{bmatrix}\begin{bmatrix}
\delta h_{com} \\
\delta q_{b} \\
\delta q_{j}
\end{bmatrix} + \lbrack 0,\ \ \  - A_{b}^{- 1}A_{j}\rbrack\begin{bmatrix}
\delta f \\
\delta{\dot{q}}_{j}
\end{bmatrix}$$

$$A_{B} = \begin{bmatrix}
A_{b}^{- 1} & - A_{b}^{- 1}dhdq\ 
\end{bmatrix},\ \ \ B_{B} = \lbrack 0,\ \ \  - A_{b}^{- 1}A_{j}\rbrack$$

3.1.C Transcription of Task dynamics

3.1.D Dynamics Transcription Summary

If task dynamics are not considered, then

$$\begin{bmatrix}
\delta{\dot{h}}_{com} \\
\delta{\dot{q}}_{b} \\
\delta{\dot{q}}_{j}
\end{bmatrix} = \begin{bmatrix}
0 & \begin{bmatrix}
0 \\
A_{G}
\end{bmatrix} \\
A_{b}^{- 1} & - A_{b}^{- 1}dhdq \\
0 & 0
\end{bmatrix}\begin{bmatrix}
\delta h_{com} \\
\begin{bmatrix}
\delta q_{b} \\
\delta q_{j}
\end{bmatrix}
\end{bmatrix} + \begin{bmatrix}
B_{G} & 0 \\
0 & - A_{b}^{- 1}A_{j} \\
0 & I
\end{bmatrix}\begin{bmatrix}
\delta f \\
\delta{\dot{q}}_{j}
\end{bmatrix}$$

### IV. Real-time Implementations {#iv.-real-time-implementations .标题3z}

Refer directly to Project 2 and
[**CTrjGen.jl**](https://github.com/QingtanZeng/CTrjGen.jl) \[5\].

### V. Reference {#v.-reference .标题3z}

\[1\] Sleiman, J. P., Farshidian, F., Minniti, M. V., & Hutter, M.
(2021). A unified mpc framework for whole-body dynamic locomotion and
manipulation. IEEE Robotics and Automation Letters, 6(3), 4688-4695.

\[2\] Manuel Yves Galliker, Whole-body Humanoid MPC: Realtime
Physics-Based Procedural Loco-Manipulation Planning and Control,
\[<https://github.com/1x-technologies/wb_humanoid_mpc>\].

\[3\] Carpentier, J., & Mansard, N. (2018, June). Analytical derivatives
of rigid body dynamics algorithms. In Robotics: Science and systems (RSS
2018).

\[4\] Farshidian, F. (2023). OCS2: An open-source library for optimal
control of switched systems. Accessed: May, 23.

\[5\] Qingtan Zeng, Computational Trajectory Generation,
\[https://github.com/QingtanZeng/CTrjGen.jl\]
