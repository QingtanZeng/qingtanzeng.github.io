---
layout: page
title: Computational Trajectory Generation
description: Sequential Convex Programming for trajectory generation with hand-parser.
img: assets/img/Proj02_image1.jpg
importance: 1
category: work
related_publications: false
github: QingtanZeng/CTrjGen.jl
---

## I. Overview on Sequential Convex Programming

**Sequential Convex Programming** (SCP) is a type of multiple-shooting
direct method for numerical optimal control problems, modeled from
autonomous systems such as Autonomous Driving, Robotic
Loco-manipulation, Rocket Landing and so on. The **Penalized Trust
Region** (PTR) is one of SCP algorithm designed by Dr. T. P. Reynolds,
Prof. B. Açıkmeşe et al. from ACL, University of Washington \[1\].

## II. PTR Design

### 2.1 PTR Overview

<p align="center">
<img alt="Proj02_image1.png"
    title="Proj02_image1.png"
    src="/assets/img/Proj02_image1.png"
    width="800px" />
</p>

PTR算法主要包含以下5个部分

i.  **初始轨迹拟合Initial trajectory guess**

使用光滑曲线拟合初始位置、关键中间位置和终止位置。在复杂环境中，离散点通常采用Search或Sample-Based算法获得。对于robot运动和操作任务，通常初始轨迹通过模仿采集、离线优化并在线样条拟合获得。

ii. **动力学线性化、离散化和归一化Linearize, discretize and normalize
    dynamic system**

线性化、离散化并归一化非线性系统动力学，即将$\dot{x} = f(x,u,p)$转录为$\delta x = A\delta x + B\delta u + S\delta p$。

iii. **约束近似Lossless convexify or approximate constraints**

通过松弛变量或等价变化，可以无损凸化特定类型的状态和输入约束，如喷嘴指向和幅值约束；\
对于其他非线性约束，则可以采用一阶线性近似、正定Hessian的二阶近似或其他凸锥函数。

iv. **成本近似Quadratic or linear approximate cost**

人工设计的成本项通常定义为状态和控制输入的L1-norm或其绝对值，如剩余质量；或L2-norm，如对角正定矩阵R定义的控制输入的二次成本项$u^{T}Ru$；\
对于追踪任务，通常定义为$\|x - x_{ref}\|$即L2-distance。

v.  **转录为子问题Transcription to subproblem**

针对Transcription过程中的Artificial Unboundedness(无界) 和
Infeasibility(不可行)，可以分别采用Virtual Control(虚拟控制)和Trust
Region(信赖域)来解决。\
随后所有Parsed cost, dynamics, constraints,
parameters用于更新标准形式的锥问题的数值结构体。

vi. **求解Solver**

将Parsed后的离散稀疏锥优化问题传递给锥优化器求解，通常需要进行Code
Generation of Specific Sparse
Pattern，即针对固定结构的优化问题进行代码生成，以加速稀疏KKT矩阵分解。

vii. **收敛准则Stopping criteria**

确定收敛准则，通常定义为相邻两次迭代过程中最大的状态变化量小于一定阈值。

### 2.2 系统动力学离散线性化

#### 2.2.1 Time-Dilation

若非线性系统动力学为

$$\dot{x}(t) = f\left( t,\ x(t),u(t) \right),\ t \in \left( 0,t_{f} \right)$$

为了实现对轨迹总时间$t_{f}$的优化，采用Time-Dilation(时间归一化)即时间区间归一化到[0,1)的建模策略：定义可逆线性变换

$$\tau:\left\lbrack 0,t_{f} \right) \rightarrow \lbrack 0,1)$$

$$\tau(t) = \frac{t}{t_{f}},\ t \in \lbrack 0,\ t_{f})$$

在自由终时最优控制问题中，将$t_{f}$视为可变参数$s_{i}$，则$t(\tau) = s_{i}\tau,\ \tau \in \lbrack 0,1)$；因此，时间归一化的系统动力学方程为

$$\frac{d}{d\tau}x(t) = \frac{dt}{d\tau}\frac{d}{dt}x(t) = s_{i}f\left( t,\ x(t),u(t) \right) ≔ F\left( \tau,\ x(\tau),u(\tau),s \right)$$

$$\dot{x}(\tau) = F(\tau,x(\tau),u(\tau),s)$$

#### 2.2.2 Linearization, Discretization, Normalization

##### A. 线性化

在任意给定的参考轨迹$(\overline{x}(\tau),\overline{u}(\tau),\overline{s})$附近，对时变非线性微分方程组进行一阶泰勒展开：

$$\delta\dot{x}(\tau) \approx A(\tau)\delta x(\tau) + B(\tau)\delta u(\tau) + S(\tau)\delta s$$

或展开为

$$\dot{x}(\tau) \approx A(\tau)x(\tau) + B(\tau)u(\tau) + S(\tau)s + d(\tau)$$

where\

$$A(\tau) ≔ \nabla_{x}F(\tau,\overline{x}(\tau),\overline{u}(\tau),\overline{s})$$

$$B(\tau) ≔ \nabla_{u}F\left( \tau,\overline{x}(\tau),\overline{u}(\tau),\overline{s} \right)$$

$$S(\tau) ≔ \nabla_{s}F\left( \tau,\overline{x}(\tau),\overline{u}(\tau),\overline{s} \right)$$

$$d(\tau) ≔ F\left( \tau,\overline{x}(\tau),\overline{u}(\tau),\overline{s} \right) - \left( A\overline{x} + B\overline{u} + S\overline{s} \right)$$

在多连杆刚体动力学中，常采用递归算法和自动微分(Automatic
Differentiation,
AD)获得函数值、Jacobian和Hessian矩阵。鉴于成本和约束表达的便捷，通常直接采用完整展开式；在部分MPC应用场景中，会使用$\delta x$增量形式。

##### B. 离散化

在Real-time OCP Programmer中通常不采用伪谱法

可采用ZOH(零阶保持)和FOH(一阶线性)对线性化微分方程进行精确离散。在间隔时长很短且控制输入变化较小的MPC场景中，ZOH可获得无因果关系的对角仿射矩阵A，并便于采用DDP/iLQR/SQP的Recursive-Riccati求解。

但是在间隔时间很长的场景中，控制输入需连续变化。FOH将控制输入信号定义为离散点之间的连续线性仿射函数(Piecewise-Affine
Function)，并通过插值获得中间状态的输入。相较ZOH，FOH有以下优势: 1)
时间间隔内存在较大变化的控制输入，2)
可以确保控制输入分段连续，因此状态光滑，3)
可添加Jerk(三阶导数)约束，以满足合理的控制器跟踪斜率限制；ZOH提供控制输入给下游执行器时，需对离散跳变信号的进行插值或滤波，无法确保规划轨迹和控制轨迹一致；4）减少ZOH离散点数量，从而减小凸问题的规模。

但是，FOH中离散点之间存在因果关系，即$\ k$节点动力学约束方程需同时包含$\{ u_{k},u_{k + 1}\}$，因此无法使用Recursive-Riccati或Block
LDLT加速KKT矩阵分解；但是后文提到的固定稀疏模式的求解器代码生成可以确保FOH依然具备Real-time应用能力。

离散点之间的线性仿射的控制输入定义为

$$u(\tau) = \sigma_{k}^{-}(\tau)u_{k} + \sigma_{k}^{+}(\tau)u_{k + 1},\ \tau \in \left\lbrack \tau_{k},\tau_{k + 1} \right)$$

where

$$\sigma_{k}^{-}(\tau) = \frac{\tau_{k + 1} - \tau}{\tau_{k + 1} - \tau_{k}},\ \sigma_{k}^{+}(\tau) = \frac{\tau - \tau_{k}}{\tau_{k + 1} - \tau_{k}},\ k = 1:N - 1$$
代入，线性化的LTV动力系统的表达式为

$$\delta\dot{x}(\tau) \approx A(\tau)\delta x(\tau) + B(\tau)\sigma_{k}^{-}(\tau){\delta u}_{k} + B(\tau)\sigma_{k}^{+}(\tau){\delta u}_{k + 1} + S(\tau)\delta s$$

则时间区间$(\tau_{k},\ \tau)$内的状态转移方程为

$$\Delta x(\tau) = \Phi\left( \tau,\tau_{k} \right)\Delta x\left( \tau_{k} \right) + \int_{\tau_{k}}^{\tau}$$

$${\Phi(\tau,\zeta)\left\{ B(\zeta)\sigma_{k}^{-}(\zeta){\Delta u}_{k} + B(\zeta)\sigma_{k}^{+}(\zeta){\Delta u}_{k + 1} + S(\tau)\Delta s \right\} d\zeta}$$

式中，$\Delta\square_{k} = \square_{k} - \overline{\square}$，与$\delta$等价但代表优化过程中更大幅度的变量改变；$\Phi\left( \tau,\tau_{k} \right)$是状态转移矩阵，其遵循以下公式:

$${\Phi(\tau,\tau) = I,\ \Phi^{- 1}\left( \tau,\tau_{k} \right) = \Phi\left( \tau_{k},\tau \right)}$$

$${\frac{\partial}{\partial\tau}\Phi\left( \tau,\tau_{k} \right) = A(\tau)\Phi\left( \tau,\tau_{k} \right)}$$

在时间区间$(\tau_{k},\ \tau_{k + 1}^{-})$内对进行积分，可以获得线性仿射形式的状态空间方程

$$\Delta x\left( \tau_{k + 1}^{-} \right) = A_{k}\Delta x_{k} + B_{k}^{-}{\Delta u}_{k} + B_{k}^{+}{\Delta u}_{k + 1} + S_{k}\Delta s$$

所有函数值、线性离散矩阵的积分和导数如下

$$\overline{x}\left( \tau_{k + 1}^{-} \right) = \overline{x}\left( \tau_{k} \right) + \int_{\tau_{k}}^{\tau_{k + 1}^{-}}{F\left( \tau,\overline{x}(\tau),\overline{u}(\tau),\overline{s} \right)d\tau},\ \frac{d}{d\tau}\overline{x}(\tau) = F\left( \tau,\overline{x}(\tau),\overline{u}(\tau),\overline{s} \right),\ \overline{x}\left( \tau_{k} \right) = \overline{x}\left( \tau_{k} \right)$$

$${A_{k} = \Phi_{k} = I + \int_{\tau_{k}}^{\tau_{k + 1}^{-}}{A(\tau)\Phi\left( \tau,\tau_{k} \right)d\tau}:\ \ \ \ \ \frac{d}{d\tau}\Phi\left( \tau,\tau_{k} \right) = A(\tau)\Phi\left( \tau,\tau_{k} \right),\ \Phi\left( \tau_{k},\tau_{k} \right) = \mathbf{I}
}$$

$${B_{k}^{-} = \int_{\tau_{k}}^{\tau_{k + 1}^{-}}{A(\tau)\Phi_{B}^{-}(\tau) + B(\tau)\sigma_{k}^{-}(\tau)d\tau}:\ \ \ \ \ \frac{d}{d\tau}\Phi_{B}^{-}(\tau) = A(\tau)\Phi_{B}^{-}(\tau) + B(\tau)\sigma_{k}^{-}(\tau),\ \Phi_{B}^{-}\left( \tau_{k} \right) = \mathbf{0}
}$$

$${B_{k}^{+} = \int_{\tau_{k}}^{\tau_{k + 1}^{-}}{A(\tau)\Phi_{B}^{+}(\tau) + B(\tau)\sigma_{k}^{+}(\tau)d\tau}:\ \ \ \ \ \frac{d}{d\tau}\Phi_{B}^{+}(\tau) = A(\tau)\Phi_{B}^{+}(\tau) + B(\tau)\sigma_{k}^{+}(\tau),\ \Phi_{B}^{+}\left( \tau_{k} \right) = \mathbf{0}}$$

$$S_{k} = \int_{\tau_{k}}^{\tau_{k + 1}^{-}}{A(\tau)\Phi_{S}(\tau) + S(\tau)d\tau}:\ \ \ \ \ \frac{d}{d\tau}\Phi_{S}(\tau) = A(\tau)\Phi_{S}(\tau) + S(\tau),\ S(\tau_{k}) = \mathbf{0}$$

然而由于线性化误差、离散化的数值积分误差、约束违反，会导致相邻区间边界上，参考状态$\overline{x}(\tau_{k + 1})$、$F(\tau)$积分前推预测值$\overline{x}(\tau_{k + 1}^{-})$、实时优化变量$x(\tau_{k + 1})$并不一致。其中，子问题优化收敛后的轨迹，即参考状态$\overline{x}(\tau_{k + 1})$和前推预测值$\overline{x}(\tau_{k + 1}^{-})$的误差称为$defect$:
$$\Delta_{k} = \left| \left| \overline{x}\left( \tau_{k + 1} \right) - \overline{x}\left( \tau_{k + 1}^{-} \right) \right| \right|,\ k = 1,\ldots,N - 1$$
若收敛后的轨迹是可行的(Feasible)，即状态、输入、参数轨迹满足非线性动力学，则defects应当收敛至0。因此，defects可以作为可行轨迹的充分必要条件。

<p align="center">
<img alt="Defects"
    title="Defects"
    src="/assets/img/Proj02_image2.png"
    width="400px" />
</p>

需要注意的是，公式中$\Delta x\left( \tau_{k + 1}^{-} \right) = x\left( \tau_{k + 1}^{-} \right) - \overline{x}\left( \tau_{k + 1}^{-} \right)$，是与预测值的差值而非参考状态$\overline{x}(\tau_{k + 1})$。尽管在部分MPC场景中可直接使用，但对于长时间轨迹生成，离散化方程展开为

$${x\left( \tau_{k + 1}^{-} \right) - \overline{x}\left( \tau_{k + 1}^{-} \right) = A_{k}x\left( \tau_{k} \right) + B_{k}^{-}u_{k} + B_{k}^{+}u_{k + 1} + S_{k}s - \left( A_{k}{\overline{x}}_{k} + B_{k}^{-}{\overline{u}}_{k} + B_{k}^{+}{\overline{u}}_{k + 1} + S_{k}\overline{s} \right)
}$$

$${x\left( \tau_{k + 1}^{-} \right) = A_{k}x\left( \tau_{k} \right) + B_{k}^{-}u_{k} + B_{k}^{+}u_{k + 1} + S_{k}s + \lbrack\overline{x}\left( \tau_{k + 1}^{-} \right) - \left( A_{k}{\overline{x}}_{k} + B_{k}^{-}{\overline{u}}_{k} + B_{k}^{+}{\overline{u}}_{k + 1} + S_{k}\overline{s} \right)\rbrack
}$$

令

$${d_{k} = \lbrack\overline{x}\left( \tau_{k + 1}^{-} \right) - \left( A_{k}{\overline{x}}_{k} + B_{k}^{-}{\overline{u}}_{k} + B_{k}^{+}{\overline{u}}_{k + 1} + S_{k}\overline{s} \right)\rbrack
}$$

，且实时优化变量$x(\tau_{k + 1})$和$x(\tau_{k})$之间应满足上式，即
$x\left( \tau_{k + 1}^{-} \right) ≔ \ x(\tau_{k + 1})$，则展开后的线性离散化的动力学方程如下,

$$x\left( \tau_{k + 1} \right) = A_{k}x\left( \tau_{k} \right) + B_{k}^{-}u_{k} + B_{k}^{+}u_{k + 1} + S_{k}s + d_{k}$$

##### C. Propagation 数值积分

为获得非线性动力学和LTV精确离散方程的函数值和矩阵，需要采用数值积分的方式向前递推获得。由于f分别对{$\overline{x}(\tau),\ \Phi\left( \tau,\tau_{k} \right),\ \Phi_{B}^{-}(\tau),\Phi_{B}^{+}(\tau),\Phi_{S}(\tau)$}进行积分离散，因此可以将5个变量和积分公式向量化为1维数组，其状态值、微分值和初始值如下,

$$P(\tau) = \begin{bmatrix}
\overline{x}(\tau) \\
\Phi\left( \tau,\tau_{k} \right) \\
\begin{matrix}
\Phi_{B}^{-}(\tau) \\
\Phi_{B}^{+}(\tau) \\
\Phi_{S}(\tau)
\end{matrix}
\end{bmatrix},\ \dot{P}(\tau) = \begin{bmatrix}
F\left( \tau,\overline{x}(\tau),\overline{u}(\tau),\overline{s} \right) \\
A(\tau)\Phi\left( \tau,\tau_{k} \right) \\
\begin{matrix}
A(\tau)\Phi_{B}^{-}(\tau) + B(\tau)\sigma_{k}^{-}(\tau) \\
A(\tau)\Phi_{B}^{+}(\tau) + B(\tau)\sigma_{k}^{+}(\tau) \\
A(\tau)\Phi_{S}(\tau) + S(\tau)
\end{matrix}
\end{bmatrix},\ P\left( \tau_{k} \right) = \begin{bmatrix}
\overline{x}\left( \tau_{k} \right) \\
\mathbf{I} \\
\begin{matrix}
\mathbf{0} \\
\mathbf{0} \\
\mathbf{0}
\end{matrix}
\end{bmatrix}$$

由于$N-1$离散点之间的动力学约束并不互相依赖，因此可以将propagation任务进行多核并行计算。

### 2.3 约束线性化

与状态和控制相关的非凸等式约束的线性化可参考6.2.2。

$$h(x,u,p) \leq 0$$

对于与与状态和控制相关的非凸不等式约束,
则可以采用一阶线性不等式近似。若其二阶Hessian矩阵$\ \nabla^{2}h \geq 0\ $，则可采用二阶锥近似；如果二阶近似可大幅提高近似精度，且计算廉价，则推荐使用。

一阶近似的一般形式推导如下，在参考轨迹附近一阶展开, 

$${h\left( {\overline{x}}_{k},{\overline{u}}_{k},\overline{p} \right) + \delta h = h\left( {\overline{x}}_{k},{\overline{u}}_{k},\overline{p} \right) + \frac{\partial h}{\partial x}\Delta x + \frac{\partial h}{\partial u}\Delta u + \frac{\partial h}{\partial p}\Delta p }$$

$${= h\left( {\overline{x}}_{k},{\overline{u}}_{k},\overline{p} \right) + \nabla h\left( {\overline{x}}_{k},{\overline{u}}_{k},\overline{p} \right)\begin{bmatrix}
\Delta x \\
\Delta u \\
\Delta p
\end{bmatrix} \leq 0
}$$

$${\nabla h\left( {\overline{x}}_{k},{\overline{u}}_{k},\overline{p} \right)\begin{bmatrix}
x_{k} \\
u_{k} \\
p
\end{bmatrix} \leq - \left( h\left( {\overline{x}}_{k},{\overline{u}}_{k},\overline{p} \right) - \nabla h\left( {\overline{x}}_{k},{\overline{u}}_{k},\overline{p} \right)\begin{bmatrix}
{\overline{x}}_{k} \\
{\overline{u}}_{k} \\
\overline{p}
\end{bmatrix} \right)
}$$

$${\nabla{\overline{h}}_{k} \cdot x \leq - \overline{h}}$$

需要注意的是，离散优化问题仅在离散节点处满足硬约束，尽管FOH可大幅减少节点区间内可能的约束违反，但仍然可能存在，该现象称为Constraint
Clipping。因此，在节点上检查可行性是评估路径约束下可行性的必要条件，但并非充分条件。相比之下，基于defect的动力学轨迹可行性评估则是必要且充分的。

### 2.4 目标函数线性化

由于目标函数是人工设计的，因此通常直接采用L1-Norm,
L2-Norm和L2-Distance，以实现轨迹跟踪、成本优化、正则化等目标。

可参见具体问题的目标函数定义，如[**Computational Loco-Manipulation**](https://qingtanzeng.github.io/projects/1_project/)。

<p align="center">
<img alt="Proj02_scp_transcription"
    title="Proj02_scp_transcription"
    src="/assets/img/Proj02_scp_transcription.jpg"
    width="800px" />
</p>

### 2.5 Artificial Unboundedness and Infeasibility

#### 2.5.1 Artificial Unboundedness: Trust Region

基于线性化和凸化的子问题，通常需要满足信赖域约束条件，以确保每次迭代的子问题解都位于近似有效的临近区域。其次，对非凸项进行线性化处理可能导致子问题解出现人为无界情况，必须使用信赖域予以约束。

信赖域约束了每次迭代的变量${x_{k},u_{k},p}$偏离参考轨迹${{\overline{x}}_{k},{\overline{u}}_{k},\overline{p}}$的距离，增加以下二阶锥约束，

$${\alpha_{x}|x_{k} - {\overline{x}}_{k}| + \alpha_{u}|u_{k} - {\overline{u}}_{k}| \leq \eta_{k},\ \ \ k = 1,\ldots,N
}$$

$${\alpha_{p}\left| p - \overline{p} \right| \leq \eta_{p}
}$$

式中采用二次双范数平方限制偏离距离，$\ \eta \in R_{+}^{N},\ \eta_{p} \in R_{+}\ $分别为状态输入和可变参数的信赖域半径，需作为优化变量参与L-1
Norm目标函数中，以鼓励SCP收敛过程中将半径推进至0，

$$J_{tr} = w_{tr}^{T}\eta + w_{tr,p}\eta_{p}$$

式中$\ w_{tr} \in R_{+}^{N},\ w_{p} \in R_{+}\ $分别为信赖域半径的惩罚权重。

尽管惩罚权重可以手工设计标定，但在SCP收敛过程中可被设计为依据动力学轨迹可行性的动态更新，借用动力学轨迹propagation获得的defect，定义

<p align="center">
<img alt="Proj02_weights_of_trust_region"
    title="Proj02_weights_of_trust_region"
    src="/assets/img/Proj02_trustregion.jpg"
    width="200px" />
</p>

式中$\ \Delta_{\min}\ $为L1-Norm权重提供一个数值上界，以保持合理的缩放尺寸并避免条件数过差带来的数值问题。动态更新的权重鼓励存在较大误差的节点变量有更大的步长，而惩罚已接近收敛的节点变量尽量保持不动，但依然允许所有优化变量在信赖域内同时优化调整。在迭代过程中，Solver内环将硬约束残差逐步缩减至0，SCP外环则引导信赖域半径逐步收敛至0，从而获得可行轨迹，并避免人为无界问题。

#### 2.5.2 Artificial Infeasibility: Virtual Control

对动力学和约束进行线性化和转录后，原始问题的非空解集可能因为近似而变成空集。如果在SCP迭代过程中遇到人为不可行，则子问题无解且迭代过程终止。除此之外，固定信赖域半径也会带来人为不可行。

因此，在动力学和硬约束中加入一组无约束但L1
Norm绝对值惩罚的松弛变量，即虚拟控制(Virtual
Control)变量，以确保每个线性化和凸化的子问题必然存在解，

$${x_{k + 1} = A_{k}x_{k} + B_{k}^{-}u_{k} + B_{k}^{+}u_{k + 1} + S_{k}s + d_{k} + v_{k}}$$

$${ \overline{h}_{j} - v_{k}' \leq 0, v_{k}' \geq 0, j = 1,..,n_{ncvx} }$$

$${A_{0}x_{1} + E_{0}p + r_{0} + v_{0} = 0, A_{N}x_{N} + E_{N}p + r_{N} + v_{N} = 0 }$$

式中$\ v_{k},\ v_{k}'\ $分别为加入动力学约束和非凸约束中的虚拟控制松弛变量。

虽然类似HSDE
IPM中的残差，但是虚拟控制仅用作弥补子问题的空集问题，确保SCP迭代继续进行，而非作为defect引导满足动力学和非凸约束，因此其仅在必要时启用，施加较大的L1-Norm绝对值惩罚，

$$J_{vc} = w_{vc}(\left| v' \right| + \sum|v_{k}|)$$

SCP收敛后虚拟控制应当为0，以获得可行轨迹。

## III. Discrete OCP Problem and Solver

综上所述，线性化和凸化后的离散子优化问题可表示为，

$${\min_{x = \left( X,U,p,V,\eta,\eta_{p} \right)}\left( x^{T}Px + c^{T}x \right) + \left( w_{tr}^{T}\eta + w_{tr,p}\eta_{p} \right) + w_{vc}\left( \left| v' \right| + \sum\left| v_{k} \right| \right)
}$$

$${s.t.\ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ x_{k + 1} = A_{k}x_{k} + B_{k}^{-}u_{k} + B_{k}^{+}u_{k + 1} + S_{k}s + d_{k} + v_{k}
}$$

$${\ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \nabla{\overline{h}}_{k} \cdot x \leq - \overline{h} + v_{k}',\ \ \ v_{k}' \geq 0,\ \ \ j = 1,..,n_{ncvx}
}$$

$${\ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ A_{0}x_{1} + E_{0}p + r_{0} + v_{0} = 0,\ \ \ A_{N}x_{N} + E_{N}p + r_{N} + v_{N} = 0,\ 
}$$

$${\ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \alpha_{x}|x_{k} - {\overline{x}}_{k}| + \alpha_{u}|u_{k} - {\overline{u}}_{k}| \leq \eta_{k},\ \ \ k = 1,\ldots,N
}$$

$${\ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \alpha_{p}\left| p - \overline{p} \right| \leq \eta_{p}
}$$

将$\ N\ $个节点的离散子问题装入下列标准锥问题形式中，其中约束仅有仿射等式、非负锥和二阶锥，以实现real-time求解，

$${\min{\ \ \ \ \ \left( \frac{1}{2} \right)x^{T}Px + w^{T}x}
}$$

$${s.t.\ \ \ \ \ \ \ Ax + s = b
}$$

$${\ \ \ \ \ \ \ \ \ \ \ \ \ \ s \in K}$$

### 3.1 归一化Scaling

由于不同物理量纲和范围的变量在尺度上可能有数倍甚至数量级的差异，因此必须对所有状态变量、控制变量和可变参数进行缩放归一化，从而获得数值条件(Numerical
Conditioning)稳定和直观权重标定的离散优化问题。尽管求解器内部的IPM
NT-Scaling或DRS
Preconditioning也会进行缩放处理，但在问w题定义时的缩放有助于合理的权重标定。

$${x_{k} = S_{x}{\widehat{x}}_{k} + c_{x}
}{u_{k} = S_{u}{\widehat{u}}_{k} + c_{u}
}{p = S_{p}\widehat{p} + c_{p}
}$$

式中$\ S\ $是对角缩放矩阵，$\ c\ $是中心化向量。通常已知所有物理量的实际范围是

$${z_{i} \in \left\lbrack z_{i,min},z_{i,max} \right\rbrack}$$

，预期缩放范围是

$${ \widehat{z}_{i} \in [ \widehat{z}_{lb}, \widehat{z}_{rb} ] = [0,1]}$$

，则可推导得，

$$S_{z} = \frac{z_{i,max} - z_{i,min}}{1 - 0},\ \ \ c_{z} = z_{i,min} - S_{z}{\widehat{z}}_{lb} = z_{i,min}$$

由于在求解器内可以要求所有变量为非负值，则隐式的实现了$\ x \geq x_{\min}\ $的下界限制约束，减少了显式定义的问题维度。

将上述缩放带入到二次目标函数中，可得

$$x^{T}Px + w^{T}x = x^{T}\left( S^{T}PS \right)x + (2c^{T}P + w^{T})Sx$$

### 3.2 Parser for 二阶锥约束

通常一般形式的二阶锥约束可以如下处理，假定仅有状态相关的约束，

$$\left| Ax_{k} + c \right| \leq g^{T}x_{k} + d$$

式中$\ A \in R^{d \times n_{x}},\ c \in R^{d},\ g \in R^{n_{x}},d \in R\ $，假定二阶锥参数不随状态变化。引入辅助变量$\ \mu_{k} \in R^{d},\ \sigma_{k} \in R_{+}\ $，

$$\mu_{k} = Ax_{k} + c,\ \ \ \sigma_{k} = g^{T}x_{k} + d \rightarrow \ \begin{bmatrix}
\sigma_{k} \\
\mu_{k}
\end{bmatrix} \in C_{soc,d + 1}$$

因此一般形式的二阶锥约束被转化为仿射等式约束和标准二阶锥约束，将N个二阶锥约束集中表达，

$${\chi ≔ \left( \sigma_{1},\mu_{1},\sigma_{2},\mu_{2},\ldots\sigma_{N},\mu_{N} \right) \in R^{N(d + 1)}}$$

$${- \left( I_{N}\bigotimes A \right)x + \left( I_{N}\bigotimes\left\lbrack 0\ \ I_{d} \right\rbrack \right)\chi = 1_{N}\bigotimes c}$$

$${- \left( I_{N}\bigotimes g^{T} \right)x + \left( I_{N}\bigotimes\left\lbrack 1\ \ 0_{1 \times d} \right\rbrack \right)\chi = d \cdot 1_{N}}$$

$${(\sigma_{k},\mu_{k}) \in C_{soc,d + 1}}$$

式中$\ \bigotimes\ $为Kronecker product.

<p align="center">
<img alt="hand-parser problem"
    title="hand-parser problem"
    src="/assets/img/Proj02_image3.jpeg"
    width="1000px" />
</p>

下图为Linear SOCP标准形式问题的稀疏结构，图中灰色为NaN，白色为0.0，蓝色和紫色分别代表+1/-1，蓝色和红色分别代表+x/-x。

<p align="center">
<img alt="Sparse Matrix Visualization"
    title="Sparse Matrix Visualization"
    src="/assets/img/Proj02_trjdb_pgm_init.png"
    width="800px" />
</p>

## IV. Reference

\[1\] Reynolds, T. P. (2020). Computational guidance and control for
aerospace systems. University of Washington.\
\[2\] <https://github.com/UW-ACL/SCPToolbox.jl>\
\[3\] Kamath, A. G., Doll, J. A., Elango, P., Kim, T., Mceowen, S., Yu,
Y., \... & Açıkmeşe, B. (2025). Onboard Dual Quaternion Guidance for
Rocket Landing. arXiv preprint arXiv:2508.10439.\
\[4\] Domahidi, A., Chu, E., & Boyd, S. (2013, July). ECOS: An SOCP
solver for embedded systems. In 2013 European control conference (ECC)
(pp. 3071-3076). IEEE.\
\[5\] O\'Donoghue, B. (2021). Operator splitting for a homogeneous
embedding of the linear complementarity problem. SIAM Journal on
Optimization, 31(3), 1999-2023.
