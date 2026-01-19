---
layout: page
title: Notes on Constrained Stein Variational Inference
description: "Constrained Global Optimization through SVI: MultiModal,NonConvex-NonConnected-NonDifferentiable, Hard Constraints, GPU Acceleration."
img: assets/img/Proj04_Particles.jpg
importance: 2
category: work
related_publications: false
---

## I. Overview of Global Optimization

### 1.1 From Convex Programming to Variational Inference

凸优化理论仅能建立局部最优的一阶条件，故其收敛到局部极小值的过程对初始值高度敏感，无法有效解决全局非凸问题。而传统处理全局非凸问题的方法，如**粒子群算法(PSO)**或**马尔可夫链蒙特卡洛(MCMC)**，前者样本效率低，而后者需要大量采样，均只能应用于离线建模、估计或训练。

然而，基于**贝叶斯推断(Bayesian Inference, BI)**的**贝叶斯优化(Bayesian
Optimization,
BO)**为"采样全局优化"提供了一个完美的理论框架。其中，**变分推理(Variational
Inference,
VI)**将推断问题重新表述为优化问题，使用一组粒子对目标函数进行高效的近似，相比PSO和MCMC有若干数量级的速度提升。

**Stein Variational Inference (SVI)**是Liu, Q., & Wang,
D.提出的一种通用的变分推理方法\[1\]，通过最小化𝑞(𝑥)和𝑝(𝑥)之间的Kullback-Leibler(KL)散度，从而更新一组分布粒子𝑞(𝑥)以逼近目标分布(目标函数)
𝑝(𝑥)。2025年的SVI设计可以实现以下优势:

i.  **Multi-Modal 多模态**: 同时更新的一组粒子可同步获得一组可行解；

ii. **Non-Convex 全局非凸**: 一组粒子可最终聚集在(收敛于)多个局部极值点;

iii. **Non-Connected 非连通可行域**:
     一组粒子可覆盖和跨越整数变量等原因导致的不连通的定义域;

iv. **Non-Differentiable 不可微**:
    粒子更新方向可通过对目标函数的局部采样而非微分获得;

v.  **Real-time**: 通过GPU并行加速，可以实现在嵌入式系统中的实时部署;

vi. **Hard-Constraints**:
    可以满足零锥、非负锥、二阶锥等基本凸锥集合的约束；

vii. ...

然而当前嵌入式系统的轨迹规划器仅能使用工程手段解决非凸全局优化问题:
使用规则或数据模型生成一组初始值猜测，然后并行求解这些OCP问题，最后依次校验并选择其一。工程方案需要设计大量专家规则，严重依赖数据模型，轨迹间无交互，计算效率低且无法泛化。

因此，SVI具备的上述特征正是当前部署在边缘异构实时系统中决策和规划器亟需的算法能力。

### 1.2 当前轨迹规划方法

全局采样运动规划方法，如RRT,
PRM，可以有效的找到复杂环境下的可行路径；尽管通过投影法(Projection
Methods)和延续法(Continuation
Method)来处理约束，但无法优化目标函数，无法严格满足约束，且对于高维度动力学或运动学问题，计算成本过高。

基于优化的轨迹生成方法，如SCP,
iLQR，可以快速获得局部最优且满足硬约束的可行轨迹；但在非凸非线性的问题中，其对初始值敏感:
如果初值较差或环境变化，其无法收敛到可行解，或重规划跳出局部不可行区域。

### 1.3 应用领域

约束通常导致可行域非凸，如避免与静止和动态障碍物发生碰撞。约束同样描述了任务要求，如Manipulate要求，通常应严格执行。

目标函数定义了末端轨迹追踪的任务要求，对于高自由度的机器人，有多种完全不同的动作方案。目标函数可能为非凸，需要探索多个可行的局部最优值。

约束的非凸非线性导致可行轨迹分布于隐式定义的低维流形上，这对于基于采样的方法很难高效满足约束的采样，而凸优化方法的线性化可能导致轨迹偏离高度非线性的约束。

博弈问题，如微分动态规划，存在多个博弈均衡点，且需要实时迭代观测、估计和博弈。

上述应用场景对实时优化算法提出了以下要求：**多模态获得多个可行解，处理目标函数的非凸，满足非凸和非线性的硬约束**...

## II. Stein Variational Inference

### 2.1 Basic Design

Stein Variational Inference使用变分后验的非参数表示方法 (a
non-parametric representation of the variational posterior)，

基于SVI理论原理，算法设计应实现以下特性或性能

1)  **核函数**:
    衡量轨迹间的近似程度，能在嵌入式平台部署的复杂度和并行能力；

2)  **退火策略**: 探索 vs. 利用，先期鼓励粒子探索，后期加速粒子收敛；

3)  **重采样策略**:
    在MPC迭代过程中，由于问题发生变化，重采样重新启动求解器；

 \
除此之外，类比通用锥优化器，SVI设计还需要满足以下目标，

1)  **满足硬约束:** a.
    典型的仿射等式约束和锥集合，以及一般非凸或非线性约束; b.
    硬约束的处理通常不应通过软化为成本实现，否则会影响目标函数本身的优化；

2)  **病态问题鲁棒**: a.
    由于不同变量的尺度不同，除了上层原始离散问题的归一化，求解器本身也应做预条件数处理(precondition)，因此得分函数、核函数、更新方向和步长做相应调整。

3)  **收敛速度和复杂度保证**:

#### 1.A SVI基本原理

$$q^{*}(x) = \arg\min_{q(x)}\ KL(q(x)||p(x))$$

式中$\ x \in R^{d}\ $，$\ p,\ q\ $分别为$\ R^{d}\ $上的两个概率密度函数。SVI使用一组粒子表示分布$\ q(x)\ $，并迭代更新这组粒子以最小化 $ KL(q(x)||p(x)) $，

$$q(x) = \frac{1}{N}\sum_{i = 1}^{N}{\delta(x - x^{i})}$$

SVGD使用下式更新粒子群

$$x_{k + 1}^{i} = x_{k}^{i} + \epsilon\phi^{*}(x_{k}^{i})$$

式中，其中基于可微分正定核函数$\ Κ\ $的更新方向$\ \phi^{*}\ $被定义为

$$\phi^{*}\left( x^{i} \right) = \frac{1}{N}\sum_{j = 1}^{N}{K\left( x^{i},x^{j} \right)\mathbf{\nabla}_{\mathbf{x}^{\mathbf{j}}}\log p\left( x^{j} \right) + \mathbf{\nabla}_{\mathbf{x}^{\mathbf{j}}}K\left( x^{i},x^{j} \right)}$$

式中第一项为"Driving Force"，驱动粒子向目标分布的高概率区域集中，即寻找局部极小点；
第二项为"Repulsive Force"，互斥力使得粒子相互远离，以防坍缩至相同的局部极小区域。

因此，粒子最终会聚集在几个局部最小值附近，然后得到多模态可行和局部最优解。

#### 1.B 核函数$\mathbf{\ Κ\ }$

核函数通常有以下选择，

i.  **RBF核函数**

两个轨迹在空间中的相似性通过其距离的指数衰减衡量的，

$$k(x,y) = \sigma^{2}exp( - \frac{|x - y|^{2}}{2l^{2}})$$
式中$\ l > 0\ $是长度尺度参数，$\ \sigma\ $是信号方差参数。

ii. **多项式核函数**

$$k(x,y) = \left( \gamma(x \cdot y) + c \right)^{d}$$

iii. **有理函数核**

柯西核，

逆多象限 (Inverse Multiquadric, IMQ) 核，

$$k(x,y) = \left( c^{2} + |x - y|^{2} \right)^{- \beta}$$

通常 $\beta = 0.5$，

### 2.2 Constrained Stein Variational Inference

1\. Orthogonal-Space Stein Variational Gradient Descent (O-SVGD), a
recent non-parametric variational inference method for domains with a
single equality constraint

2\. constrained Stein Variational Gradient Descent (SVGD) algorithm
**differentiable equality and inequality constraints**. The
**trajectories are *approximately* constraint-satisfying** because we do
**not run the algorithm until convergence to avoid excessive computation
times**.

3\. We additionally incorporate **a novel re-sampling step that
re-samples and perturbs particles in the tangent space of the
constraints** to escape local minima.

### 2.3 Annealed Strategy

<https://www.youtube.com/watch?v=5zqo-JzI4J8>, **Annealed Stein
Variational Gradient Descent**

### 2.4 Scaling Direction

**Entropy Regularized Motion Planning via Stein Variational Inference**

### 2.8 Summary

## V. Reference

\[1\] Liu, Q., & Wang, D. (2016). Stein variational gradient descent: A
general purpose bayesian inference algorithm. Advances in neural
information processing systems, 29. \
\[2\] Power, T., & Berenson, D. (2024). Constrained stein variational
trajectory optimization. IEEE Transactions on Robotics.
