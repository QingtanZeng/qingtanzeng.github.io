---
layout: page
title: Active Discharge
author: Qingtan Zeng
description: Report on Active Discharge
img: assets/img/Proj07_Active Discharge.png
importance: 7
category: work
related_publications: false
---

# Project 7. Active Discharge Design

## I. Introduction

高压安全是带有高压电池包的整车系统工程中至关重要的一环，直接关系到车辆在碰撞、维修或正常下电后，能否避免对乘客、救援人员或维修技师造成电击伤害。不同厂家有各自的放电策略设计，但都必须保证功能足够鲁棒、响应迅速，在任何工况下均能放电，如低压掉电、硬件故障、带转速电压反冲等。

<p align="center">
<img alt="Active Discharge"
    title="Active Discharge"
    src="/assets/img/Proj07_Active Discharge.png"
    width="800px" />
</p>

为降低硬件成本，电控系统的绕组放电可通过电磁控制将母线电容电压降低到60V以下，但需要实现以下功能：

1.  **综合放电状态机设计**:
    除了绕组放电，通常还有硬件放电回路冗余，因此如何选择和切换放电方式；

2.  **门极驱动配合:**
    门极驱动电路已设计了ASC保护、Flyback稳压等功能，上层需要与其交互，确保放电正常进行；

3.  **继电器黏连检测**:
    继电器有可能因为高压电弧发生黏连，从而无法立即断开，因此需实时估计放电速率，确定是否存在黏连故障；

4.  **热保护**:
    若出现黏连故障等问题，应当防止放电导致逆变器或电机过热，导致电驱硬件损坏；

5.  **带转速放电**:
    车辆行驶时若出现紧急下电工况，也应快速完成放电过程，但带转速的永磁转子可能导致能量倒灌，导致电容电压反冲升高，因此如何鲁棒控制带转速放电过程的电流轨迹、母线电压和ASC软切过程至关重要。

6.  **...**
