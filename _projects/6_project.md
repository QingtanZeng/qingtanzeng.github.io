---
layout: page
title: Inverter Modulation
author: Qingtan Zeng
description: Report on Pulse Heating
img: assets/img/Proj06_Pulseheating.png
importance: 3
category: work
related_publications: false
---

# Pulse Heating in E-Drive

## I. Introduction

正常电驱电磁控制是形成旋转磁场从而产生扭矩，且产生带有开关纹波的母线直流电流。但是脉冲加热则在静止工况下，利用电池与电机电感之间的能量交换生成交流的母线电流，进而完成在电池内部(内阻)的加热。

<p align="center">
<img alt="Pulse Heating"
    title="Pulse Heating"
    src="/assets/img/Proj06_Pulseheating.png"
    width="800px" />
</p>

尽管原理简单，但需要实现以下设计和功能，以确保安全稳定鲁棒的脉冲加热，

1.  **电流波形**:
    确定对电池内阻升温最高效的电流波形，避免大量热量被耗散在逆变器和电机上；

2.  **电流控制器设计:**
    完全不同于旋转磁场的控制，需依据波形定制化设计电流控制器;

3.  **电感饱和:**
    通常400-800V母线电压会导致很高的$\ dV/dt\ $，因此需在低载波比下稳定控制周期波形，避免电感饱和造成失稳过流；

4.  **转子过热退磁:**
    交流磁链会导致转子涡流损耗及快速温升，因此需实时估计转子温度并适时保护；

5.  **意外扭矩:** 防止静止加热时出现意外扭矩；

6.  **NVH改善:**
    由于功能理论上就存在基频相关的谐波，且造成周期径向力冲击，需协同各个环节尽可能降低终端噪音水平至可接受的范围。

7.  **...**.

