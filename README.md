# Active_Wheel_Camber_Control_Strategy_for_Optimal_Lap_Time
Active wheel camber control strategy for optimal lap time (race car chassis control). Code comments and documentation are primarily in Chinese, as this work targets a Chinese journal — English-speaking researchers are very welcome to reach out via Issues.
# 面向圈速最优的主动车轮外倾控制策略

本仓库为论文《面向圈速最优的主动车轮外倾控制策略》配套的建模与仿真代码。

## About This Repository (English)

This repository accompanies the paper *"Active Wheel Camber Control Strategy for Optimal Lap Time"*. The goal of this work is to propose a lap-time-optimal active wheel camber control strategy for race cars, addressing the nonlinear coupling within the driver-vehicle-track system under track driving conditions.

**Please note:** as this work is intended for submission to a Chinese-language journal, most code comments, variable names, and documentation in this repository are written in **Chinese**. We have not systematically adapted the codebase for English readers. That said, we sincerely welcome researchers from all backgrounds to explore, use, and discuss this work — please feel free to open an Issue if you have questions or would like to exchange ideas, and we'll do our best to help in English as well.

---

> **中文说明**：本项目主要面向国内中文期刊投稿及中文读者交流，代码注释、变量命名与文档说明**以中文为主，未做系统的英文适配**。海外读者阅读或复现代码时可能需要结合论文英文摘要与图表自行理解，敬请谅解。如有需求欢迎通过 Issue 交流。

## 研究背景

主动车轮外倾通过动态调节轮胎附着特性并主动产生轮胎力，可有效拓展赛车的动力学性能边界。本研究以2023年中国大学生方程式汽车大赛冠军赛车为对象，围绕"圈速最优"这一核心目标，提出了一套完整的主动车轮外倾控制策略设计方法，整体思路分三步：

1. **全局寻优**：搭建含主动车轮外倾的圈速优化模型，模拟赛车手-赛车-赛道系统的非线性耦合特性，求解圈速最优目标下的理想车轮外倾角
2. **影响量化**：提出适用于圈速优化模型的敏感性分析方法，基于拉格朗日乘子量化车轮外倾偏离最优值对单圈用时的影响
3. **策略构建**：以理想外倾角为样本点、敏感性分析结果为拟合权重，构建以车辆纵、侧向加速度为输入的在线主动外倾控制策略

赛车手在环测试结果表明，该策略可使赛车峰值侧向加速度提升 5.9%，最快单圈用时降低 1.7%。

## 本仓库开源的内容

**注意**：出于论文投稿及研究完整性考虑，主要开放以下内容：

- 建模过程的中文说明文档（车辆动力学方程、约束条件等对应论文正文与公式推导）
- 被动悬架与主动车轮外倾悬架赛车的圈速仿真模型
- 敏感性分析方法的实际计算代码
- 控制策略参数拟合与闭环圈速验证代码
- 部分结果后处理与可视化脚本

如需有任何问题，请联系开源数据维护人员或参考论文正文获取更多细节。

## 环境要求

- MATLAB（建议 R2024a 或以上）
- 圈速优化部分全部依赖开源符号计算框架 [CasADi](https://web.casadi.org/) 与非线性规划求解器 IPOPT（Matlab通常自带），需要读者自行下载与安装.

## 目录结构

```
.
├── 含主动车轮外倾的圈速优化建模.docx                  # 建模过程中文说明
├── 00_TrackTire&other/                    # 赛道模型与轮胎模型
├── 01_Vehiclemodel/               # 被动悬架与主动车轮外倾悬架圈速优化模型
├── 02_LamCauculate/               # 拉格朗日乘子计算模块
├── 03_ControlStrategy/               # 控制策略拟合代码
├── 04_Verification/               # 控制策略闭环测试
├── 05_PostProcessing/              # 后处理模块
├── License
└── README.md

## 使用说明

在Matlab中将本文件添加并包含在子文件下，保证在运行过程中每个代码以及产生的数据都能进行读取。
为了方便读者理解圈速仿真模型，本开源库不设置子程序，除了控制策略拟合以及后处理文件，每个.m文件都是独立运行的圈速仿真代码。

具体功能为：每个文件架中的文件具体功能为
01_Vehiclemodel/ 进行被动悬架与主动车轮外倾圈速仿真，生成passive0704.mat与control0704.mat结果文件
02_LamCauculate/ 读取control0704.mat中的主动外倾控制，计算每个控制点的权重（拉格朗日乘子），生成lag004.mat（4代表偏移0.04°，其他数字同理）。同时LagVerify验算不同偏移生成的拉格朗日乘子是否一致。
03_ControlStrategy/ 读取control0704.mat与lag004.mat文件，中的主动外倾控制与拉格朗日乘子，进行控制策略拟合。
04_Verification/ 通过拟合后的控制策略代入圈速仿真软件，分析拟合效果。（一阶的并不能收敛到容差，主要原因是并没有按照控制策略重新构建逆动力学代码，收敛性较差，但圈速会稳定在15.2082-15.2080s，并不影响结论。）
05_PostProcessing/              # 一些后处理脚本，负责对比控制效果。

## 引用

如果本仓库的代码或方法对你的研究有帮助，请引用：

```
论文名称待补充 [期刊名称待补充], 2026.
```

## 联系方式

开源代码库维护工作人员：姚全，吉林大学
Email：yaoquan21@jlu.edu.cn
