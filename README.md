# SmartCar_Official

MSPM0G3507 的 H 题主控工程，只保留比赛主链路：

- TB6612 双电机方向和 PWM 控制
- 双编码器计数和 10 ms 速度估计
- 8 路红外循迹、3 帧滤波和基础循迹控制
- K230 钢球位置 UART 接收及超时保护
- 单个 LDX-218 摆杆舵机，PB20 输出 50 Hz PWM
- 四按键消抖和 START/MODE/PLUS/MINUS 功能映射
- OLED SSD1306 状态、目标位置、视觉位置和行驶时间显示
- 调试串口、消息队列、1 ms SysTick 和五段状态机加单间三架构

## 串口命令

```text
2..6  选择 H 题对应测试模式
s     启动或停止
x     立即停止
```

K230 协议：

```text
B,<offset_mm>,<valid>\n
```

`offset_mm` 是钢球相对 O 点的毫米偏差，左负右正。K230 接到 PA26/PA25 的 UART3，详见 `PINMAP.md`。

## 当前状态

引脚、底层 BSP、消息映射和状态机地基已经接通。仍需由固件负责人完成：

- H3 摆杆位置控制器实车调参
- H4~H6 行驶中滚球稳定流程
- 轮速闭环和完整赛道调参

首次上电必须架空车轮，先确认电机方向、编码器方向、8 路红外顺序、四按键和执行器安全范围。

上电默认选择 H2 并自动启动循迹。按 START 可停止；停止状态下按 MODE 在 H2 到 H6 间切换，再按 START 启动所选题目。H2 使用编码器累计脉冲防止误停，达到预计一圈的 80% 后才允许识别 A 线；确认 A 线后继续循迹 1 秒，再停车并冻结计时。

OLED 固定显示六行：

```text
H2  RUN           当前题目和 STOP/RUN/DONE/FAULT 状态
TIME 0012.34 S    本次运行时间，停车后冻结
ENC 043837        左右编码器绝对计数的平均值
TARGET +000 MM    钢球目标位置
BALL +000 MM V1   钢球毫米偏差；V1 有效，V0 无效
GRAY FF           8 路红外状态，1 表示检测到黑线
```

## 环境

- LP-MSPM0G3507，LQFP-64
- MSPM0 SDK 2.10.00.04
- SysConfig 1.28.0
- TI Arm Clang 5.1.1.LTS

接线见 `PINMAP.md`，代码分层见 `docs/ARCHITECTURE.md`，H 题状态规划和队友任务见 `docs/H_TASK_ARCHITECTURE.md`。
