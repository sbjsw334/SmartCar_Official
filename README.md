# SmartCar_Official

MSPM0G3507 的 H 题主控工程，只保留比赛主链路：

- TB6612 双电机方向和 PWM 控制
- 双编码器计数及 10 ms 速度估计
- 八路循迹输入、3 帧滤波和基础循迹
- 摆杆舵机 PWM
- K230 钢球位置 UART 接收及超时保护
- 启动按键、调试串口、消息队列和 1 ms SysTick 调度

## 串口命令

```text
1..5  选择题目模式
s     启动或停止
x     立即停止
```

K230 协议：

```text
B,<offset_px>,<valid>\n
```

`offset_px` 是像素偏差，不是厘米。

## 当前控制状态

基础循迹已经启用，参数位于 `Application/trace_control.c`。轮速 PI、钢球位置 PD 和模式 2~5 的完整任务流程尚未实现。

首次上电应架空车轮，先确认电机方向、灰度位序和舵机安全范围。

## 环境

- MSPM0G3507，LQFP-64
- MSPM0 SDK 2.10.00.04
- TI Arm Clang 5.1.1.LTS
- 输出：`Debug/SmartCar_Official.out`

接线见 `PINMAP.md`，代码分层见 `docs/ARCHITECTURE.md`。
