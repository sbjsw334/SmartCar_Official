# SmartCar_Official

MSPM0G3507 的 H 题主控工程，只保留比赛主链路：

- TB6612 双电机方向和 PWM 控制
- 双编码器计数和 10 ms 速度估计
- 8 路红外循迹、3 帧滤波和基础循迹控制
- K230 钢球位置 UART 接收及超时保护
- 摆杆舵机 PWM，并预留步进 STEP/DIR/EN
- 四按键消抖和 START/MODE/PLUS/MINUS 功能映射
- OLED SSD1306 状态、目标位置、视觉位置和行驶时间显示
- 调试串口、消息队列、1 ms SysTick 和五段状态机加单间三架构

## 串口命令

```text
1..5  选择 H 题测试模式
s     启动或停止
x     立即停止
```

K230 协议：

```text
B,<offset_px>,<valid>\n
```

`offset_px` 是像素偏差，不是厘米。K230 接到 PA26/PA25 的 UART3，详见 `PINMAP.md`。

## 当前状态

引脚、底层 BSP、消息映射和状态机地基已经接通。仍需由固件负责人完成：

- 钢球像素到毫米标定
- 摆杆位置控制器及舵机/步进参数
- 轮速闭环、停车判定和完整赛道调参

首次上电必须架空车轮，先确认电机方向、编码器方向、8 路红外顺序、四按键和执行器安全范围。

## 环境

- LP-MSPM0G3507，LQFP-64
- MSPM0 SDK 2.10.00.04
- SysConfig 1.28.0
- TI Arm Clang 5.1.1.LTS

接线见 `PINMAP.md`，代码分层见 `docs/ARCHITECTURE.md`，H 题状态规划和队友任务见 `docs/H_TASK_ARCHITECTURE.md`。
