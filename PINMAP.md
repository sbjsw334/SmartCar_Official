# H题引脚映射

主控为 LP-MSPM0G3507 LaunchPad。此表只保留 H 题实际使用和比赛现场调试所需的引脚。

| 模块 | 信号 | MSPM0G3507 引脚 |
| --- | --- | --- |
| 下载调试 | SWDIO / SWCLK / RESET | PA19 / PA20 / NRST |
| TB6612 左电机 | PWMA / AIN1 / AIN2 | PB1 / PA12 / PA13 |
| TB6612 右电机 | PWMB / BIN1 / BIN2 | PB13 / PA14 / PA15 |
| TB6612 | STBY | PA17 |
| 左编码器 | A / B | PB10 / PB11 |
| 右编码器 | A / B | PB6 / PB7 |
| 8 路红外循迹 | OUT1..OUT8 | PB0, PB4, PB15, PB16, PB17, PB18, PB19, PB24 |
| OLED I2C | SCL / SDA | PB2 / PB3 |
| K230 UART3 | G3507 TX / RX | PB12 / PA25 |
| 四按键 | START / MODE / PLUS / MINUS | PA18 / PA22 / PA24 / PA27 |
| LDX-218 摆杆舵机 | PWM | PB20 |
| PC 调试串口 | TX / RX | PA10 / PA11 |

## K230 接线

```text
K230 UART_TX -> G3507 PA25 / UART3_RX
K230 UART_RX <- G3507 PB12 / UART3_TX
K230 GND     -> G3507 GND
波特率       = 115200, 8N1
协议         = B,<offset_mm>,<valid>\n
```

录像控制 TX 改用板上可接出的 `PB12 / UART3_TX`，视觉数据 RX 使用 `PA25 / UART3_RX`。

## 四按键

四个按键均使用内部上拉，按下时接地：

```text
PA18 START  启动或停止当前测试
PA22 MODE   停止状态下切换 H 题测试项目
PA24 PLUS   目标钢球位置增加 10 mm
PA27 MINUS  目标钢球位置减少 10 mm
```

PA18 受 LaunchPad J15 跳线影响，装车前必须确认 BoosterPack 对应位置实际连接 PA18。

## 摆杆机构

```text
LDX-218：PB20 输出 50 Hz PWM，中心脉宽 1600 us，软件安全范围 1100~1900 us
```

舵机必须使用独立 6.0~8.4 V 电源，并与 MSPM0、K230 共地。PB20 只接舵机 PWM 信号；PA31、PA28 本题不使用。

OLED 使用 PB2/PB3 的 I2C1，负责显示测试模式、目标位置和行驶总时间，屏幕尺寸不得超过 2 英寸。
