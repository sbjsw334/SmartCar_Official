# 当前引脚映射

| 模块 | 信号 | MSPM0G3507 引脚 |
| --- | --- | --- |
| TB6612 A | PWMA / AIN1 / AIN2 | PA16 / PA31 / PB24 |
| TB6612 B | PWMB / BIN1 / BIN2 | PB20 / PB0 / PB1 |
| TB6612 | STBY | PA8 |
| 左编码器 | A / B | PA12 / PB21 |
| 右编码器 | A / B | PA26 / PA29 |
| 八路循迹 | D1..D8 | PB17, PB18, PB26, PB27, PA7, PB15, PB11, PB12 |
| 摆杆舵机 | SERVO1 / SERVO2 | PB13 / PB14 |
| 调试串口 | TX / RX | PA10 / PA11 |
| K230 UART | G3507 TX / RX | PB2 / PB3 |
| 启动按键 | 低电平按下 | PA18 |

## K230 接线

```text
K230 GPIO5 / UART2_TX -> G3507 PB3 / UART3_RX
K230 GPIO6 / UART2_RX <- G3507 PB2 / UART3_TX
K230 GND              -> G3507 GND
波特率                 = 115200, 8N1
协议                   = B,<offset_px>,<valid>\n
```

`offset_px` 是钢球相对图像中心的像素偏差，约为 `-544..544`。

PB17/PB18 已用于循迹 D1/D2，因此本工程不启用 OLED。舵机使用独立电源，K230、舵机电源和 G3507 必须共地。
