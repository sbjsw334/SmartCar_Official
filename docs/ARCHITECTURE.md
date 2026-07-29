# 软件结构

```text
SysConfig -> BSP -> Application -> main
```

## 文件职责

- `SmartCar_Official.syscfg`：时钟、GPIO、PWM、UART 和 SysTick
- `BSP`：motor、encoder、gray、servo、key、uart、k230
- `Application/msg_map`：中断和主循环之间的16项环形消息队列
- `Application/trace_control`：八路循迹和丢线恢复
- `Application/app_car`：模式、启停、数据汇总和控制入口
- `main.c`：初始化、1 ms 调度和串口命令

## 调度

```text
SysTick 1 ms
|- 按键消抖
|- 灰度采样
|- 编码器速度更新
|- K230 超时计数
|- 设置 10 ms 控制事件
`- 设置 200 ms 遥测事件
```

中断只采样或投递消息。10 ms和200 ms周期消息自动去重，其他消息按投递顺序处理；循迹、电机控制和串口打印都在主循环执行。

## 控制链

```text
八路灰度 -> trace_control -> 左右电机指令 -> TB6612
编码器   -> 速度反馈（轮速 PI 待实现）
K230     -> 钢球像素偏差（位置 PD 待实现）-> 摆杆舵机
```

## K230 协议

```text
B,<offset_px>,<valid>\n
```

超过 1 秒没有收到数据后，钢球状态自动失效。
