# SmartCar_Official_Plus

MSPM0G3507 H 题融合工程，当前保留 H2/H3/H4/H5 主线逻辑。

- H2：纯循迹一圈，识别回 A 线后锁当前陀螺仪航向直行，再刹车并回正
- H3：静态滚球，-50 mm 到 +50 mm 定稿逻辑
- H4：A 到 B 行驶滚球，参数保留为可调
- H5：绕行一圈滚球，参数保留为可调

## 串口命令

    2     H2 纯循迹
    3     H3 静态滚球
    4     H4 A->B 滚球
    5     H5 一圈滚球
    s/S   启动
    x/X   停止

K230 钢球位置协议：

    B,<offset_mm>,<valid>\n

offset_mm 为钢球相对 O 点的毫米偏差，左负右正。

## 调试重点

- H2 看 iy/cmd：终点后是否保持锁定航向直行，回正是否接近 0 deg
- H3 看 target/ball/speed：-50 到 +50 过程中是否过冲、抖动或丢帧
- H4/H5 看 h5b/h5af/ENC：滚球偏置、加速度前馈和里程门槛是否稳定
- 所有模式先架空确认电机方向、编码器方向、灰度顺序和急停有效

## 工程说明

Keil 工程文件在 keil/SmartCar_Official.uvprojx。CCS 版本由该 Plus 源码同步生成。
