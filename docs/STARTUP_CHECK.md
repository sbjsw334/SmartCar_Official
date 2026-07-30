# Startup Check

## SysConfig names

The eight infrared inputs are one GPIO group named `GRAY_GPIOB_IN`.
Expand that group to see OUT1 through OUT8:

```text
OUT1 PB0
OUT2 PB4
OUT3 PB15
OUT4 PB16
OUT5 PB17
OUT6 PB18
OUT7 PB19
OUT8 PB24
```

The OLED controller is named `I2C_OLED`. It uses I2C1 at 400 kHz:

```text
SCL PB2
SDA PB3
SSD1306 address 0x3C
```

## Power-on behavior

The car powers up in the safe STOP state. Motors do not run until START on
PA18 is pressed for at least 20 ms.

```text
PRESS START  application is alive and waiting
KEY 1        START is detected; MODE/PLUS/MINUS are bits 1/2/3
GRAY xx 8CH  current filtered eight-channel infrared value
TIMER RUN    lap timer is running
TIME LOCKED  finish line was detected and the result is held
```

Lap timing uses the configured 1 ms SysTick. RTC is intentionally not used:
the requirement is elapsed time from the START press, not calendar time.
