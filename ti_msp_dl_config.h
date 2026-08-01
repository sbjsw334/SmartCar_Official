/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)


#define CPUCLK_FREQ                                                     32000000



/* Defines for PWM_MOTOR_A */
#define PWM_MOTOR_A_INST                                                   TIMA1
#define PWM_MOTOR_A_INST_IRQHandler                             TIMA1_IRQHandler
#define PWM_MOTOR_A_INST_INT_IRQN                               (TIMA1_INT_IRQn)
#define PWM_MOTOR_A_INST_CLK_FREQ                                        1000000
/* GPIO defines for channel 1 */
#define GPIO_PWM_MOTOR_A_C1_PORT                                           GPIOB
#define GPIO_PWM_MOTOR_A_C1_PIN                                    DL_GPIO_PIN_1
#define GPIO_PWM_MOTOR_A_C1_IOMUX                                (IOMUX_PINCM13)
#define GPIO_PWM_MOTOR_A_C1_IOMUX_FUNC               IOMUX_PINCM13_PF_TIMA1_CCP1
#define GPIO_PWM_MOTOR_A_C1_IDX                              DL_TIMER_CC_1_INDEX

/* Defines for PWM_MOTOR_B */
#define PWM_MOTOR_B_INST                                                   TIMA0
#define PWM_MOTOR_B_INST_IRQHandler                             TIMA0_IRQHandler
#define PWM_MOTOR_B_INST_INT_IRQN                               (TIMA0_INT_IRQn)
#define PWM_MOTOR_B_INST_CLK_FREQ                                        1000000
/* GPIO defines for channel 1 */
#define GPIO_PWM_MOTOR_B_C1_PORT                                           GPIOA
#define GPIO_PWM_MOTOR_B_C1_PIN                                    DL_GPIO_PIN_3
#define GPIO_PWM_MOTOR_B_C1_IOMUX                                 (IOMUX_PINCM8)
#define GPIO_PWM_MOTOR_B_C1_IOMUX_FUNC                IOMUX_PINCM8_PF_TIMA0_CCP1
#define GPIO_PWM_MOTOR_B_C1_IDX                              DL_TIMER_CC_1_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_MOTOR_B_C1_CMPL_PORT                                      GPIOB
#define GPIO_PWM_MOTOR_B_C1_CMPL_PIN                              DL_GPIO_PIN_13
#define GPIO_PWM_MOTOR_B_C1_CMPL_IOMUX                           (IOMUX_PINCM30)
#define GPIO_PWM_MOTOR_B_C1_CMPL_IOMUX_FUNC        IOMUX_PINCM30_PF_TIMA0_CCP1_CMPL


/* Defines for PWM_SERVO */
#define PWM_SERVO_INST                                                    TIMG12
#define PWM_SERVO_INST_IRQHandler                              TIMG12_IRQHandler
#define PWM_SERVO_INST_INT_IRQN                                (TIMG12_INT_IRQn)
#define PWM_SERVO_INST_CLK_FREQ                                          4000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_SERVO_C0_PORT                                             GPIOB
#define GPIO_PWM_SERVO_C0_PIN                                     DL_GPIO_PIN_20
#define GPIO_PWM_SERVO_C0_IOMUX                                  (IOMUX_PINCM48)
#define GPIO_PWM_SERVO_C0_IOMUX_FUNC                IOMUX_PINCM48_PF_TIMG12_CCP0
#define GPIO_PWM_SERVO_C0_IDX                                DL_TIMER_CC_0_INDEX




/* Defines for I2C_OLED */
#define I2C_OLED_INST                                                       I2C1
#define I2C_OLED_INST_IRQHandler                                 I2C1_IRQHandler
#define I2C_OLED_INST_INT_IRQN                                     I2C1_INT_IRQn
#define I2C_OLED_BUS_SPEED_HZ                                             400000
#define GPIO_I2C_OLED_SDA_PORT                                             GPIOB
#define GPIO_I2C_OLED_SDA_PIN                                      DL_GPIO_PIN_3
#define GPIO_I2C_OLED_IOMUX_SDA                                  (IOMUX_PINCM16)
#define GPIO_I2C_OLED_IOMUX_SDA_FUNC                   IOMUX_PINCM16_PF_I2C1_SDA
#define GPIO_I2C_OLED_SCL_PORT                                             GPIOB
#define GPIO_I2C_OLED_SCL_PIN                                      DL_GPIO_PIN_2
#define GPIO_I2C_OLED_IOMUX_SCL                                  (IOMUX_PINCM15)
#define GPIO_I2C_OLED_IOMUX_SCL_FUNC                   IOMUX_PINCM15_PF_I2C1_SCL


/* Defines for UART_0 */
#define UART_0_INST                                                        UART0
#define UART_0_INST_FREQUENCY                                           32000000
#define UART_0_INST_IRQHandler                                  UART0_IRQHandler
#define UART_0_INST_INT_IRQN                                      UART0_INT_IRQn
#define GPIO_UART_0_RX_PORT                                                GPIOA
#define GPIO_UART_0_TX_PORT                                                GPIOA
#define GPIO_UART_0_RX_PIN                                        DL_GPIO_PIN_11
#define GPIO_UART_0_TX_PIN                                        DL_GPIO_PIN_10
#define GPIO_UART_0_IOMUX_RX                                     (IOMUX_PINCM22)
#define GPIO_UART_0_IOMUX_TX                                     (IOMUX_PINCM21)
#define GPIO_UART_0_IOMUX_RX_FUNC                      IOMUX_PINCM22_PF_UART0_RX
#define GPIO_UART_0_IOMUX_TX_FUNC                      IOMUX_PINCM21_PF_UART0_TX
#define UART_0_BAUD_RATE                                                (115200)
#define UART_0_IBRD_32_MHZ_115200_BAUD                                      (17)
#define UART_0_FBRD_32_MHZ_115200_BAUD                                      (23)
/* Defines for UART_K230 */
#define UART_K230_INST                                                     UART3
#define UART_K230_INST_FREQUENCY                                        32000000
#define UART_K230_INST_IRQHandler                               UART3_IRQHandler
#define UART_K230_INST_INT_IRQN                                   UART3_INT_IRQn
#define GPIO_UART_K230_RX_PORT                                             GPIOA
#define GPIO_UART_K230_TX_PORT                                             GPIOA
#define GPIO_UART_K230_RX_PIN                                     DL_GPIO_PIN_25
#define GPIO_UART_K230_TX_PIN                                     DL_GPIO_PIN_26
#define GPIO_UART_K230_IOMUX_RX                                  (IOMUX_PINCM55)
#define GPIO_UART_K230_IOMUX_TX                                  (IOMUX_PINCM59)
#define GPIO_UART_K230_IOMUX_RX_FUNC                   IOMUX_PINCM55_PF_UART3_RX
#define GPIO_UART_K230_IOMUX_TX_FUNC                   IOMUX_PINCM59_PF_UART3_TX
#define UART_K230_BAUD_RATE                                             (115200)
#define UART_K230_IBRD_32_MHZ_115200_BAUD                                   (17)
#define UART_K230_FBRD_32_MHZ_115200_BAUD                                   (23)
/* Defines for UART_JY61P */
#define UART_JY61P_INST                                                    UART1
#define UART_JY61P_INST_FREQUENCY                                       32000000
#define UART_JY61P_INST_IRQHandler                              UART1_IRQHandler
#define UART_JY61P_INST_INT_IRQN                                  UART1_INT_IRQn
#define GPIO_UART_JY61P_RX_PORT                                            GPIOA
#define GPIO_UART_JY61P_TX_PORT                                            GPIOA
#define GPIO_UART_JY61P_RX_PIN                                     DL_GPIO_PIN_9
#define GPIO_UART_JY61P_TX_PIN                                     DL_GPIO_PIN_8
#define GPIO_UART_JY61P_IOMUX_RX                                 (IOMUX_PINCM20)
#define GPIO_UART_JY61P_IOMUX_TX                                 (IOMUX_PINCM19)
#define GPIO_UART_JY61P_IOMUX_RX_FUNC                  IOMUX_PINCM20_PF_UART1_RX
#define GPIO_UART_JY61P_IOMUX_TX_FUNC                  IOMUX_PINCM19_PF_UART1_TX
#define UART_JY61P_BAUD_RATE                                            (115200)
#define UART_JY61P_IBRD_32_MHZ_115200_BAUD                                  (17)
#define UART_JY61P_FBRD_32_MHZ_115200_BAUD                                  (23)





/* Port definition for Pin Group MOTOR_GPIOA_OUT */
#define MOTOR_GPIOA_OUT_PORT                                             (GPIOA)

/* Defines for AIN1: GPIOA.12 with pinCMx 34 on package pin 5 */
#define MOTOR_GPIOA_OUT_AIN1_PIN                                (DL_GPIO_PIN_12)
#define MOTOR_GPIOA_OUT_AIN1_IOMUX                               (IOMUX_PINCM34)
/* Defines for AIN2: GPIOA.13 with pinCMx 35 on package pin 6 */
#define MOTOR_GPIOA_OUT_AIN2_PIN                                (DL_GPIO_PIN_13)
#define MOTOR_GPIOA_OUT_AIN2_IOMUX                               (IOMUX_PINCM35)
/* Defines for BIN1: GPIOA.14 with pinCMx 36 on package pin 7 */
#define MOTOR_GPIOA_OUT_BIN1_PIN                                (DL_GPIO_PIN_14)
#define MOTOR_GPIOA_OUT_BIN1_IOMUX                               (IOMUX_PINCM36)
/* Defines for BIN2: GPIOA.15 with pinCMx 37 on package pin 8 */
#define MOTOR_GPIOA_OUT_BIN2_PIN                                (DL_GPIO_PIN_15)
#define MOTOR_GPIOA_OUT_BIN2_IOMUX                               (IOMUX_PINCM37)
/* Defines for STBY: GPIOA.17 with pinCMx 39 on package pin 10 */
#define MOTOR_GPIOA_OUT_STBY_PIN                                (DL_GPIO_PIN_17)
#define MOTOR_GPIOA_OUT_STBY_IOMUX                               (IOMUX_PINCM39)
/* Port definition for Pin Group GRAY_GPIOB_IN */
#define GRAY_GPIOB_IN_PORT                                               (GPIOB)

/* Defines for OUT1: GPIOB.0 with pinCMx 12 on package pin 47 */
#define GRAY_GPIOB_IN_OUT1_PIN                                   (DL_GPIO_PIN_0)
#define GRAY_GPIOB_IN_OUT1_IOMUX                                 (IOMUX_PINCM12)
/* Defines for OUT2: GPIOB.4 with pinCMx 17 on package pin 52 */
#define GRAY_GPIOB_IN_OUT2_PIN                                   (DL_GPIO_PIN_4)
#define GRAY_GPIOB_IN_OUT2_IOMUX                                 (IOMUX_PINCM17)
/* Defines for OUT3: GPIOB.15 with pinCMx 32 on package pin 3 */
#define GRAY_GPIOB_IN_OUT3_PIN                                  (DL_GPIO_PIN_15)
#define GRAY_GPIOB_IN_OUT3_IOMUX                                 (IOMUX_PINCM32)
/* Defines for OUT4: GPIOB.16 with pinCMx 33 on package pin 4 */
#define GRAY_GPIOB_IN_OUT4_PIN                                  (DL_GPIO_PIN_16)
#define GRAY_GPIOB_IN_OUT4_IOMUX                                 (IOMUX_PINCM33)
/* Defines for OUT5: GPIOB.17 with pinCMx 43 on package pin 14 */
#define GRAY_GPIOB_IN_OUT5_PIN                                  (DL_GPIO_PIN_17)
#define GRAY_GPIOB_IN_OUT5_IOMUX                                 (IOMUX_PINCM43)
/* Defines for OUT6: GPIOB.18 with pinCMx 44 on package pin 15 */
#define GRAY_GPIOB_IN_OUT6_PIN                                  (DL_GPIO_PIN_18)
#define GRAY_GPIOB_IN_OUT6_IOMUX                                 (IOMUX_PINCM44)
/* Defines for OUT7: GPIOB.19 with pinCMx 45 on package pin 16 */
#define GRAY_GPIOB_IN_OUT7_PIN                                  (DL_GPIO_PIN_19)
#define GRAY_GPIOB_IN_OUT7_IOMUX                                 (IOMUX_PINCM45)
/* Defines for OUT8: GPIOB.24 with pinCMx 52 on package pin 23 */
#define GRAY_GPIOB_IN_OUT8_PIN                                  (DL_GPIO_PIN_24)
#define GRAY_GPIOB_IN_OUT8_IOMUX                                 (IOMUX_PINCM52)
/* Port definition for Pin Group ENCODER_GPIOB_IN */
#define ENCODER_GPIOB_IN_PORT                                            (GPIOB)

/* Defines for LEFT_A: GPIOB.10 with pinCMx 27 on package pin 62 */
// pins affected by this interrupt request:["LEFT_A","LEFT_B","RIGHT_A","RIGHT_B"]
#define ENCODER_GPIOB_IN_INT_IRQN                               (GPIOB_INT_IRQn)
#define ENCODER_GPIOB_IN_INT_IIDX               (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define ENCODER_GPIOB_IN_LEFT_A_IIDX                        (DL_GPIO_IIDX_DIO10)
#define ENCODER_GPIOB_IN_LEFT_A_PIN                             (DL_GPIO_PIN_10)
#define ENCODER_GPIOB_IN_LEFT_A_IOMUX                            (IOMUX_PINCM27)
/* Defines for LEFT_B: GPIOB.11 with pinCMx 28 on package pin 63 */
#define ENCODER_GPIOB_IN_LEFT_B_IIDX                        (DL_GPIO_IIDX_DIO11)
#define ENCODER_GPIOB_IN_LEFT_B_PIN                             (DL_GPIO_PIN_11)
#define ENCODER_GPIOB_IN_LEFT_B_IOMUX                            (IOMUX_PINCM28)
/* Defines for RIGHT_A: GPIOB.6 with pinCMx 23 on package pin 58 */
#define ENCODER_GPIOB_IN_RIGHT_A_IIDX                        (DL_GPIO_IIDX_DIO6)
#define ENCODER_GPIOB_IN_RIGHT_A_PIN                             (DL_GPIO_PIN_6)
#define ENCODER_GPIOB_IN_RIGHT_A_IOMUX                           (IOMUX_PINCM23)
/* Defines for RIGHT_B: GPIOB.7 with pinCMx 24 on package pin 59 */
#define ENCODER_GPIOB_IN_RIGHT_B_IIDX                        (DL_GPIO_IIDX_DIO7)
#define ENCODER_GPIOB_IN_RIGHT_B_PIN                             (DL_GPIO_PIN_7)
#define ENCODER_GPIOB_IN_RIGHT_B_IOMUX                           (IOMUX_PINCM24)
/* Port definition for Pin Group KEY_GPIOA_IN */
#define KEY_GPIOA_IN_PORT                                                (GPIOA)

/* Defines for START: GPIOA.18 with pinCMx 40 on package pin 11 */
#define KEY_GPIOA_IN_START_PIN                                  (DL_GPIO_PIN_18)
#define KEY_GPIOA_IN_START_IOMUX                                 (IOMUX_PINCM40)
/* Defines for MODE: GPIOA.22 with pinCMx 47 on package pin 18 */
#define KEY_GPIOA_IN_MODE_PIN                                   (DL_GPIO_PIN_22)
#define KEY_GPIOA_IN_MODE_IOMUX                                  (IOMUX_PINCM47)
/* Defines for PLUS: GPIOA.24 with pinCMx 54 on package pin 25 */
#define KEY_GPIOA_IN_PLUS_PIN                                   (DL_GPIO_PIN_24)
#define KEY_GPIOA_IN_PLUS_IOMUX                                  (IOMUX_PINCM54)
/* Defines for MINUS: GPIOA.27 with pinCMx 60 on package pin 31 */
#define KEY_GPIOA_IN_MINUS_PIN                                  (DL_GPIO_PIN_27)
#define KEY_GPIOA_IN_MINUS_IOMUX                                 (IOMUX_PINCM60)




/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_MOTOR_A_init(void);
void SYSCFG_DL_PWM_MOTOR_B_init(void);
void SYSCFG_DL_PWM_SERVO_init(void);
void SYSCFG_DL_I2C_OLED_init(void);
void SYSCFG_DL_UART_0_init(void);
void SYSCFG_DL_UART_K230_init(void);
void SYSCFG_DL_UART_JY61P_init(void);

void SYSCFG_DL_SYSTICK_init(void);

bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
