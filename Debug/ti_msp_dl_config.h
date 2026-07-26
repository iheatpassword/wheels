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



/* Defines for PWM_motor */
#define PWM_motor_INST                                                     TIMG0
#define PWM_motor_INST_IRQHandler                               TIMG0_IRQHandler
#define PWM_motor_INST_INT_IRQN                                 (TIMG0_INT_IRQn)
#define PWM_motor_INST_CLK_FREQ                                         32000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_motor_C0_PORT                                             GPIOA
#define GPIO_PWM_motor_C0_PIN                                     DL_GPIO_PIN_23
#define GPIO_PWM_motor_C0_IOMUX                                  (IOMUX_PINCM53)
#define GPIO_PWM_motor_C0_IOMUX_FUNC                 IOMUX_PINCM53_PF_TIMG0_CCP0
#define GPIO_PWM_motor_C0_IDX                                DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_motor_C1_PORT                                             GPIOA
#define GPIO_PWM_motor_C1_PIN                                     DL_GPIO_PIN_13
#define GPIO_PWM_motor_C1_IOMUX                                  (IOMUX_PINCM35)
#define GPIO_PWM_motor_C1_IOMUX_FUNC                 IOMUX_PINCM35_PF_TIMG0_CCP1
#define GPIO_PWM_motor_C1_IDX                                DL_TIMER_CC_1_INDEX

/* Defines for PWM_servo */
#define PWM_servo_INST                                                     TIMG7
#define PWM_servo_INST_IRQHandler                               TIMG7_IRQHandler
#define PWM_servo_INST_INT_IRQN                                 (TIMG7_INT_IRQn)
#define PWM_servo_INST_CLK_FREQ                                            40000
/* GPIO defines for channel 0 */
#define GPIO_PWM_servo_C0_PORT                                             GPIOA
#define GPIO_PWM_servo_C0_PIN                                     DL_GPIO_PIN_26
#define GPIO_PWM_servo_C0_IOMUX                                  (IOMUX_PINCM59)
#define GPIO_PWM_servo_C0_IOMUX_FUNC                 IOMUX_PINCM59_PF_TIMG7_CCP0
#define GPIO_PWM_servo_C0_IDX                                DL_TIMER_CC_0_INDEX



/* Defines for TIMER_0 */
#define TIMER_0_INST                                                     (TIMG6)
#define TIMER_0_INST_IRQHandler                                 TIMG6_IRQHandler
#define TIMER_0_INST_INT_IRQN                                   (TIMG6_INT_IRQn)
#define TIMER_0_INST_LOAD_VALUE                                            (39U)




/* Defines for I2C_MPU6050 */
#define I2C_MPU6050_INST                                                    I2C1
#define I2C_MPU6050_INST_IRQHandler                              I2C1_IRQHandler
#define I2C_MPU6050_INST_INT_IRQN                                  I2C1_INT_IRQn
#define GPIO_I2C_MPU6050_SDA_PORT                                          GPIOA
#define GPIO_I2C_MPU6050_SDA_PIN                                  DL_GPIO_PIN_16
#define GPIO_I2C_MPU6050_IOMUX_SDA                               (IOMUX_PINCM38)
#define GPIO_I2C_MPU6050_IOMUX_SDA_FUNC                IOMUX_PINCM38_PF_I2C1_SDA
#define GPIO_I2C_MPU6050_SCL_PORT                                          GPIOA
#define GPIO_I2C_MPU6050_SCL_PIN                                  DL_GPIO_PIN_15
#define GPIO_I2C_MPU6050_IOMUX_SCL                               (IOMUX_PINCM37)
#define GPIO_I2C_MPU6050_IOMUX_SCL_FUNC                IOMUX_PINCM37_PF_I2C1_SCL

/* Defines for I2C_0 */
#define I2C_0_INST                                                          I2C0
#define I2C_0_INST_IRQHandler                                    I2C0_IRQHandler
#define I2C_0_INST_INT_IRQN                                        I2C0_INT_IRQn
#define GPIO_I2C_0_SDA_PORT                                                GPIOA
#define GPIO_I2C_0_SDA_PIN                                        DL_GPIO_PIN_28
#define GPIO_I2C_0_IOMUX_SDA                                      (IOMUX_PINCM3)
#define GPIO_I2C_0_IOMUX_SDA_FUNC                       IOMUX_PINCM3_PF_I2C0_SDA
#define GPIO_I2C_0_SCL_PORT                                                GPIOA
#define GPIO_I2C_0_SCL_PIN                                        DL_GPIO_PIN_31
#define GPIO_I2C_0_IOMUX_SCL                                      (IOMUX_PINCM6)
#define GPIO_I2C_0_IOMUX_SCL_FUNC                       IOMUX_PINCM6_PF_I2C0_SCL


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
#define UART_0_BAUD_RATE                                                  (9600)
#define UART_0_IBRD_32_MHZ_9600_BAUD                                       (208)
#define UART_0_FBRD_32_MHZ_9600_BAUD                                        (21)
/* Defines for UART_1 */
#define UART_1_INST                                                        UART1
#define UART_1_INST_FREQUENCY                                           32000000
#define UART_1_INST_IRQHandler                                  UART1_IRQHandler
#define UART_1_INST_INT_IRQN                                      UART1_INT_IRQn
#define GPIO_UART_1_RX_PORT                                                GPIOB
#define GPIO_UART_1_TX_PORT                                                GPIOA
#define GPIO_UART_1_RX_PIN                                         DL_GPIO_PIN_7
#define GPIO_UART_1_TX_PIN                                        DL_GPIO_PIN_17
#define GPIO_UART_1_IOMUX_RX                                     (IOMUX_PINCM24)
#define GPIO_UART_1_IOMUX_TX                                     (IOMUX_PINCM39)
#define GPIO_UART_1_IOMUX_RX_FUNC                      IOMUX_PINCM24_PF_UART1_RX
#define GPIO_UART_1_IOMUX_TX_FUNC                      IOMUX_PINCM39_PF_UART1_TX
#define UART_1_BAUD_RATE                                                  (9600)
#define UART_1_IBRD_32_MHZ_9600_BAUD                                       (208)
#define UART_1_FBRD_32_MHZ_9600_BAUD                                        (21)




/* Defines for SPI_0 */
#define SPI_0_INST                                                         SPI1
#define SPI_0_INST_IRQHandler                                   SPI1_IRQHandler
#define SPI_0_INST_INT_IRQN                                       SPI1_INT_IRQn
#define GPIO_SPI_0_PICO_PORT                                              GPIOB
#define GPIO_SPI_0_PICO_PIN                                      DL_GPIO_PIN_15
#define GPIO_SPI_0_IOMUX_PICO                                   (IOMUX_PINCM32)
#define GPIO_SPI_0_IOMUX_PICO_FUNC                   IOMUX_PINCM32_PF_SPI1_PICO
#define GPIO_SPI_0_POCI_PORT                                              GPIOB
#define GPIO_SPI_0_POCI_PIN                                      DL_GPIO_PIN_14
#define GPIO_SPI_0_IOMUX_POCI                                   (IOMUX_PINCM31)
#define GPIO_SPI_0_IOMUX_POCI_FUNC                   IOMUX_PINCM31_PF_SPI1_POCI
/* GPIO configuration for SPI_0 */
#define GPIO_SPI_0_SCLK_PORT                                              GPIOB
#define GPIO_SPI_0_SCLK_PIN                                      DL_GPIO_PIN_16
#define GPIO_SPI_0_IOMUX_SCLK                                   (IOMUX_PINCM33)
#define GPIO_SPI_0_IOMUX_SCLK_FUNC                   IOMUX_PINCM33_PF_SPI1_SCLK
#define GPIO_SPI_0_CS1_PORT                                               GPIOB
#define GPIO_SPI_0_CS1_PIN                                       DL_GPIO_PIN_17
#define GPIO_SPI_0_IOMUX_CS1                                    (IOMUX_PINCM43)
#define GPIO_SPI_0_IOMUX_CS1_FUNC               IOMUX_PINCM43_PF_SPI1_CS1_POCI1



/* Port definition for Pin Group Beeper */
#define Beeper_PORT                                                      (GPIOB)

/* Defines for PIN_0: GPIOB.24 with pinCMx 52 on package pin 42 */
#define Beeper_PIN_0_PIN                                        (DL_GPIO_PIN_24)
#define Beeper_PIN_0_IOMUX                                       (IOMUX_PINCM52)
/* Port definition for Pin Group LED */
#define LED_PORT                                                         (GPIOA)

/* Defines for led1: GPIOA.12 with pinCMx 34 on package pin 27 */
#define LED_led1_PIN                                            (DL_GPIO_PIN_12)
#define LED_led1_IOMUX                                           (IOMUX_PINCM34)
/* Defines for led0: GPIOA.14 with pinCMx 36 on package pin 29 */
#define LED_led0_PIN                                            (DL_GPIO_PIN_14)
#define LED_led0_IOMUX                                           (IOMUX_PINCM36)
/* Defines for key0: GPIOA.18 with pinCMx 40 on package pin 33 */
#define KEY_key0_PORT                                                    (GPIOA)
#define KEY_key0_PIN                                            (DL_GPIO_PIN_18)
#define KEY_key0_IOMUX                                           (IOMUX_PINCM40)
/* Defines for key1: GPIOA.7 with pinCMx 14 on package pin 13 */
#define KEY_key1_PORT                                                    (GPIOA)
#define KEY_key1_PIN                                             (DL_GPIO_PIN_7)
#define KEY_key1_IOMUX                                           (IOMUX_PINCM14)
/* Defines for key2: GPIOB.2 with pinCMx 15 on package pin 14 */
#define KEY_key2_PORT                                                    (GPIOB)
#define KEY_key2_PIN                                             (DL_GPIO_PIN_2)
#define KEY_key2_IOMUX                                           (IOMUX_PINCM15)
/* Defines for key3: GPIOB.3 with pinCMx 16 on package pin 15 */
#define KEY_key3_PORT                                                    (GPIOB)
#define KEY_key3_PIN                                             (DL_GPIO_PIN_3)
#define KEY_key3_IOMUX                                           (IOMUX_PINCM16)
/* Defines for key4: GPIOA.8 with pinCMx 19 on package pin 16 */
#define KEY_key4_PORT                                                    (GPIOA)
#define KEY_key4_PIN                                             (DL_GPIO_PIN_8)
#define KEY_key4_IOMUX                                           (IOMUX_PINCM19)
/* Defines for leftA: GPIOA.2 with pinCMx 7 on package pin 8 */
#define Encoder_leftA_PORT                                               (GPIOA)
// pins affected by this interrupt request:["leftA","leftB","rightA"]
#define Encoder_GPIOA_INT_IRQN                                  (GPIOA_INT_IRQn)
#define Encoder_GPIOA_INT_IIDX                  (DL_INTERRUPT_GROUP1_IIDX_GPIOA)
#define Encoder_leftA_IIDX                                   (DL_GPIO_IIDX_DIO2)
#define Encoder_leftA_PIN                                        (DL_GPIO_PIN_2)
#define Encoder_leftA_IOMUX                                       (IOMUX_PINCM7)
/* Defines for leftB: GPIOA.1 with pinCMx 2 on package pin 2 */
#define Encoder_leftB_PORT                                               (GPIOA)
#define Encoder_leftB_IIDX                                   (DL_GPIO_IIDX_DIO1)
#define Encoder_leftB_PIN                                        (DL_GPIO_PIN_1)
#define Encoder_leftB_IOMUX                                       (IOMUX_PINCM2)
/* Defines for rightA: GPIOA.9 with pinCMx 20 on package pin 17 */
#define Encoder_rightA_PORT                                              (GPIOA)
#define Encoder_rightA_IIDX                                  (DL_GPIO_IIDX_DIO9)
#define Encoder_rightA_PIN                                       (DL_GPIO_PIN_9)
#define Encoder_rightA_IOMUX                                     (IOMUX_PINCM20)
/* Defines for rightB: GPIOB.6 with pinCMx 23 on package pin 20 */
#define Encoder_rightB_PORT                                              (GPIOB)
// pins affected by this interrupt request:["rightB"]
#define Encoder_GPIOB_INT_IRQN                                  (GPIOB_INT_IRQn)
#define Encoder_GPIOB_INT_IIDX                  (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define Encoder_rightB_IIDX                                  (DL_GPIO_IIDX_DIO6)
#define Encoder_rightB_PIN                                       (DL_GPIO_PIN_6)
#define Encoder_rightB_IOMUX                                     (IOMUX_PINCM23)
/* Port definition for Pin Group Patrol */
#define Patrol_PORT                                                      (GPIOA)

/* Defines for R2: GPIOA.25 with pinCMx 55 on package pin 45 */
#define Patrol_R2_PIN                                           (DL_GPIO_PIN_25)
#define Patrol_R2_IOMUX                                          (IOMUX_PINCM55)
/* Defines for R1: GPIOA.24 with pinCMx 54 on package pin 44 */
#define Patrol_R1_PIN                                           (DL_GPIO_PIN_24)
#define Patrol_R1_IOMUX                                          (IOMUX_PINCM54)
/* Defines for L1: GPIOA.27 with pinCMx 60 on package pin 47 */
#define Patrol_L1_PIN                                           (DL_GPIO_PIN_27)
#define Patrol_L1_IOMUX                                          (IOMUX_PINCM60)
/* Defines for L2: GPIOA.0 with pinCMx 1 on package pin 1 */
#define Patrol_L2_PIN                                            (DL_GPIO_PIN_0)
#define Patrol_L2_IOMUX                                           (IOMUX_PINCM1)
/* Defines for AIN1: GPIOB.18 with pinCMx 44 on package pin 37 */
#define Motor_AIN1_PORT                                                  (GPIOB)
#define Motor_AIN1_PIN                                          (DL_GPIO_PIN_18)
#define Motor_AIN1_IOMUX                                         (IOMUX_PINCM44)
/* Defines for AIN2: GPIOB.19 with pinCMx 45 on package pin 38 */
#define Motor_AIN2_PORT                                                  (GPIOB)
#define Motor_AIN2_PIN                                          (DL_GPIO_PIN_19)
#define Motor_AIN2_IOMUX                                         (IOMUX_PINCM45)
/* Defines for BIN1: GPIOB.8 with pinCMx 25 on package pin 22 */
#define Motor_BIN1_PORT                                                  (GPIOB)
#define Motor_BIN1_PIN                                           (DL_GPIO_PIN_8)
#define Motor_BIN1_IOMUX                                         (IOMUX_PINCM25)
/* Defines for BIN2: GPIOB.9 with pinCMx 26 on package pin 23 */
#define Motor_BIN2_PORT                                                  (GPIOB)
#define Motor_BIN2_PIN                                           (DL_GPIO_PIN_9)
#define Motor_BIN2_IOMUX                                         (IOMUX_PINCM26)
/* Defines for STBY: GPIOA.22 with pinCMx 47 on package pin 40 */
#define Motor_STBY_PORT                                                  (GPIOA)
#define Motor_STBY_PIN                                          (DL_GPIO_PIN_22)
#define Motor_STBY_IOMUX                                         (IOMUX_PINCM47)
#define GPIOA_EVENT_PUBLISHER_0_CHANNEL                                      (1)




/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_motor_init(void);
void SYSCFG_DL_PWM_servo_init(void);
void SYSCFG_DL_TIMER_0_init(void);
void SYSCFG_DL_I2C_MPU6050_init(void);
void SYSCFG_DL_I2C_0_init(void);
void SYSCFG_DL_UART_0_init(void);
void SYSCFG_DL_UART_1_init(void);
void SYSCFG_DL_SPI_0_init(void);

void SYSCFG_DL_SYSTICK_init(void);

bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
