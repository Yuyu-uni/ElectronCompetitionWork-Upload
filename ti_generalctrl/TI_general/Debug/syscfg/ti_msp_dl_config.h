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



/* Defines for Motor */
#define Motor_INST                                                         TIMA1
#define Motor_INST_IRQHandler                                   TIMA1_IRQHandler
#define Motor_INST_INT_IRQN                                     (TIMA1_INT_IRQn)
#define Motor_INST_CLK_FREQ                                             32000000
/* GPIO defines for channel 0 */
#define GPIO_Motor_C0_PORT                                                 GPIOA
#define GPIO_Motor_C0_PIN                                         DL_GPIO_PIN_17
#define GPIO_Motor_C0_IOMUX                                      (IOMUX_PINCM39)
#define GPIO_Motor_C0_IOMUX_FUNC                     IOMUX_PINCM39_PF_TIMA1_CCP0
#define GPIO_Motor_C0_IDX                                    DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_Motor_C1_PORT                                                 GPIOA
#define GPIO_Motor_C1_PIN                                         DL_GPIO_PIN_18
#define GPIO_Motor_C1_IOMUX                                      (IOMUX_PINCM40)
#define GPIO_Motor_C1_IOMUX_FUNC                     IOMUX_PINCM40_PF_TIMA1_CCP1
#define GPIO_Motor_C1_IDX                                    DL_TIMER_CC_1_INDEX



/* Defines for Pid */
#define Pid_INST                                                         (TIMG0)
#define Pid_INST_IRQHandler                                     TIMG0_IRQHandler
#define Pid_INST_INT_IRQN                                       (TIMG0_INT_IRQn)
#define Pid_INST_LOAD_VALUE                                             (39999U)
/* Defines for Cac */
#define Cac_INST                                                         (TIMG6)
#define Cac_INST_IRQHandler                                     TIMG6_IRQHandler
#define Cac_INST_INT_IRQN                                       (TIMG6_INT_IRQn)
#define Cac_INST_LOAD_VALUE                                               (388U)
/* Defines for Following */
#define Following_INST                                                   (TIMA0)
#define Following_INST_IRQHandler                               TIMA0_IRQHandler
#define Following_INST_INT_IRQN                                 (TIMA0_INT_IRQn)
#define Following_INST_LOAD_VALUE                                         (999U)



/* Defines for TJC */
#define TJC_INST                                                           UART0
#define TJC_INST_FREQUENCY                                              32000000
#define TJC_INST_IRQHandler                                     UART0_IRQHandler
#define TJC_INST_INT_IRQN                                         UART0_INT_IRQn
#define GPIO_TJC_RX_PORT                                                   GPIOB
#define GPIO_TJC_TX_PORT                                                   GPIOA
#define GPIO_TJC_RX_PIN                                            DL_GPIO_PIN_1
#define GPIO_TJC_TX_PIN                                           DL_GPIO_PIN_28
#define GPIO_TJC_IOMUX_RX                                        (IOMUX_PINCM13)
#define GPIO_TJC_IOMUX_TX                                         (IOMUX_PINCM3)
#define GPIO_TJC_IOMUX_RX_FUNC                         IOMUX_PINCM13_PF_UART0_RX
#define GPIO_TJC_IOMUX_TX_FUNC                          IOMUX_PINCM3_PF_UART0_TX
#define TJC_BAUD_RATE                                                   (115200)
#define TJC_IBRD_32_MHZ_115200_BAUD                                         (17)
#define TJC_FBRD_32_MHZ_115200_BAUD                                         (23)





/* Defines for ADC12_0 */
#define ADC12_0_INST                                                        ADC0
#define ADC12_0_INST_IRQHandler                                  ADC0_IRQHandler
#define ADC12_0_INST_INT_IRQN                                    (ADC0_INT_IRQn)
#define ADC12_0_ADCMEM_ADC_CH1                                DL_ADC12_MEM_IDX_0
#define ADC12_0_ADCMEM_ADC_CH1_REF               DL_ADC12_REFERENCE_VOLTAGE_VDDA
#define ADC12_0_ADCMEM_ADC_CH1_REF_VOLTAGE_V                                     3.3
#define GPIO_ADC12_0_C0_PORT                                               GPIOA
#define GPIO_ADC12_0_C0_PIN                                       DL_GPIO_PIN_27



/* Port definition for Pin Group LED1 */
#define LED1_PORT                                                        (GPIOB)

/* Defines for PIN_22: GPIOB.22 with pinCMx 50 on package pin 21 */
#define LED1_PIN_22_PIN                                         (DL_GPIO_PIN_22)
#define LED1_PIN_22_IOMUX                                        (IOMUX_PINCM50)
/* Port definition for Pin Group Ain */
#define Ain_PORT                                                         (GPIOB)

/* Defines for PIN_0: GPIOB.8 with pinCMx 25 on package pin 60 */
#define Ain_PIN_0_PIN                                            (DL_GPIO_PIN_8)
#define Ain_PIN_0_IOMUX                                          (IOMUX_PINCM25)
/* Defines for PIN_1: GPIOB.7 with pinCMx 24 on package pin 59 */
#define Ain_PIN_1_PIN                                            (DL_GPIO_PIN_7)
#define Ain_PIN_1_IOMUX                                          (IOMUX_PINCM24)
/* Port definition for Pin Group Bin */
#define Bin_PORT                                                         (GPIOB)

/* Defines for PIN_2: GPIOB.6 with pinCMx 23 on package pin 58 */
#define Bin_PIN_2_PIN                                            (DL_GPIO_PIN_6)
#define Bin_PIN_2_IOMUX                                          (IOMUX_PINCM23)
/* Defines for PIN_3: GPIOB.0 with pinCMx 12 on package pin 47 */
#define Bin_PIN_3_PIN                                            (DL_GPIO_PIN_0)
#define Bin_PIN_3_IOMUX                                          (IOMUX_PINCM12)
/* Defines for one: GPIOB.18 with pinCMx 44 on package pin 15 */
#define Follow_one_PORT                                                  (GPIOB)
#define Follow_one_PIN                                          (DL_GPIO_PIN_18)
#define Follow_one_IOMUX                                         (IOMUX_PINCM44)
/* Defines for two: GPIOB.17 with pinCMx 43 on package pin 14 */
#define Follow_two_PORT                                                  (GPIOB)
#define Follow_two_PIN                                          (DL_GPIO_PIN_17)
#define Follow_two_IOMUX                                         (IOMUX_PINCM43)
/* Defines for three: GPIOA.16 with pinCMx 38 on package pin 9 */
#define Follow_three_PORT                                                (GPIOA)
#define Follow_three_PIN                                        (DL_GPIO_PIN_16)
#define Follow_three_IOMUX                                       (IOMUX_PINCM38)
/* Defines for four: GPIOA.15 with pinCMx 37 on package pin 8 */
#define Follow_four_PORT                                                 (GPIOA)
#define Follow_four_PIN                                         (DL_GPIO_PIN_15)
#define Follow_four_IOMUX                                        (IOMUX_PINCM37)
/* Defines for five: GPIOA.14 with pinCMx 36 on package pin 7 */
#define Follow_five_PORT                                                 (GPIOA)
#define Follow_five_PIN                                         (DL_GPIO_PIN_14)
#define Follow_five_IOMUX                                        (IOMUX_PINCM36)
/* Defines for six: GPIOA.13 with pinCMx 35 on package pin 6 */
#define Follow_six_PORT                                                  (GPIOA)
#define Follow_six_PIN                                          (DL_GPIO_PIN_13)
#define Follow_six_IOMUX                                         (IOMUX_PINCM35)
/* Defines for seven: GPIOA.12 with pinCMx 34 on package pin 5 */
#define Follow_seven_PORT                                                (GPIOA)
#define Follow_seven_PIN                                        (DL_GPIO_PIN_12)
#define Follow_seven_IOMUX                                       (IOMUX_PINCM34)
/* Defines for eight: GPIOB.16 with pinCMx 33 on package pin 4 */
#define Follow_eight_PORT                                                (GPIOB)
#define Follow_eight_PIN                                        (DL_GPIO_PIN_16)
#define Follow_eight_IOMUX                                       (IOMUX_PINCM33)
/* Port definition for Pin Group Gray_Address */
#define Gray_Address_PORT                                                (GPIOB)

/* Defines for FPIN_0: GPIOB.15 with pinCMx 32 on package pin 3 */
#define Gray_Address_FPIN_0_PIN                                 (DL_GPIO_PIN_15)
#define Gray_Address_FPIN_0_IOMUX                                (IOMUX_PINCM32)
/* Defines for FPIN_1: GPIOB.14 with pinCMx 31 on package pin 2 */
#define Gray_Address_FPIN_1_PIN                                 (DL_GPIO_PIN_14)
#define Gray_Address_FPIN_1_IOMUX                                (IOMUX_PINCM31)
/* Defines for FPIN_2: GPIOB.13 with pinCMx 30 on package pin 1 */
#define Gray_Address_FPIN_2_PIN                                 (DL_GPIO_PIN_13)
#define Gray_Address_FPIN_2_IOMUX                                (IOMUX_PINCM30)

/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_Motor_init(void);
void SYSCFG_DL_Pid_init(void);
void SYSCFG_DL_Cac_init(void);
void SYSCFG_DL_Following_init(void);
void SYSCFG_DL_TJC_init(void);
void SYSCFG_DL_ADC12_0_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
