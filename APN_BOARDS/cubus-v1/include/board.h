/****************************************************************************
 * boards/arm/stm32/stm32f427a-minimal/include/board.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __BOARDS_ARM_STM32_STM32F427A_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32_STM32F427A_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/* DO NOT include STM32 internal header files here */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The STM32F427A minimal board uses a single 24MHz crystal.
 *  Space is provided for a 32kHz RTC backup crystal, but it is not stuffed.
 *
 * This is the canonical configuration:
 *   System Clock source           : PLL (HSE)
 *   SYSCLK(Hz)                    : 180000000    Determined by PLL
 *                                                configuration
 *   HCLK(Hz)                      : 180000000    (STM32_RCC_CFGR_HPRE)
 *   AHB Prescaler                 : 1            (STM32_RCC_CFGR_HPRE)
 *   APB1 Prescaler                : 4            (STM32_RCC_CFGR_PPRE1)
 *   APB2 Prescaler                : 2            (STM32_RCC_CFGR_PPRE2)
 *   HSE Frequency(Hz)             : 8000000      (STM32_BOARD_XTAL)
 *   PLLM                          : 24            (STM32_PLLCFG_PLLM)
 *   PLLN                          : 336          (STM32_PLLCFG_PLLN)
 *   PLLP                          : 2            (STM32_PLLCFG_PLLP)
 *   PLLQ                          : 7            (STM32_PLLCFG_PLLQ)
 *   Main regulator output voltage : Scale1 mode  Needed for high speed
 *                                                SYSCLK
 *   Flash Latency(WS)             : 5
 *   Prefetch Buffer               : OFF
 *   Instruction cache             : ON
 *   Data cache                    : ON
 *   Require 48MHz for USB OTG FS, : Enabled
 *   SDIO and RNG clock
 */

/* HSI - 16 MHz RC factory-trimmed
 * LSI - 32 KHz RC
 * HSE - On-board crystal frequency is 14MHz
 * LSE - 32.768 kHz
 */

#define STM32_BOARD_XTAL        24000000ul

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     32768

/* Main PLL Configuration.
 *
 * PLL source is HSE
 * PLL_VCO = (STM32_HSE_FREQUENCY / PLLM) * PLLN
 *         = (24,000,000 / 24) * 336
 *         = 336,000,000
 * SYSCLK  = PLL_VCO / PLLP
 *         = 336,000,000 / 4 = 84,000,000 
 * USB OTG FS, SDIO and RNG Clock
 *         =  PLL_VCO / PLLQ
 *         = 48,000,000
 */

#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(24)
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(336)
#define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP_4
#define STM32_PLLCFG_PLLQ       RCC_PLLCFG_PLLQ(7)

#define STM32_SYSCLK_FREQUENCY  84000000ul

/* AHB clock (HCLK) is SYSCLK (168MHz) */

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK  /* HCLK  = SYSCLK / 1 */
#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY

/* APB1 clock (PCLK1) is HCLK/4 (42MHz) */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd4     /* PCLK1 = HCLK / 4 */
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY/4)

/* Timers driven from APB1 will be twice PCLK1 */

#define STM32_APB1_TIM2_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM5_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM12_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM13_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM14_CLKIN  (2*STM32_PCLK1_FREQUENCY)

/* APB2 clock (PCLK2) is HCLK/2 (84MHz) */

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLKd2     /* PCLK2 = HCLK / 2 */
#define STM32_PCLK2_FREQUENCY   (STM32_HCLK_FREQUENCY/2)

/* Timers driven from APB2 will be twice PCLK2 */

#define STM32_APB2_TIM1_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM9_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM10_CLKIN  (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM11_CLKIN  (2*STM32_PCLK2_FREQUENCY)

/* Timer Frequencies, if APBx is set to 1, frequency is same to APBx
 * otherwise frequency is 2xAPBx.
 * Note: TIM1,8 are on APB2, others on APB1
 */

#define BOARD_TIM1_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM2_FREQUENCY    (STM32_HCLK_FREQUENCY/2)
#define BOARD_TIM3_FREQUENCY    (STM32_HCLK_FREQUENCY/2)
#define BOARD_TIM4_FREQUENCY    (STM32_HCLK_FREQUENCY/2)
#define BOARD_TIM5_FREQUENCY    (STM32_HCLK_FREQUENCY/2)
#define BOARD_TIM6_FREQUENCY    (STM32_HCLK_FREQUENCY/2)
#define BOARD_TIM7_FREQUENCY    (STM32_HCLK_FREQUENCY/2)
#define BOARD_TIM8_FREQUENCY    STM32_HCLK_FREQUENCY


/* Alternate function pin selections ****************************************/

/* USART1:
 *
 * The STM32F4 Discovery has no on-board serial devices, but the console is
 * brought out to PA9 (TX) and PA10 (RX) for connection to an external serial
 * device. (See the README.txt file for other options)
 */
#define GPIO_USART1_TX  GPIO_USART1_TX_2		// ttyS0  [COM - OBC UART  1]
#define GPIO_USART1_RX	GPIO_USART1_RX_2

#define GPIO_USART2_TX	GPIO_USART2_TX_2		// ttyS1  [COM - OBC UART 2]
#define GPIO_USART2_RX	GPIO_USART2_RX_2

#define GPIO_UART4_RX	GPIO_UART4_RX_2			// ttyS2  [OBC UART 1]
#define GPIO_UART4_TX	GPIO_UART4_TX_2

#define GPIO_USART6_RX	GPIO_USART6_RX_1		// ttyS3  [OBC UART 2]
#define GPIO_USART6_TX	GPIO_USART6_TX_1

#define GPIO_UART7_TX	GPIO_UART7_TX_1			// ttyS4  [Debug UART]
#define GPIO_UART7_RX	GPIO_UART7_RX_1

/* PWM
 *
 * The STM32F4 Discovery has no real on-board PWM devices, but the board can
 * be configured to output a pulse train using TIM4 CH2 on PD13.
 */

#define GPIO_TIM4_CH2OUT GPIO_TIM4_CH2OUT_2

#define GPIO_TIM1_CH1OUT  GPIO_TIM1_CH1OUT_2 /* PE9 */
#define GPIO_TIM1_CH1NOUT GPIO_TIM1_CH1N_3   /* PE8 */
#define GPIO_TIM1_CH2OUT  GPIO_TIM1_CH2OUT_2 /* PE11 */
#define GPIO_TIM1_CH2NOUT GPIO_TIM1_CH2N_3   /* PE10 */
#define GPIO_TIM1_CH3OUT  GPIO_TIM1_CH3OUT_2 /* PE13 */
#define GPIO_TIM1_CH3NOUT GPIO_TIM1_CH3N_3   /* PE12 */

/* I2C - There is a STMPE811 TouchPanel on I2C3 using these pins: */

#define GPIO_I2C3_SCL GPIO_I2C3_SCL_1
#define GPIO_I2C3_SDA GPIO_I2C3_SDA_1


/*
 * SPI
 *
 * There are sensors on SPI5, External ADC on SPI2
 * And Flash Memory on SPI 3 [Main FM] and SPI4 [Shared FM]
 */

#define GPIO_SPI2_MISO  (GPIO_SPI2_MISO_1|GPIO_SPEED_25MHz)
#define GPIO_SPI2_MOSI  (GPIO_SPI2_MOSI_1|GPIO_SPEED_25MHz)
#define GPIO_SPI2_SCK   (GPIO_SPI2_SCK_2|GPIO_SPEED_25MHz)

#define GPIO_SPI3_MISO	(GPIO_SPI3_MISO_1|GPIO_SPEED_25MHz)
#define GPIO_SPI3_MOSI	(GPIO_SPI3_MOSI_2|GPIO_SPEED_25MHz)
#define GPIO_SPI3_SCK	(GPIO_SPI3_SCK_1|GPIO_SPEED_25MHz)

#define GPIO_SPI4_MISO  (GPIO_SPI4_MISO_1|GPIO_SPEED_25MHz)
#define GPIO_SPI4_MOSI  (GPIO_SPI4_MOSI_1|GPIO_SPEED_25MHz)
#define GPIO_SPI4_SCK   (GPIO_SPI4_SCK_1|GPIO_SPEED_25MHz)

#define GPIO_SPI5_MISO  (GPIO_SPI5_MISO_2|GPIO_SPEED_25MHz)
#define GPIO_SPI5_MOSI  (GPIO_SPI5_MOSI_2|GPIO_SPEED_25MHz)
#define GPIO_SPI5_SCK   (GPIO_SPI5_SCK_2|GPIO_SPEED_25MHz)


/* Timer Inputs/Outputs (see the README.txt file for options) */

#define GPIO_TIM2_CH1IN  GPIO_TIM2_CH1IN_2
#define GPIO_TIM2_CH2IN  GPIO_TIM2_CH2IN_1

#define GPIO_TIM8_CH1IN  GPIO_TIM8_CH1IN_1
#define GPIO_TIM8_CH2IN  GPIO_TIM8_CH2IN_1


/* DMA **********************************************************************/

#define ADC1_DMA_CHAN DMAMAP_ADC1_1

#endif /* __BOARDS_ARM_STM32_STM32F427A_INCLUDE_BOARD_H */
