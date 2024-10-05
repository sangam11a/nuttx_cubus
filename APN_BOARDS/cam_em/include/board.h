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

#ifndef __BOARDS_BOARD_H
#define __BOARDS_BOARD_H

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

/* The STM32F427A minimal board uses a single 8MHz crystal. 
 *
 * This is the canonical configuration:
 *   System Clock source           : PLL (HSE)
 *   SYSCLK(Hz)                    : 180000000    Determined by PLL configuration
 *   HCLK(Hz)                      : 180000000    (STM32_RCC_CFGR_HPRE)
 *   AHB Prescaler                 : 1            (STM32_RCC_CFGR_HPRE)
 *   APB1 Prescaler                : 4            (STM32_RCC_CFGR_PPRE1)
 *   APB2 Prescaler                : 2            (STM32_RCC_CFGR_PPRE2)
 *   HSE Frequency(Hz)             : 8000000      (STM32_BOARD_XTAL)
 *   PLLM                          : 4            (STM32_PLLCFG_PLLM)
 *   PLLN                          : 72           (STM32_PLLCFG_PLLN)
 *   PLLP                          : 2            (STM32_PLLCFG_PLLP)
 *   PLLQ                          : 3            (STM32_PLLCFG_PLLQ)
 *   Main regulator output voltage : Scale1 mode  Needed for high speed SYSCLK
 *   Flash Latency(WS)             : 5
 *   Prefetch Buffer               : OFF
 *   Instruction cache             : ON
 *   Data cache                    : ON
 *   Require 48MHz for USB OTG FS, : Enabled
 *   SDIO and RNG clock
 */

/* Frequencies of various clock sources */

#define STM32_BOARD_XTAL        8000000ul
#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     32768

/* Main PLL Configuration.
 *
 * PLL source is HSE
 * PLL_VCO = (STM32_HSE_FREQUENCY / PLLM) * PLLN
 *         = (8,000,000 / 4) * 72
 *         = 144,000,000
 * SYSCLK  = PLL_VCO / PLLP
 *         = 144,000,000 / 2 = 72,000,000
 * USB OTG FS, SDIO and RNG Clock
 *         =  PLL_VCO / PLLQ
 *         = 48,000,000
 */

#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(4)
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(72)
#define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP_2
#define STM32_PLLCFG_PLLQ       RCC_PLLCFG_PLLQ(3)

#define STM32_SYSCLK_FREQUENCY  72000000ul

/* AHB clock (HCLK) is SYSCLK (72MHz) */

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK    /* HCLK  = SYSCLK */
#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY

/* APB1 clock (PCLK1) is HCLK/2 (36MHz) */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd2     /* PCLK1 = HCLK / 2 */
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY/2)

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

/* APB2 clock (PCLK2) is HCLK (72MHz) */

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLK    /* PCLK2 = HCLK */
#define STM32_PCLK2_FREQUENCY   STM32_HCLK_FREQUENCY

/* Timers driven from APB2 will be twice PCLK2 */

#define STM32_APB2_TIM1_CLKIN   (STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN   (STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM9_CLKIN   (STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM10_CLKIN  (STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM11_CLKIN  (STM32_PCLK2_FREQUENCY)

/* Alternate function pin selections ****************************************/

/* USART1:
 * Cubus-v1 has no on-board serial device. The console is brought up to UART7
 * (PE8) TX and (PE7) RX. 
 */

#define GPIO_USART1_TX  GPIO_USART1_TX_2
#define GPIO_USART1_RX  GPIO_USART1_RX_2

#define GPIO_USART2_TX  GPIO_USART2_TX_1
#define GPIO_USART2_RX  GPIO_USART2_RX_1

#define GPIO_UART4_RX   GPIO_UART4_RX_1
#define GPIO_UART4_TX   GPIO_UART4_TX_1

#define GPIO_USART6_RX  GPIO_USART6_RX_1 
#define GPIO_USART6_TX  GPIO_USART6_TX_1

#define GPIO_UART7_TX   GPIO_UART7_TX_1
#define GPIO_UART7_RX   GPIO_UART7_RX_1

#define GPIO_UART8_TX   GPIO_UART8_TX_0
#define GPIO_UART8_RX   GPIO_UART8_RX_0

/*
 * SPI
 *
 * There are sensors [MPU6500 & LIS3MDL] on SPI5, External ADC on SPI2
 * And Flash Memory on SPI 3 [Main FM] and SPI4 [Shared FM]
 */

#define GPIO_SPI2_MISO  (GPIO_SPI2_MISO_2|GPIO_SPEED_50MHz)
#define GPIO_SPI2_MOSI  (GPIO_SPI2_MOSI_2|GPIO_SPEED_50MHz)
#define GPIO_SPI2_SCK   (GPIO_SPI2_SCK_1|GPIO_SPEED_50MHz)

#define GPIO_SPI3_MISO  (GPIO_SPI3_MISO_2|GPIO_SPEED_50MHz)
#define GPIO_SPI3_MOSI  (GPIO_SPI3_MOSI_2|GPIO_SPEED_50MHz)
#define GPIO_SPI3_SCK   (GPIO_SPI3_SCK_2|GPIO_SPEED_50MHz)

#define GPIO_SPI4_MISO  (GPIO_SPI4_MISO_1|GPIO_SPEED_50MHz)
#define GPIO_SPI4_MOSI  (GPIO_SPI4_MOSI_1|GPIO_SPEED_50MHz)
#define GPIO_SPI4_SCK   (GPIO_SPI4_SCK_1|GPIO_SPEED_50MHz)

#define GPIO_SPI5_MISO  (GPIO_SPI5_MISO_2|GPIO_SPEED_50MHz)
#define GPIO_SPI5_MOSI  (GPIO_SPI5_MOSI_2|GPIO_SPEED_50MHz)
#define GPIO_SPI5_SCK   (GPIO_SPI5_SCK_2|GPIO_SPEED_50MHz)

/* Timer Inputs/Outputs (see the README.txt file for options) */

#define GPIO_TIM2_CH1IN  GPIO_TIM2_CH1IN_2
#define GPIO_TIM2_CH2IN  GPIO_TIM2_CH2IN_1

#define GPIO_TIM8_CH1IN  GPIO_TIM8_CH1IN_1
#define GPIO_TIM8_CH2IN  GPIO_TIM8_CH2IN_1

/* DMA **********************************************************************/

#define ADC1_DMA_CHAN DMAMAP_ADC1_1
#define ADC2_DMA_CHAN DMAMAP_ADC2_1
#define ADC3_DMA_CHAN DMAMAP_ADC3_1

#define DMAMAP_SPI3_RX  DMAMAP_SPI3_RX_1
#define DMAMAP_SPI3_TX  DMAMAP_SPI3_TX_1
#define DMAMAP_SPI4_RX  DMAMAP_SPI4_RX_1
#define DMAMAP_SPI4_TX  DMAMAP_SPI4_TX_1

/* DMA Channel/Stream Selections ********************************************/

#define DMAMAP_USART1_RX   DMAMAP_USART1_RX_2
#define DMAMAP_USART1_TX   DMAMAP_USART1_TX_2

#define DMAMAP_UART4_RX    DMAMAP_UART4_RX_1
#define DMAMAP_UART4_TX    DMAMAP_UART4_TX_1

#define DMAMAP_UART5_RX    DMAMAP_UART5_RX_1
#define DMAMAP_UART5_TX    DMAMAP_UART5_TX_1

#define DMAMAP_USART6_RX   DMAMAP_USART6_RX_1
#define DMAMAP_USART6_TX   DMAMAP_USART6_TX_1

#define DMAMAP_UART7_RX    DMAMAP_UART7_RX_1
#define DMAMAP_UART7_TX    DMAMAP_UART7_TX_1

#define DMAMAP_UART8_RX    DMAMAP_UART8_RX_1
#define DMAMAP_UART8_TX    DMAMAP_UART8_TX_1

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void stm32_boardinitialize(void);

#endif /* __BOARDS_BOARD_H */
