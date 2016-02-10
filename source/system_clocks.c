/****************************************************************************
 * Copyright (c) 2016, ARM Limited, All Rights Reserved
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ***************************************************************************/

 // backwards compatibility
 #ifdef YOTTA_CFG_MBED_OS_TARGET_INHERITANCE

 #include "stm32f4xx.h"
 #include "hal_tick.h"

 // HSE-, HSI-, LSE- and LSI_VALUE are defined in here
 #include "stm32f4xx_hal_conf.h"

 #ifndef YOTTA_CFG_HARDWARE_CLOCK_FCPU
 #   error "You must define the CPU frequency you wish to have in yotta config: config.hardware.clock.fcpu"
 #endif

 #ifndef YOTTA_CFG_HARDWARE_CLOCK_FCPU_MAX
 #   error "You must define the maximum CPU frequency possible in yotta config: config.hardware.clock.fcpu_max"
 #endif

 #if YOTTA_CFG_HARDWARE_CLOCK_FCPU > YOTTA_CFG_HARDWARE_CLOCK_FCPU_MAX
 #   error "Requested CPU Frequency is above maximum allowed frequency!"
 #endif

 #if HSE_VALUE > 63000000
 #   error "External clock frequency is too high for PLL computations!"
 #endif

 #if HSE_VALUE % 1000000
 #   error "Only multiples of 1MHz are supported for external clock frequency!"
 #endif

 #define FCPU_VALUE YOTTA_CFG_HARDWARE_CLOCK_FCPU


/**
 * Helper function to calculate the PLL settings.
 *
 * This is a very rudimentary algorithm, with preselected clock tree configurations for:
 * - 84MHz for STM32F401
 * - 100MHz for STM32F410
 * - 168MHz for all others
 */
static uint8_t SetSysClock_PLL(RCC_OscInitTypeDef *const rcc)
{
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    /* If you want to do this in a more generic way, use a custom Jinja2 filter
     * for computing these settings in python.
     *
     * This will require turning this file into a `system_clocks.c.jinja2` and adding
     * a `jinja2_filters.py` file with the custom filter for pll calculation code.
     *
     * Oh, I bet this message will still be here in 20 years.
     */

    rcc->PLL.PLLState = RCC_PLL_ON;
    /* PLL Output must be between 192 and 432 MHz */
    /* HSI on F4 is always 16MHz */
    rcc->PLL.PLLM            = HSE_VALUE / 1000000; // VCO input clock = 1 MHz
#if FCPU_VALUE == 84000000      //  84MHz
    rcc->PLL.PLLN            = 336;            // VCO output clock = 336 MHz
    rcc->PLL.PLLP            = RCC_PLLP_DIV4;  // PLLCLK = 84 MHz (336 MHz / 4)
    rcc->PLL.PLLQ            = 7;              // USB clock = 48 MHz (336 MHz / 7)
#elif FCPU_VALUE == 100000000   // 100MHz
    rcc->PLL.PLLN            = 400;            // VCO output clock = 400 MHz
    rcc->PLL.PLLP            = RCC_PLLP_DIV4;  // PLLCLK = 100 MHz (400 MHz / 4)
    rcc->PLL.PLLQ            = 9;              // USB clock = 44.44 MHz (400 MHz / 9) --> Not good for USB
#elif FCPU_VALUE == 168000000   // 168MHz
    rcc->PLL.PLLN            = 336;            // VCO output clock = 336 MHz
    rcc->PLL.PLLP            = RCC_PLLP_DIV2;  // PLLCLK = 168 MHz (336 MHz / 2)
    rcc->PLL.PLLQ            = 7;              // USB clock = 48 MHz (336 MHz / 7)
#elif FCPU_VALUE == 180000000   // 180MHz
    rcc->PLL.PLLN            = 360;            // VCO output clock = 360 MHz
    rcc->PLL.PLLP            = RCC_PLLP_DIV2;  // PLLCLK = 180 MHz (360 MHz / 2)
    rcc->PLL.PLLQ            = 8;              // USB clock = 45 MHz (360 MHz / 8) --> Not good for USB
#else
#   error "Unsupported CPU Frequency for PLL calculations! Choose between 84MHz, 100MHz, 168MHz or 180MHz."
#endif

    if (HAL_RCC_OscConfig(rcc) != HAL_OK)
    {
        return 0; // FAIL
    }

    /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
    RCC_ClkInitStruct.ClockType      = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
#if FCPU_VALUE <= 100000000 // 100MHz
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
#else
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
#endif

    // wait stages for 2.7V - 3.6V
#if FCPU_VALUE == 84000000
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, 2) != HAL_OK)
#elif FCPU_VALUE == 100000000
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, 3) != HAL_OK)
#elif FCPU_VALUE == 168000000
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, 5) != HAL_OK)
#elif FCPU_VALUE == 180000000
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, 6) != HAL_OK)
#endif
    {
        return 0; // FAIL
    }

    /* Output clock on MCO1 pin(PA8) for debugging purpose */
    //HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1); // 16 MHz

    return 1; // OK
}

/******************************************************************************/
/*            PLL (clocked by HSI) used as System clock source                */
/******************************************************************************/
uint8_t SetSysClock_PLL_HSI(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct;

    /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet. */
    __PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /* Enable HSI oscillator and activate PLL with HSI as source */
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSEState            = RCC_HSE_OFF;
    RCC_OscInitStruct.HSICalibrationValue = 16;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;

    return SetSysClock_PLL(&RCC_OscInitStruct);
}

/******************************************************************************/
/*            PLL (clocked by HSE) used as System clock source                */
/******************************************************************************/
uint8_t SetSysClock_PLL_HSE(uint8_t bypass)
{
    RCC_OscInitTypeDef RCC_OscInitStruct;

    /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet. */
    __PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /* Enable HSE oscillator and activate PLL with HSE as source */
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState            = (bypass == 0) ? RCC_HSE_ON : RCC_HSE_BYPASS;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSE;

    return SetSysClock_PLL(&RCC_OscInitStruct);
}

#endif
