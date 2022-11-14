/***************************************************************************
 *   Copyright (C) 2022 by Alberto Nidasio                                 *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   As a special exception, if other files instantiate templates or use   *
 *   macros or inline functions from this file, or you compile this file   *
 *   and link it with other works to produce a work based on this file,    *
 *   this file does not by itself cause the resulting work to be covered   *
 *   by the GNU General Public License. However the source code for this   *
 *   file must still be made available in accordance with the GNU General  *
 *   Public License. This exception does not invalidate any other reasons  *
 *   why a work based on this file might be covered by the GNU General     *
 *   Public License.                                                       *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, see <http://www.gnu.org/licenses/>   *
 ***************************************************************************/

#pragma once

#include <interfaces/arch_registers.h>
#include <kernel/kernel.h>

namespace miosix {

/**
 * @brief Utilities for the Cortex clocks.
 */
namespace ClockUtils {

/**
 * @brief Advanced Peripheral Bus enumeration.
 */
enum class APB {
    APB1,
#ifndef _ARCH_CORTEXM0_STM32F0
    APB2
#endif
};

/**
 * @brief Computes the output frequency for the given APB bus.
 *
 * @param bus Advanced Peripheral Bus
 * @return Prescaler input frequency.
 */
uint32_t getAPBFrequency(APB bus = APB::APB1);

/**
 * @brief Enables a peripheral clock source from the APB1 and APB2 peripheral
 * buses.
 */
bool enablePeripheralClock(void* peripheral);

/**
 * @brief Disables a peripheral clock source from the APB1 and APB2 peripheral
 * buses.
 */
bool disablePeripheralClock(void* peripheral);

}  // namespace ClockUtils

inline uint32_t ClockUtils::getAPBFrequency(APB bus) {
    // The global variable SystemCoreClock from ARM's CMSIS allows to know the
    // HCLK frequency
    uint32_t inputFrequency = SystemCoreClock;

    // The timer frequency may be a submultiple of the CPU frequency, due to the
    // bus at which the peripheral is connected being slower.
    // The RCC-CFGR register tells us how slower the APB bus is running.
    // The following formula takes into account that if the APB1 clock is
    // divided by a factor of two or grater, the timer is clocked at twice the
    // bus interface.
    if (bus == APB::APB1) {
        // The position of the PPRE1 bit in RCC->CFGR is different between stm32
#if defined(_ARCH_CORTEXM0_STM32F0) || defined(_ARCH_CORTEXM3_STM32F1)
        constexpr uint32_t ppre1 = 8;
#elif defined(_ARCH_CORTEXM4_STM32F4) || defined(_ARCH_CORTEXM3_STM32F2)
        constexpr uint32_t ppre1 = 10;
#else
#error "Architecture not supported by TimerUtils"
#endif

#ifdef _ARCH_CORTEXM0_STM32F0
        if (RCC->CFGR & RCC_CFGR_PPRE)
#else
        if (RCC->CFGR & RCC_CFGR_PPRE1_2)
#endif
            inputFrequency /= 1 << ((RCC->CFGR >> ppre1) & 0x3);

#ifndef _ARCH_CORTEXM0_STM32F0
    } else {
        // The position of the PPRE2 bit in RCC->CFGR is different in some stm32
#ifdef _ARCH_CORTEXM3_STM32
        constexpr uint32_t ppre2 = 11;
#elif defined(_ARCH_CORTEXM4_STM32F4) || defined(_ARCH_CORTEXM3_STM32F2)
        constexpr uint32_t ppre2 = 13;
#else
#error "Architecture not supported by TimerUtils"
#endif

        if (RCC->CFGR & RCC_CFGR_PPRE2_2) {
            inputFrequency /= 1 << ((RCC->CFGR >> ppre2) >> 0x3);
        }
#endif
    }

    return inputFrequency;
}

#ifndef _ARCH_CORTEXM0_STM32F0
inline bool ClockUtils::enablePeripheralClock(void* peripheral) {
    miosix::FastInterruptDisableLock dLock;

    switch (reinterpret_cast<uint32_t>(peripheral)) {
        // AHB1 peripherals
        {
            case GPIOA_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
                break;
            case GPIOB_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
                break;
            case GPIOC_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
                break;
            case GPIOD_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
                break;
            case GPIOE_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
                break;
            case GPIOF_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
                break;
            case GPIOG_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;
                break;
            case GPIOH_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;
                break;
            case GPIOI_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_GPIOIEN;
                break;
#ifdef STM32F429xx
            case GPIOJ_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_GPIOJEN;
                break;
            case GPIOK_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_GPIOKEN;
                break;
#endif
            case CRC_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;
                break;
            case BKPSRAM_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_BKPSRAMEN;
                break;
#ifndef _ARCH_CORTEXM3_STM32F2
            case CCMDATARAM_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_CCMDATARAMEN;
                break;
#endif
            case DMA1_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
                break;
            case DMA2_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
                break;
#ifdef STM32F429xx
            case DMA2D_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_DMA2DEN;
                break;
#endif
#ifndef _ARCH_CORTEXM3_STM32F2
            case ETH_MAC_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_ETHMACEN;
                break;
#endif
            case USB_OTG_HS_PERIPH_BASE:
                RCC->AHB1ENR |= RCC_AHB1ENR_OTGHSEN;
                break;
        }

        // AHB2 peripherals
        {
#ifndef _ARCH_CORTEXM3_STM32F2
            case DCMI_BASE:
                RCC->AHB2ENR |= RCC_AHB2ENR_DCMIEN;
                break;
#endif
            case RNG_BASE:
                RCC->AHB2ENR |= RCC_AHB2ENR_RNGEN;
                break;
            case USB_OTG_FS_PERIPH_BASE:
                RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;
                break;
        }

        // APB1 peripherals
        {
            case TIM2_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
                break;
            case TIM3_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
                break;
            case TIM4_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
                break;
            case TIM5_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
                break;
            case TIM6_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
                break;
            case TIM7_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
                break;
            case TIM12_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_TIM12EN;
                break;
            case TIM13_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_TIM13EN;
                break;
            case TIM14_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
                break;
            case WWDG_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_WWDGEN;
                break;
            case SPI2_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
                break;
            case SPI3_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
                break;
            case USART2_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
                break;
            case USART3_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
                break;
            case UART4_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_UART4EN;
                break;
            case UART5_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_UART5EN;
                break;
            case I2C1_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
                break;
            case I2C2_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
                break;
            case I2C3_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_I2C3EN;
                break;
            case CAN1_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
                break;
            case CAN2_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_CAN2EN;
                break;
            case PWR_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_PWREN;
                break;
            case DAC_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_DACEN;
                break;
#ifdef STM32F429xx
            case UART7_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_UART7EN;
                break;
            case UART8_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_UART8EN;
                break;
#endif
        }

        // APB2 peripherals
        {
            case TIM1_BASE:
                RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
                break;
            case TIM8_BASE:
                RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;
                break;
            case USART1_BASE:
                RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
                break;
            case USART6_BASE:
                RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
                break;
            case ADC1_BASE:
                RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
                break;
            case ADC2_BASE:
                RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;
                break;
            case ADC3_BASE:
                RCC->APB2ENR |= RCC_APB2ENR_ADC3EN;
                break;
            case SDIO_BASE:
                RCC->APB2ENR |= RCC_APB2ENR_SDIOEN;
                break;
            case SPI1_BASE:
                RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
                break;
#ifdef STM32F429xx
            case SPI4_BASE:
                RCC->APB2ENR |= RCC_APB2ENR_SPI4EN;
                break;
#endif
            case SYSCFG_BASE:
                RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
                break;
            case TIM9_BASE:
                RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;
                break;
            case TIM10_BASE:
                RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;
                break;
            case TIM11_BASE:
                RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;
                break;
#ifdef STM32F429xx
            case SPI5_BASE:
                RCC->APB2ENR |= RCC_APB2ENR_SPI5EN;
                break;
            case SPI6_BASE:
                RCC->APB2ENR |= RCC_APB2ENR_SPI6EN;
                break;
            case SAI1_BASE:
                RCC->APB2ENR |= RCC_APB2ENR_SAI1EN;
                break;
            case LTDC_BASE:
                RCC->APB2ENR |= RCC_APB2ENR_LTDCEN;
                break;
#endif
        }

        default:
            return false;
    }

    RCC_SYNC();

    return true;
}
#else
inline bool ClockUtils::enablePeripheralClock(void* peripheral) {
    miosix::FastInterruptDisableLock dLock;

    switch (reinterpret_cast<uint32_t>(peripheral)) {
        // AHB peripherals
        {
            case TSC_BASE:
                RCC->AHBENR |= RCC_AHBENR_TSCEN;
                break;
            case GPIOA_BASE:
                RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
                break;
            case GPIOB_BASE:
                RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
                break;
            case GPIOC_BASE:
                RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
                break;
            case GPIOD_BASE:
                RCC->AHBENR |= RCC_AHBENR_GPIODEN;
                break;
            case GPIOE_BASE:
                RCC->AHBENR |= RCC_AHBENR_GPIOEEN;
                break;
            case GPIOF_BASE:
                RCC->AHBENR |= RCC_AHBENR_GPIOFEN;
                break;
            case CRC_BASE:
                RCC->AHBENR |= RCC_AHBENR_CRCEN;
                break;
            case DMA1_BASE:
                RCC->AHBENR |= RCC_AHBENR_DMA1EN;
                break;
                // DMA2 only on STM32F09x
        }

        // APB1 peripherals
        {
            case TIM2_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
                break;
            case TIM3_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
                break;
            case TIM6_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
                break;
            case TIM7_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
                break;
            case TIM14_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
                break;
            case WWDG_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_WWDGEN;
                break;
            case SPI2_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
                break;
            case USART2_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
                break;
            case USART3_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
                break;
            case USART4_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_USART4EN;
                break;
            case I2C1_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
                break;
            case I2C2_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
                break;
            case USB_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_USBEN;
                break;
            case CAN_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_CANEN;
                break;
            case CRS_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_CRSEN;
                break;
            case PWR_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_PWREN;
                break;
            case DAC_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_DACEN;
                break;
            case CEC_BASE:
                RCC->APB1ENR |= RCC_APB1ENR_DACEN;
                break;
        }

        default:
            return false;
    }

    RCC_SYNC();

    return true;
}
#endif

#ifndef _ARCH_CORTEXM0_STM32F0
inline bool ClockUtils::disablePeripheralClock(void* peripheral) {
    miosix::FastInterruptDisableLock dLock;

    switch (reinterpret_cast<uint32_t>(peripheral)) {
        // AHB1 peripherals
        {
            case GPIOA_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOAEN;
                break;
            case GPIOB_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOBEN;
                break;
            case GPIOC_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOCEN;
                break;
            case GPIOD_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIODEN;
                break;
            case GPIOE_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOEEN;
                break;
            case GPIOF_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOFEN;
                break;
            case GPIOG_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOGEN;
                break;
            case GPIOH_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOHEN;
                break;
            case GPIOI_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOIEN;
                break;
#ifdef STM32F429xx
            case GPIOJ_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOJEN;
                break;
            case GPIOK_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOKEN;
                break;
#endif
            case CRC_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_CRCEN;
                break;
            case BKPSRAM_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_BKPSRAMEN;
                break;
#ifndef _ARCH_CORTEXM3_STM32F2
            case CCMDATARAM_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_CCMDATARAMEN;
                break;
#endif
            case DMA1_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_DMA1EN;
                break;
            case DMA2_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_DMA2EN;
                break;
#ifdef STM32F429xx
            case DMA2D_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_DMA2DEN;
                break;
#endif
#ifndef _ARCH_CORTEXM3_STM32F2
            case ETH_MAC_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_ETHMACEN;
                break;
#endif
            case USB_OTG_HS_PERIPH_BASE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_OTGHSEN;
                break;
        }

        // AHB2 peripherals
        {
#ifndef _ARCH_CORTEXM3_STM32F2
            case DCMI_BASE:
                RCC->AHB2ENR &= ~RCC_AHB2ENR_DCMIEN;
                break;
#endif
            case RNG_BASE:
                RCC->AHB2ENR &= ~RCC_AHB2ENR_RNGEN;
                break;
            case USB_OTG_FS_PERIPH_BASE:
                RCC->AHB2ENR &= ~RCC_AHB2ENR_OTGFSEN;
                break;
        }

        // APB1 peripherals
        {
            case TIM2_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_TIM2EN;
                break;
            case TIM3_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_TIM2EN;
                break;
            case TIM4_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_TIM4EN;
                break;
            case TIM5_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_TIM5EN;
                break;
            case TIM6_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_TIM6EN;
                break;
            case TIM7_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_TIM7EN;
                break;
            case TIM12_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_TIM12EN;
                break;
            case TIM13_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_TIM13EN;
                break;
            case TIM14_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_TIM14EN;
                break;
            case WWDG_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_WWDGEN;
                break;
            case SPI2_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_SPI2EN;
                break;
            case SPI3_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_SPI3EN;
                break;
            case USART2_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_USART2EN;
                break;
            case USART3_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_USART3EN;
                break;
            case UART4_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_UART4EN;
                break;
            case UART5_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_UART5EN;
                break;
            case I2C1_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_I2C1EN;
                break;
            case I2C2_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_I2C2EN;
                break;
            case I2C3_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_I2C3EN;
                break;
            case CAN1_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_CAN1EN;
                break;
            case CAN2_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_CAN2EN;
                break;
            case PWR_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_PWREN;
                break;
            case DAC_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_DACEN;
                break;
#ifdef STM32F429xx
            case UART7_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_UART7EN;
                break;
            case UART8_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_UART8EN;
                break;
#endif
        }

        // APB2 peripherals
        {
            case TIM1_BASE:
                RCC->APB2ENR &= ~RCC_APB2ENR_TIM1EN;
                break;
            case TIM8_BASE:
                RCC->APB2ENR &= ~RCC_APB2ENR_TIM8EN;
                break;
            case USART1_BASE:
                RCC->APB2ENR &= ~RCC_APB2ENR_USART1EN;
                break;
            case USART6_BASE:
                RCC->APB2ENR &= ~RCC_APB2ENR_USART6EN;
                break;
            case ADC1_BASE:
                RCC->APB2ENR &= ~RCC_APB2ENR_ADC1EN;
                break;
            case ADC2_BASE:
                RCC->APB2ENR &= ~RCC_APB2ENR_ADC2EN;
                break;
            case ADC3_BASE:
                RCC->APB2ENR &= ~RCC_APB2ENR_ADC3EN;
                break;
            case SDIO_BASE:
                RCC->APB2ENR &= ~RCC_APB2ENR_SDIOEN;
                break;
            case SPI1_BASE:
                RCC->APB2ENR &= ~RCC_APB2ENR_SPI1EN;
                break;
#ifdef STM32F429xx
            case SPI4_BASE:
                RCC->APB2ENR &= ~RCC_APB2ENR_SPI4EN;
                break;
#endif
            case SYSCFG_BASE:
                RCC->APB2ENR &= ~RCC_APB2ENR_SYSCFGEN;
                break;
            case TIM9_BASE:
                RCC->APB2ENR &= ~RCC_APB2ENR_TIM9EN;
                break;
            case TIM10_BASE:
                RCC->APB2ENR &= ~RCC_APB2ENR_TIM10EN;
                break;
            case TIM11_BASE:
                RCC->APB2ENR &= ~RCC_APB2ENR_TIM11EN;
                break;
#ifdef STM32F429xx
            case SPI5_BASE:
                RCC->APB2ENR &= ~RCC_APB2ENR_SPI5EN;
                break;
            case SPI6_BASE:
                RCC->APB2ENR &= ~RCC_APB2ENR_SPI6EN;
                break;
            case SAI1_BASE:
                RCC->APB2ENR &= ~RCC_APB2ENR_SAI1EN;
                break;
            case LTDC_BASE:
                RCC->APB2ENR &= ~RCC_APB2ENR_LTDCEN;
                break;
#endif
        }

        default:
            return false;
    }

    RCC_SYNC();

    return true;
}
#else
inline bool ClockUtils::disablePeripheralClock(void* peripheral) {
    miosix::FastInterruptDisableLock dLock;

    switch (reinterpret_cast<uint32_t>(peripheral)) {
        // AHB peripherals
        {
            case TSC_BASE:
                RCC->AHBENR &= ~RCC_AHBENR_TSCEN;
                break;
            case GPIOA_BASE:
                RCC->AHBENR &= ~RCC_AHBENR_GPIOAEN;
                break;
            case GPIOB_BASE:
                RCC->AHBENR &= ~RCC_AHBENR_GPIOBEN;
                break;
            case GPIOC_BASE:
                RCC->AHBENR &= ~RCC_AHBENR_GPIOCEN;
                break;
            case GPIOD_BASE:
                RCC->AHBENR &= ~RCC_AHBENR_GPIODEN;
                break;
            case GPIOE_BASE:
                RCC->AHBENR &= ~RCC_AHBENR_GPIOEEN;
                break;
            case GPIOF_BASE:
                RCC->AHBENR &= ~RCC_AHBENR_GPIOFEN;
                break;
            case CRC_BASE:
                RCC->AHBENR &= ~RCC_AHBENR_CRCEN;
                break;
            case DMA1_BASE:
                RCC->AHBENR &= ~RCC_AHBENR_DMA1EN;
                break;
                // DMA2 only on STM32F09x
        }

        // APB1 peripherals
        {
            case TIM2_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_TIM2EN;
                break;
            case TIM3_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_TIM3EN;
                break;
            case TIM6_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_TIM6EN;
                break;
            case TIM7_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_TIM7EN;
                break;
            case TIM14_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_TIM14EN;
                break;
            case WWDG_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_WWDGEN;
                break;
            case SPI2_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_SPI2EN;
                break;
            case USART2_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_USART2EN;
                break;
            case USART3_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_USART3EN;
                break;
            case USART4_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_USART4EN;
                break;
            case I2C1_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_I2C1EN;
                break;
            case I2C2_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_I2C2EN;
                break;
            case USB_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_USBEN;
                break;
            case CAN_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_CANEN;
                break;
            case CRS_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_CRSEN;
                break;
            case PWR_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_PWREN;
                break;
            case DAC_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_DACEN;
                break;
            case CEC_BASE:
                RCC->APB1ENR &= ~RCC_APB1ENR_DACEN;
                break;
        }

        default:
            return false;
    }

    RCC_SYNC();

    return true;
}
#endif

}  // namespace miosix
