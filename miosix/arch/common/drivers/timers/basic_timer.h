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
#include <util/clock_utils.h>

namespace miosix {

/**
 * @brief Driver for STM32 basic timers.
 *
 * This driver was tested on the STM32F0xx family.
 *
 * Basic timers main features are:
 * - 16bit auto-reload counter
 * - 16bit programmable prescaler used to divide the counter clock frequency
 * - Interrupt/DMA generation on the update event
 *
 * TIM6 and TIM7 are basic timers. You can also use any other timer as a basic
 * timer.
 *
 * The timer consist in a 16bit up-counter which is incremented every clock
 * cycle, the clock frequency can be divided by a 16bit prescaler and the
 * counter resets when it reaches an auto-reload value.
 *
 * Every time the counter reaches the auto-reload value, an UPDATE EVENT (UEV)
 * is fired and the counter is reset to 0. You can also generate the UEV by
 * software.
 *
 * The counter is working only when the prescaler is active, thus receiving the
 * clock signal.
 *
 * The auto-reload register can be buffered. If enabled, the value in in the
 * register are applied only at the update event, otherwise the new value takes
 * effect immediately.
 *
 * When the UEV occurs:
 * - The prescaler value is applied
 * - The auto-reload value is applied (if enabled)
 * - The update interrupt flag is set
 */
class BasicTimerDriver {
    /**
     * @brief Create a BasicTimer object. Note that this does not resets the
     * timer configuration but automatically enables the timer peripheral clock.
     */
    explicit BasicTimerDriver(TIM_TypeDef *timer);

    /**
     * @brief Disables the peripheral clock.
     */
    ~BasicTimerDriver();

    /**
     * @brief Resets the timer configuration to the default state.
     *
     * This means that:
     * - Auto reload register is not buffered, thus when you modify its value it
     * is taken into effect immediately
     * - One pulse mode disabled
     * - UEV and UG can trigger interrupt and DMA
     * - UEV generation (UG) is enabled
     * - The counter is disabled
     * - Master mode reset
     * - Interrupt and DMA request generation disabled
     * - Counter and prescaler set to 0
     * - Auto reload register set to 65535 (2^16-1)
     */
    virtual void reset();

    virtual void enable();

    virtual void disable();

protected:
    TIM_TypeDef *timer;
};

inline BasicTimerDriver::~BasicTimerDriver() {
    ClockUtils::disablePeripheralClock(timer);
}

inline void BasicTimerDriver::reset() {
    timer->CR1 = 0;
    timer->CR2 = 0;
    timer->DIER = 0;
    timer->CNT = 0;
    timer->PSC = 0;
    timer->ARR = 0xFFFF;
}

inline void BasicTimerDriver::enable() { timer->CR1 |= TIM_CR1_CEN; }

inline void BasicTimerDriver::disable() { timer->CR1 &= ~TIM_CR1_CEN; }

}  // namespace miosix