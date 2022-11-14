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

#include "clock_utils.h"

namespace miosix {

/**
 * @brief Utilities for Timers.
 */
namespace TimerUtils {

enum class MasterMode : uint32_t {
    /**
     * @brief Only the updateGeneration() function is used as trigger
     * output.
     */
    RESET = 0,

    /**
     * @brief Only the timer enable is used as trigger output.
     *
     * This is useful to start several timers at the same time.
     */
    ENABLE = TIM_CR2_MMS_0,

    /**
     * @brief The UEV is selected as trigger output.
     *
     * This is useful when one timer is used as a prescaler for another
     * timer.
     */
    UPDATE = TIM_CR2_MMS_1,
};

enum class Channel : uint8_t {
    CHANNEL_1 = 0,
    CHANNEL_2 = 1,
    CHANNEL_3 = 2,
    CHANNEL_4 = 3
};

/**
 * @brief Returns the timer input clock.
 *
 * @return Timer input clock, APB1 or ABP2.
 */
ClockUtils::APB getTimerInputClock(const TIM_TypeDef *timer);

/**
 * @brief Returns the timer clock frequency before the prescaler.
 *
 * Class borrowed from the SynchronizedServo class.
 *
 * @param timer Timer to use.
 * @return Prescaler input frequency.
 */
uint32_t getPrescalerInputFrequency(const TIM_TypeDef *timer);

/**
 * @brief Return the timer clock frequency.
 *
 * @param timer Timer to use.
 * @return Timer frequency.
 */
uint32_t getClockFrequency(TIM_TypeDef *timer);

/**
 * @brief Returns the specified value converted in microseconds based on the
 * timer clock frequency and prescaler.
 *
 * @returns Timer counter in microseconds.
 */
float toMicroSeconds(TIM_TypeDef *timer, uint32_t value);

/**
 * @brief Returns the timer counter converted in microseconds based on the
 * timer clock frequency and prescaler.
 *
 * @returns Timer counter in microseconds.
 */
float toMicroSeconds(TIM_TypeDef *timer);

/**
 * @brief Returns the specified value converted in microseconds based on the
 * timer clock frequency and prescaler.
 *
 * Calculation using integer values.
 *
 * @returns Timer counter in microseconds.
 */
uint64_t toIntMicroSeconds(TIM_TypeDef *timer, uint32_t value);

/**
 * @brief Returns the timer counter converted in microseconds based on the
 * timer clock frequency and prescaler.
 *
 * Calculation using integer values.
 *
 * @returns Timer counter in microseconds.
 */
uint64_t toIntMicroSeconds(TIM_TypeDef *timer);

/**
 * @brief Returns the specified value converted in milliseconds based on the
 * timer clock frequency and prescaler.
 *
 * @returns Timer counter in milliseconds.
 */
float toMilliSeconds(TIM_TypeDef *timer, uint32_t value);

/**
 * @brief Returns the timer counter converted in milliseconds based on the
 * timer clock frequency and prescaler.
 *
 * @returns Timer counter in milliseconds.
 */
float toMilliSeconds(TIM_TypeDef *timer);

/**
 * @brief Returns the timer counter converted in seconds based on the timer
 * clock frequency and prescaler.
 *
 * @returns Timer counter in seconds.
 */
float toSeconds(TIM_TypeDef *timer);

/**
 * @brief Computes the timer resolution in microseconds.
 *
 * @return Microseconds per timer tick.
 */
float getResolution(TIM_TypeDef *timer);

/**
 * @brief Computes the number of seconds for timer reset.
 *
 * @return Timer duration before counter reset in seconds.
 */
float getMaxDuration(TIM_TypeDef *timer);

/**
 * @brief Compute the prescaler value for the specified target frequency.
 *
 * If the target frequency is above the prescaler input frequency, the returned
 * value will be 0 which is the maximum.
 *
 * @return Prescaler value for the target frequency.
 */
uint16_t computePrescalerValue(TIM_TypeDef *timer, int targetFrequency);

}  // namespace TimerUtils

inline ClockUtils::APB TimerUtils::getTimerInputClock(
    const TIM_TypeDef *timer) {
#ifndef _ARCH_CORTEXM0_STM32F0
    if (timer == TIM1 || timer == TIM8 || timer == TIM9 || timer == TIM10 ||
        timer == TIM11)
        return ClockUtils::APB::APB2;
    else
        return ClockUtils::APB::APB1;
#else
    return ClockUtils::APB::APB1;
#endif
}

inline uint32_t TimerUtils::getPrescalerInputFrequency(
    const TIM_TypeDef *timer) {
    return ClockUtils::getAPBFrequency(getTimerInputClock(timer));
}

inline uint32_t TimerUtils::getClockFrequency(TIM_TypeDef *timer) {
    return getPrescalerInputFrequency(timer) / (1 + timer->PSC);
}

inline float TimerUtils::toMicroSeconds(TIM_TypeDef *timer, uint32_t value) {
    return (1.0f * value * 1e6 * (1 + timer->PSC)) /
           getPrescalerInputFrequency(timer);
}

inline float TimerUtils::toMicroSeconds(TIM_TypeDef *timer) {
    return toMicroSeconds(timer, timer->CNT);
}

inline uint64_t TimerUtils::toIntMicroSeconds(TIM_TypeDef *timer,
                                              uint32_t value) {
    return ((uint64_t)value * 1e6 * (uint64_t)(1 + timer->PSC)) /
           getPrescalerInputFrequency(timer);
}

inline uint64_t TimerUtils::toIntMicroSeconds(TIM_TypeDef *timer) {
    return toIntMicroSeconds(timer, timer->CNT);
}

inline float TimerUtils::toMilliSeconds(TIM_TypeDef *timer, uint32_t value) {
    return (1.0f * value * 1e3 * (1 + timer->PSC)) /
           getPrescalerInputFrequency(timer);
}

inline float TimerUtils::toMilliSeconds(TIM_TypeDef *timer) {
    return toMilliSeconds(timer, timer->CNT);
}

inline float TimerUtils::toSeconds(TIM_TypeDef *timer) {
    return (1.0f * timer->CNT * (1 + timer->PSC)) /
           getPrescalerInputFrequency(timer);
}

inline float TimerUtils::getResolution(TIM_TypeDef *timer) {
    return (1.0e6f * (1 + timer->PSC)) / getPrescalerInputFrequency(timer);
}

inline float TimerUtils::getMaxDuration(TIM_TypeDef *timer) {
    return (1.0f * timer->ARR * 1e6 * (1 + timer->PSC)) /
           getPrescalerInputFrequency(timer);
}

inline uint16_t TimerUtils::computePrescalerValue(TIM_TypeDef *timer,
                                                  int targetFrequency) {
    int32_t targetPrescaler =
        TimerUtils::getPrescalerInputFrequency(timer) / targetFrequency - 1;
    return targetPrescaler >= 0 ? targetPrescaler : 0;
}

}  // namespace miosix
