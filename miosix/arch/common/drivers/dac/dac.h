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

namespace miosix {

/**
 * Driver for the DAC peripheral in STM32F4 microcontrollers
 */
class DACDriver {
public:
    enum class TriggerSource : uint32_t {
        TIM6_TRGO = 0,
        TIM3_TRGO = DAC_CR_TSEL1_0,
        TIM7_TRGO = DAC_CR_TSEL1_1,
        TIM15_TRGO = DAC_CR_TSEL1_1 | DAC_CR_TSEL1_0,
        TIM2_TRGO = DAC_CR_TSEL1_2,
        EXTI_line_9 = DAC_CR_TSEL1_2 | DAC_CR_TSEL1_1,
        SWTRIG = DAC_CR_TSEL1
    };

    /**
     * @brief Enables one channel of the DAC.
     *
     * @param channel Channel number, must be either 1 or 2.
     * @return True if the channel was enabled.
     */
    bool enableChannel(int channel);

    /**
     * @brief Disables one channel of the DAC.
     *
     * @param channel Channel number, must be either 1 or 2.
     * @return True if the channel was disabled.
     */
    bool disableChannel(int channel);

    /**
     * @brief Set the channel output voltage
     *
     * @param voltage On most boards the output is between 0V and 3V
     * @param channel Channel number, must be either 1 or 2.
     * @return True if the output voltage was changed.
     */
    bool setChannel(int channel, float voltage);

    bool enableTrigger(int channel,
                       TriggerSource source = TriggerSource::SWTRIG);

    bool disableTrigger(int channel);

    bool dispatchSoftwareTrigger(int channel);

    bool enableNoiseGeneration(int channel,
                               TriggerSource source = TriggerSource::SWTRIG);

    bool enableTriangularWaveGeneration(
        int channel, TriggerSource source = TriggerSource::SWTRIG);

    bool disableWaveGenerator(int channel);

    bool setWaveGeneratorMask(int channel, uint8_t bits);

    bool centerWaveOutput(int channel, float voltage);
};

}  // namespace miosix