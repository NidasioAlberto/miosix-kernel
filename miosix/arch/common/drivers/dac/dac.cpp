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

#include "dac.h"

#include <interfaces/gpio.h>
#include <kernel/kernel.h>
#include <math.h>
#include <util/clock_utils.h>

#ifndef V_DDA_VOLTAGE
#error "V_DDA_VOLTAGE not defined. Pease specify the analog supply voltage."
#endif

#ifndef STM32F072xB
#warning "DAC Driver tested only on STM32F072xB"
#endif

namespace miosix {

#ifdef _ARCH_CORTEXM0_STM32F0
typedef Gpio<GPIOA_BASE, 4> ch1;
typedef Gpio<GPIOA_BASE, 5> ch2;
#endif

bool DACDriver::enableChannel(int channel) {
    if (channel < 1 || channel > 2)
        return false;

    if (channel == 1)
        ch1::mode(Mode::INPUT_ANALOG);
    else
        ch2::mode(Mode::INPUT_ANALOG);

    ClockUtils::enablePeripheralClock(DAC);

    // Enable the channel
    DAC->CR |= channel == 1 ? DAC_CR_EN1 : DAC_CR_EN2;

    return true;
}

bool DACDriver::disableChannel(int channel) {
    if (channel < 1 || channel > 2)
        return false;

    // Leave the gpio pins in analog configuration to consume less power.
    // Check AN4899 chapter 7

    // Disable the DAC clock if both channels are disabled
    if (DAC->CR & (channel == 1 ? DAC_CR_EN2 : DAC_CR_EN1))
        ClockUtils::disablePeripheralClock(DAC);

    return true;
}

bool DACDriver::setChannel(int channel, float voltage) {
    if (channel < 1 || channel > 2)
        return false;

    if (voltage > V_DDA_VOLTAGE)
        voltage = V_DDA_VOLTAGE;

    if (channel == 1)
        DAC->DHR12R1 = static_cast<uint16_t>(0xfff / V_DDA_VOLTAGE * voltage);
    else
        DAC->DHR12R2 = static_cast<uint16_t>(0xfff / V_DDA_VOLTAGE * voltage);

    return true;
}

bool DACDriver::enableBuffer(int channel) {
    if (channel < 1 || channel > 2)
        return false;

    if (channel == 1)
        DAC->CR |= DAC_CR_BOFF1;
    else
        DAC->CR |= DAC_CR_BOFF2;

    return true;
}

bool DACDriver::disableBuffer(int channel) {
    if (channel < 1 || channel > 2)
        return false;

    if (channel == 1)
        DAC->CR &= ~DAC_CR_BOFF1;
    else
        DAC->CR &= ~DAC_CR_BOFF2;

    return true;
}

bool DACDriver::enableTrigger(int channel, TriggerSource source) {
    if (channel < 1 || channel > 2)
        return false;

    // Prevent other threads or interrupts to access the DAC registers while
    // configuring
    FastInterruptDisableLock l;

    // We need to:
    // - First disable the trigger before changing the TSEL bits
    // - Change the trigger source (TSEL bits)
    // - Enable the trigger source

    if (channel == 1) {
        DAC->CR &= ~DAC_CR_TEN1;
        DAC->CR &= ~DAC_CR_TSEL1;
        DAC->CR |= static_cast<uint32_t>(source);
        DAC->CR |= DAC_CR_TEN1;
    } else {
        DAC->CR &= ~DAC_CR_TEN2;
        DAC->CR &= ~DAC_CR_TSEL2;
        DAC->CR |= static_cast<uint32_t>(source) << 16;
        DAC->CR |= DAC_CR_TEN2;
    }

    return true;
}

bool DACDriver::disableTrigger(int channel) {
    if (channel < 1 || channel > 2)
        return false;

    if (channel == 1)
        DAC->CR &= ~DAC_CR_TEN1;
    else
        DAC->CR &= ~DAC_CR_TEN2;

    return true;
}

bool DACDriver::dispatchSoftwareTrigger(int channel) {
    if (channel < 1 || channel > 2)
        return false;

    // The SWTRIGRx bit will be reset by hardware once the current DHRx value is
    // shifted into DORx
    if (channel == 1)
        DAC->SWTRIGR |= DAC_SWTRIGR_SWTRIG1;
    else
        DAC->SWTRIGR |= DAC_SWTRIGR_SWTRIG2;

    return true;
}

bool DACDriver::enableNoiseGeneration(int channel, TriggerSource source) {
    if (channel < 1 || channel > 2)
        return false;

    // The noise generator requires a trigger source, so we'll enable it
    enableTrigger(channel, source);

    // Enable the noise generator and set the mask to unmask all bits by default
    if (channel == 1) {
        DAC->CR &= ~DAC_CR_WAVE1;
        DAC->CR |= DAC_CR_WAVE1_0 | DAC_CR_MAMP1;
    } else {
        DAC->CR &= ~DAC_CR_WAVE2;
        DAC->CR |= DAC_CR_WAVE2_0 | DAC_CR_MAMP2;
    }

    return true;
}

bool DACDriver::enableTriangularWaveGeneration(int channel,
                                               TriggerSource source) {
    if (channel < 1 || channel > 2)
        return false;

    // The noise generator requires a trigger source, so we'll enable it
    enableTrigger(channel, source);

    // Enable the noise generator and set the mask to full amplitude by default
    if (channel == 1) {
        DAC->CR &= ~DAC_CR_WAVE1;
        DAC->CR |= DAC_CR_WAVE1_1 | DAC_CR_MAMP1;
    } else {
        DAC->CR &= ~DAC_CR_WAVE2;
        DAC->CR |= DAC_CR_WAVE2_1 | DAC_CR_MAMP2;
    }

    return true;
}

bool DACDriver::disableWaveGenerator(int channel) {
    if (channel < 1 || channel > 2)
        return false;

    // Disable the trigger that was enabled
    disableTrigger(channel);

    // Disable the wave generator
    if (channel == 1)
        DAC->CR &= ~DAC_CR_WAVE1;
    else
        DAC->CR &= ~DAC_CR_WAVE2;

    return true;
}

bool DACDriver::setWaveGeneratorMask(int channel, uint8_t bits) {
    if (channel < 1 || channel > 2 || bits < 1 || bits > 12)
        return false;

    if (channel == 1) {
        DAC->CR &= ~DAC_CR_MAMP1;                   // Reset the value
        DAC->CR |= (bits - 1) << DAC_CR_MAMP1_Pos;  // Set the new value
    } else {
        DAC->CR &= ~DAC_CR_MAMP2;                   // Reset the value
        DAC->CR |= (bits - 1) << DAC_CR_MAMP2_Pos;  // Set the new value
    }

    return true;
}

bool DACDriver::centerWaveOutput(int channel, float voltage) {
    if (channel < 1 || channel > 2)
        return false;

    // Fetch the unmasked bits
    uint8_t bits;
    if (channel == 1)
        bits = ((DAC->CR & DAC_CR_MAMP1) >> DAC_CR_MAMP1_Pos) + 1;
    else
        bits = ((DAC->CR & DAC_CR_MAMP2) >> DAC_CR_MAMP2_Pos) + 1;

    // Compute the wave amplitude
    float amplitude = V_DDA_VOLTAGE / 4095.0 * (powf(2, bits) - 1);

    // Set the appropriate voltage
    setChannel(channel, voltage - amplitude / 2);

    return true;
}

bool DACDriver::enableDMA(int channel) {
    if (channel < 1 || channel > 2)
        return false;

    if (channel == 1)
        DAC->CR |= DAC_CR_DMAEN1;
    else
        DAC->CR |= DAC_CR_DMAEN2;

    return true;
}

bool DACDriver::disableDMA(int channel) {
    if (channel < 1 || channel > 2)
        return false;

    if (channel == 1)
        DAC->CR &= ~DAC_CR_DMAEN1;
    else
        DAC->CR &= ~DAC_CR_DMAEN2;

    return true;
}

}  // namespace miosix