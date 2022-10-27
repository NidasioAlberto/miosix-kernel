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

#include <miosix.h>

#ifndef V_DDA_VOLTAGE
#error "V_DDA_VOLTAGE not defined. Pease specify the analog supply voltage."
#endif

namespace miosix {

#ifdef _ARCH_CORTEXM0_STM32
typedef Gpio<GPIOA_BASE, 4> ch1;
typedef Gpio<GPIOA_BASE, 5> ch2;
#endif

bool DACDriver::enableChannel(int n) {
    if (n < 1 || n > 2)
        return false;

    {
        // Modify the gpio and clock register in a critical section just in case
        // another thread is doing the same
        FastInterruptDisableLock l;

        if (n == 1)
            ch1::mode(Mode::INPUT_ANALOG);
        else
            ch2::mode(Mode::INPUT_ANALOG);

        // Enable the peripheral clock if not already done
        if (!(RCC->APB1ENR & RCC_APB1ENR_DACEN)) {
            RCC->APB1ENR |= RCC_APB1ENR_DACEN;
            RCC_SYNC();
        }
    }

    // Enable the channel
    DAC->CR |= n == 1 ? DAC_CR_EN1 : DAC_CR_EN2;

    return true;
}

bool DACDriver::setChannel(float voltage, int n) {
    if (n < 1 || n > 2)
        return false;

    if (voltage > V_DDA_VOLTAGE)
        voltage = V_DDA_VOLTAGE;

    if (n == 1)
        DAC->DHR12R1 = static_cast<uint16_t>(0xfff / V_DDA_VOLTAGE * voltage);
    else
        DAC->DHR12R2 = static_cast<uint16_t>(0xfff / V_DDA_VOLTAGE * voltage);

    return true;
}

}  // namespace miosix