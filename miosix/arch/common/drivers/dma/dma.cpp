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

#include "dma.h"

#include <util/clock_utils.h>

namespace miosix {

DMADriver& DMADriver::instance() {
    static DMADriver instance;
    return instance;
}

bool DMADriver::tryChannel(DMAChannelId id) {
    Lock<FastMutex> l(mutex);

    // Return true, meaning that the channel is free, only if it is not yet
    // allocated
    return channels.count(id) == 0;
}

DMAChannel* DMADriver::acquireChannel(DMAChannelId id) {
    Lock<FastMutex> l(mutex);

    // Wait until the channel is free
    while (channels.count(id) != 0)
        cv.wait(l);

    // Enable the clock if not already done
    if (channels.size() == 0)
        ClockUtils::enablePeripheralClock(DMA1);

    return channels[id] = new DMAChannel(id);
}

void DMADriver::releaseChannel(DMAChannelId id) {
    Lock<FastMutex> l(mutex);

    if (channels.count(id) != 0) {
        delete channels[id];
        channels.erase(id);
    }

    // Disable the clock if there are no more channels
    if (channels.size() == 0)
        ClockUtils::enablePeripheralClock(DMA1);
}

DMADriver::DMADriver() {}

void DMAChannel::setup(DMATransaction transaction) {
    // Reset the configuration
    registers->CCR = 0;

    registers->CCR |= static_cast<uint32_t>(transaction.direction);
    registers->CCR |= static_cast<uint32_t>(transaction.priority);
    if (transaction.circularMode)
        registers->CCR |= DMA_CCR_CIRC;
    registers->CNDTR = transaction.numberOfDataItems;

    if (transaction.direction == DMATransaction::Direction::MEM_TO_PER) {
        // In memory to peripheral mode, the source address is the memory
        // address

        registers->CCR |= static_cast<uint32_t>(transaction.sourceSize)
                          << DMA_CCR_MSIZE_Pos;
        registers->CCR |= static_cast<uint32_t>(transaction.destinationSize)
                          << DMA_CCR_PSIZE_Pos;

        if (transaction.sourceIncrement)
            registers->CCR |= DMA_CCR_MINC;
        if (transaction.destinationIncrement)
            registers->CCR |= DMA_CCR_PINC;

        registers->CMAR = reinterpret_cast<uint32_t>(transaction.sourceAddress);
        registers->CPAR =
            reinterpret_cast<uint32_t>(transaction.destinationAddress);

    } else {
        // In peripheral to memory or memory to memory mode, the source address
        // goes into the peripheral address register

        registers->CCR |= static_cast<uint32_t>(transaction.sourceSize)
                          << DMA_CCR_PSIZE_Pos;
        registers->CCR |= static_cast<uint32_t>(transaction.destinationSize)
                          << DMA_CCR_MSIZE_Pos;

        if (transaction.sourceIncrement)
            registers->CCR |= DMA_CCR_PINC;
        if (transaction.destinationIncrement)
            registers->CCR |= DMA_CCR_MINC;

        registers->CPAR = reinterpret_cast<uint32_t>(transaction.sourceAddress);
        registers->CMAR =
            reinterpret_cast<uint32_t>(transaction.destinationAddress);
    }
}

void DMAChannel::enable() { registers->CCR |= DMA_CCR_EN; }

void DMAChannel::disable() { registers->CCR &= ~DMA_CCR_EN; }

DMAChannel::DMAChannel(DMAChannelId id) {
    // Get the channel registers base address
    registers = reinterpret_cast<DMA_Channel_TypeDef*>(
        DMA1_BASE + 0x08 + 0x14 * static_cast<int>(id));
}

}  // namespace miosix