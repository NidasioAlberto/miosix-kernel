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

DMADriver::DMADriver(DMA_TypeDef *dma, DMA_Channel_TypeDef *channel)
    : dma(dma), channel(channel) {}

DMADriver::~DMADriver() { ClockUtils::disablePeripheralClock(dma); }

void DMADriver::clockOn() { ClockUtils::enablePeripheralClock(dma); }

void DMADriver::clockOff() { ClockUtils::disablePeripheralClock(dma); }

void DMADriver::enable() { DMA1_Channel3->CCR |= DMA_CCR_EN; }

void DMADriver::setPeripheralDataAddress(void *data) {
    DMA1_Channel3->CPAR |= reinterpret_cast<uint32_t>(data);
}

void DMADriver::setMemoryDataAddress(void *data) {
    DMA1_Channel3->CMAR |= reinterpret_cast<uint32_t>(data);
}

void DMADriver::setTransferSize(uint32_t size) { DMA1_Channel3->CNDTR = size; }

void DMADriver::enableMemoryIncrement() { DMA1_Channel3->CCR |= DMA_CCR_MINC; }

void DMADriver::setMemoryDataSize(DataSize size) {
    DMA1_Channel3->CCR |= static_cast<uint32_t>(size) << DMA_CCR_MSIZE_Pos;
}

void DMADriver::setPeripheralDataSize(DataSize size) {
    DMA1_Channel3->CCR |= static_cast<uint32_t>(size) << DMA_CCR_PSIZE_Pos;
}

void DMADriver::setMemoryToPeripheralDirection() {
    DMA1_Channel3->CCR |= DMA_CCR_DIR;
}

void DMADriver::enableCircularMode() { DMA1_Channel3->CCR |= DMA_CCR_CIRC; }

}  // namespace miosix