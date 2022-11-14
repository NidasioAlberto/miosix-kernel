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
 * @brief Direct Memory Access Controller Driver.
 *
 * The peripheral DMA requests can be enabled by programming the DMA control bit
 * in the registers of the corresponding peripheral. On each channel, multiple
 * peripheral request signals are ored together. This means that only one
 * request per channel must be enabled.
 */
class DMADriver {
public:
    enum class DataSize : uint32_t {
        BITS_8 = 0,
        BITS_16 = 1,
        BITS_32 = 2,
    };

    explicit DMADriver(DMA_TypeDef *dma, DMA_Channel_TypeDef *channel);

    /**
     * @brief Ensures that peripheral clock gets disabled.
     */
    ~DMADriver();

    /**
     * @brief Disables the peripheral clock.
     */
    void clockOn();

    /**
     * @brief Enables the peripheral clock.
     */
    void clockOff();

    void enable();

    void setPeripheralDataAddress(void *data);

    void setMemoryDataAddress(void *data);

    void setTransferSize(uint32_t size);

    void enableMemoryIncrement();

    void setMemoryDataSize(DataSize size);

    void setPeripheralDataSize(DataSize size);

    void setMemoryToPeripheralDirection();

    void enableCircularMode();

private:
    DMA_TypeDef *dma;
    DMA_Channel_TypeDef *channel;
};

}  // namespace miosix