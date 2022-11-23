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
#include <kernel/sync.h>

#include <map>

namespace miosix {

enum class DMAChannelId {
    DMA1_CH1 = 0,
    DMA1_CH2,
    DMA1_CH3,
    DMA1_CH4,
    DMA1_CH5,
    DMA1_CH6,
    DMA1_CH7,
};

struct DMATransaction {
    enum class Direction : uint16_t {
        MEM_TO_MEM = DMA_CCR_MEM2MEM,
        MEM_TO_PER = DMA_CCR_DIR,
        PER_TO_MEM = 0,
    };
    enum class Priority : uint16_t {
        VERY_HIGH = DMA_CCR_PL,
        HIGH = DMA_CCR_PL_1,
        MEDIOUM = DMA_CCR_PL_0,
        LOW = 0,
    };
    enum class DataSize : uint8_t { BITS_8, BITS_16, BIT_32 };

    Direction direction = Direction::MEM_TO_MEM;
    Priority priority = Priority::LOW;
    DataSize sourceSize = DataSize::BIT_32;
    DataSize destinationSize = DataSize::BIT_32;
    void* sourceAddress = nullptr;
    void* destinationAddress = nullptr;
    uint16_t numberOfDataItems = 0;
    bool sourceIncrement = false;
    bool destinationIncrement = false;
    bool circularMode = false;
};

// Forward declaration
class DMAChannel;

class DMADriver {
public:
    static DMADriver& instance();

    bool tryChannel(DMAChannelId id);

    DMAChannel* acquireChannel(DMAChannelId id);

    void releaseChannel(DMAChannelId id);

private:
    DMADriver();

    FastMutex mutex;
    ConditionVariable cv;
    std::map<DMAChannelId, DMAChannel*> channels;

public:
    DMADriver(const DMADriver&) = delete;
    DMADriver& operator=(const DMADriver&) = delete;
};

class DMAChannel {
    friend DMADriver;

public:
    void setup(DMATransaction transaction);

    void enable();

    void disable();

private:
    DMAChannel(DMAChannelId id);

    DMAChannelId id;
    DMA_Channel_TypeDef* registers;

public:
    DMAChannel(const DMAChannel&) = delete;
    DMAChannel& operator=(const DMAChannel&) = delete;
};

}  // namespace miosix