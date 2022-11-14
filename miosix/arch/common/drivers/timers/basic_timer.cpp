#include "basic_timer.h"

namespace miosix {

BasicTimerDriver::BasicTimerDriver(TIM_TypeDef *timer) {
    this->timer = timer;
    ClockUtils::enablePeripheralClock(timer);
}

BasicTimerDriver::~BasicTimerDriver() {
    ClockUtils::disablePeripheralClock(timer);
}

void BasicTimerDriver::reset() {
    timer->CR1 = 0;
    timer->CR2 = 0;
    timer->DIER = 0;
    timer->CNT = 0;
    timer->PSC = 0;
    timer->ARR = 0xFFFF;
}

void BasicTimerDriver::enable() { timer->CR1 |= TIM_CR1_CEN; }

void BasicTimerDriver::disable() { timer->CR1 &= ~TIM_CR1_CEN; }

uint16_t BasicTimerDriver::readPrescaler() { return timer->PSC; }

void BasicTimerDriver::setPrescaler(uint16_t prescalerValue) {
    timer->PSC = prescalerValue;
}

int BasicTimerDriver::getClockFrequency() {
    return TimerUtils::getClockFrequency(timer);
}

void BasicTimerDriver::setClockFrequency(int frequency) {
    setPrescaler(TimerUtils::computePrescalerValue(timer, frequency));
}

uint16_t BasicTimerDriver::readAutoReloadRegister() { return timer->ARR; }

void BasicTimerDriver::setAutoReloadRegister(uint16_t autoReloadValue) {
    timer->ARR = autoReloadValue;
}

void BasicTimerDriver::setMasterMode(TimerUtils::MasterMode mode) {
    timer->CR2 &= ~TIM_CR2_MMS;
    timer->CR2 |= static_cast<uint32_t>(mode);
}

}  // namespace miosix
