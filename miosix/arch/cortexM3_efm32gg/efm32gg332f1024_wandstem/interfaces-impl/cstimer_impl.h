/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   cstimer_impl.h
 * Author: fabiuz
 *
 * Created on October 3, 2016, 2:36 AM
 */
#include "high_resolution_timer_base.h"

#ifndef CSTIMER_IMPL_H
#define CSTIMER_IMPL_H
namespace miosix{
    
class ContextSwitchTimerImpl{
    public:
        HighResolutionTimerBase& b;
        ContextSwitchTimerImpl():
            b(HighResolutionTimerBase::instance()){};
};

}

#endif /* CSTIMER_IMPL_H */

