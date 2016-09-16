//
// Created by root on 16/09/16.
//

#ifndef SCHEDULER_H
#define SCHEDULER_H

#include "event.h"
#include <stdint-gcc.h>

#define SCHEDULER_FULL 1
#define SCHEDULER_IS_NOEvent_FULL 0

extern "C" {
    extern volatile uint32_t ticks;
}

template <uint16_t ssize>
class Scheduler {
private:
    Event *array[ssize];
    uint32_t invoke_time[ssize];

public:
    uint16_t start_index;
    uint16_t end_index;
    uint8_t full;
    Scheduler();
    ~Scheduler();
    Event & operator[](uint16_t element);
    Event & pop(uint16_t element);
    uint8_t push(Event element);
};


template <uint16_t ssize>
Scheduler<ssize>::Scheduler()
{
    full = SCHEDULER_IS_NOEvent_FULL;
    start_index = end_index = 0;
}

template <uint16_t ssize>
Scheduler<ssize>::~Scheduler() {}

template <uint16_t ssize>
uint8_t Scheduler<ssize>::push(Event index) {
    if (end_index < ssize) {
        *array[end_index++] = index;
    } else return SCHEDULER_FULL;
}

template <uint16_t ssize>
Event & Scheduler<ssize>::operator[](uint16_t index) {
    for (int i  = 0; i < ssize; i++) {
        if (array[index]) {
            if (array[index]->invoke_time ){}
//            if (array[index]->invoke_time <= ticks){
//
//            }
//            return *array[index];

        }
    }
};

template <uint16_t ssize>
Event & Scheduler<ssize>::pop(uint16_t index) {
    if (array[index]) {
        return *array[index];
    }
}


#endif //SHEDULER_H
