//
// Created by root on 16/09/16.
//

#ifndef SCHEDULER_H
#define SCHEDULER_H

#include "event.h"
#include "array.h"
#include <stdint-gcc.h>

#define SCHEDULER_FULL 1
#define SCHEDULER_IS_NOT_FULL 0
#define EVENT_TIMED_OUT 2


    extern volatile uint32_t ticks;


class Scheduler {
private:
    Array<Event, 100> _events;
   // uint32_t *_current_time;

public:
    uint16_t start_index;
    uint16_t end_index;
    uint8_t full;
    Scheduler();
    ~Scheduler();
//    Event & operator[](uint16_t element);
    uint8_t push(Event element);
    uint8_t handle();
};


Scheduler::Scheduler()
{
    full = SCHEDULER_IS_NOT_FULL;
    start_index = end_index = 0;
}

Scheduler::~Scheduler() {}

uint8_t Scheduler::push(Event event) {
    if (event.invoke_time < ticks){
        return EVENT_TIMED_OUT;
    }
    if (_events.capacity() < 100) {
        _events.push(event);
    } else return SCHEDULER_FULL;
}

uint8_t Scheduler::handle(){
    int arr[10];
    int i = 0;
//    Array<Event,100>::iterator it1 = _events.begin();
//    Array<Event,100>::iterator it2 = _events.end();
//    uint32_t test1 = _events.begin()->invoke_time;
//    uint32_t test2 = _events.end()->invoke_time;

    for(Array<Event,100>::iterator it = _events.begin(); it != _events.end(); ++it) {
        //arr[i++] = it->invoke_time;
        if (it->invoke_time <= ticks && it->deleted != 1) {


            it->handler();
            it->deleted = 1;
        }
    }
}
/*
Event & Scheduler<ssize>::operator[](uint16_t index) {
    for (int i  = 0; i < ssize; i++) {
        if (array[index]) {
            if (array[index]->invoke_time ){}
            if (array[index]->invoke_time <= ticks){

            }
            return *array[index];

        }
    }
}; */



#endif //SHEDULER_H
