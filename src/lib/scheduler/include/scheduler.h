//
// Created by root on 16/09/16.
//

#pragma once
#pragma pack(1)

#include "event.h"
#include <array/array.h>
#include <stdint-gcc.h>

#define SCHEDULER_FULL 1
#define SCHEDULER_IS_NOT_FULL 0
#define EVENT_TIMED_OUT 2

extern "C" {
    extern volatile uint32_t ticks;
}

template <typename T, uint16_t scheduler_size>
class Scheduler {
private:
    Array<T, scheduler_size> _events;

public:
    uint16_t start_index;
    uint16_t end_index;
    uint8_t full;
    Scheduler();
    ~Scheduler();
    uint8_t push(T event);
    uint8_t handle();
    uint8_t invalidate(void *obj);
    uint32_t get_current_time();
    uint32_t size();
    void clear();
};

template <typename T, uint16_t scheduler_size>
Scheduler<T, scheduler_size>::Scheduler()
{
    full = SCHEDULER_IS_NOT_FULL;
    start_index = end_index = 0;
}

template <typename T, uint16_t scheduler_size>
Scheduler<T, scheduler_size>::~Scheduler() {}

template <typename T, uint16_t scheduler_size>
uint8_t Scheduler<T, scheduler_size>::push(T event) {
    if (event.invoke_time < ticks){
        return EVENT_TIMED_OUT;
    }
    if (!_events.full) {
        _events.push(event);
    } else return SCHEDULER_FULL;
}

template <typename T, uint16_t scheduler_size>
uint8_t Scheduler<T, scheduler_size>::handle() {
    int arr[10];
    int i = 0;
    typename Array<T, scheduler_size>::iterator it1 = _events.begin();
    typename Array<T, scheduler_size>::iterator it2 = _events.end();

    for(typename Array<T, scheduler_size>::iterator it = _events.begin(); it != _events.end(); ++it) {
        if (it->invoke_time <= ticks && it->deleted != 1) {
            T event = _events.pop(it.current_index);
            event.run();
        }
    }
}

template <typename T, uint16_t scheduler_size>
uint8_t Scheduler<T, scheduler_size>::invalidate(void *ptr) {
    typename Array<T, scheduler_size>::iterator it1 = _events.begin();
    typename Array<T, scheduler_size>::iterator it2 = _events.end();
    for(typename Array<T, scheduler_size>::iterator it = _events.begin(); it != _events.end(); ++it) {
        if (it->invalidate(ptr)) {
            _events.pop(it.current_index);
        }
    }
}

template <typename T, uint16_t scheduler_size>
uint32_t Scheduler<T, scheduler_size>::size() {
    return _events.size();
}

template <typename T, uint16_t scheduler_size>
uint32_t Scheduler<T, scheduler_size>::get_current_time() {
    return ticks;
}

template <typename T, uint16_t scheduler_size>
void Scheduler<T, scheduler_size>::clear() {
    _events.clear();
}
