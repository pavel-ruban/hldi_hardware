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

class Event;

class Scheduler {
private:
    Array<Event, 100> _events;

public:
    uint16_t start_index;
    uint16_t end_index;
    uint8_t full;
    Scheduler();
    ~Scheduler();
    uint8_t push(Event event);
    uint8_t handle();
};


