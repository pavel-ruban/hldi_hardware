//
// Created by root on 16/09/16.
//
#pragma once
#pragma pack(1)

#include <stdint.h>
#include "config.h"

class led;

class Event{
public:
    uint8_t deleted;
    uint32_t  invoke_time;
public:
    Event();
    Event(uint32_t time, void (led::*)());
    ~Event();
    //void (*handler)();
    void (led::*handler)();
};
