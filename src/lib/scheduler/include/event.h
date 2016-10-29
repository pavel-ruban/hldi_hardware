//
// Created by root on 16/09/16.
//

#pragma once
#pragma pack(1)

#include <stdint.h>
#include "config.h"

class led;

template <typename T>
class Event{
public:
    uint8_t deleted;
    uint32_t  invoke_time;
public:
    Event();
    Event(uint32_t time, T *obj, void (T::*handler)());
    Event(uint32_t time, void (*handler)());
    ~Event();
    T *obj;
    void run();
    bool invalidate(void *ptr);
    //void (*handler)();
    void (*handler)();
    void (T::*obj_handler)();
};

template <typename T>
Event<T>::Event() {
    handler = NULL;
    invoke_time = 0;
}

template <typename T>
Event<T>::~Event() {}

template <typename T>
Event<T>::Event(uint32_t time, T *_obj, void (T::*handler)()) {
    obj = _obj;
    Event::handler = NULL;
    Event::obj_handler = handler;
    invoke_time = time;
    deleted = 0;
}

template <typename T>
Event<T>::Event(uint32_t time, void (*handler)()) {
    obj = NULL;
    Event::obj_handler = NULL;
    Event::handler = handler;
    invoke_time = time;
    deleted = 0;
}

template <typename T>
void Event<T>::run() {
    if (obj) {
        (obj->*obj_handler)();
    } else {
        handler();
    }
}

template <typename T>
bool Event<T>::invalidate(void * ptr) {
    if (deleted != 1 && (T *) ptr == obj) {
        return true;
    }

    return false;
}