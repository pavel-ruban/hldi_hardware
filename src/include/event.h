//
// Created by root on 16/09/16.
//

#ifndef EVENT_H
#define EVENT_H

#include "config.h"

extern "C" {
    extern volatile uint32_t ticks;
}

class Event{
public:
    uint8_t deleted;
    uint32_t  invoke_time;
public:
    Event();
    Event(uint32_t time, void (*fptr)());
    ~Event();
    void run();
    void (*handler)();
};

Event::Event() {
    handler = NULL;
    invoke_time = 0;
}

Event::~Event(){

}

Event::Event(uint32_t time, void (*handler)()) {
    Event::handler = handler;
    invoke_time = time;
    deleted = 0;
}

void Event::run() {

    if (ticks >= invoke_time) {
        handler();
    }
}

/*struct bar
{
    void say(int a, int b)
    { cout << a << ' ' << b << endl; }
};

template <typename T>
struct foo
{
    template <typename fptr>
    void say(fptr f, int a, int b)
    {
        (i.*f)(a, b);
    }

    T i;
};

int main() {
    foo<bar> f;
    f.say(&bar::say, 10, 100);
    //bar::say();
} */

#endif //EVENT_H
