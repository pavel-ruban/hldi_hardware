#include "include/scheduler.h"
#pragma pack(1)

extern volatile uint32_t ticks;


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
    Array<Event,100>::iterator it1 = _events.begin();
    Array<Event,100>::iterator it2 = _events.end();
    uint32_t test1 = _events.begin()->invoke_time;
    uint32_t test2 = _events.end()->invoke_time;

    for(Array<Event,100>::iterator it = _events.begin(); it != _events.end(); ++it) {
        if (it->invoke_time <= ticks && it->deleted != 1) {
            _events.pop(it.current_index).handler();
        }
    }
}