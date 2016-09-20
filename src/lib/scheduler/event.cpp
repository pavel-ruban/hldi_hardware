#include <scheduler/include/event.h>

Event::Event() {
    handler = NULL;
    invoke_time = 0;
}

Event::~Event(){

}

Event::Event(uint32_t time, void (led::*handler)()) {
    Event::handler = handler;
    invoke_time = time;
    deleted = 0;
}
