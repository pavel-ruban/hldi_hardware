#pragma once
#pragma pack(1)


#include <stdint.h>
#include "queue.h"

//10 hours.
#define EXPIRATION_TIME 36000000
#define EVENT_CACHE_SIZE 100
#define CARD_CACHE_SIZE 100

#define ACCESS_DENIED 0
#define ACCESS_GRANTED 1
#define NOT_CACHED 2

typedef struct {
    uint8_t tag_id[4];
    uint32_t event_time;
    uint8_t node;
} tag_event;


extern "C" {
    extern volatile uint32_t ticks;
}

class Cache_handler {
public:

    uint8_t eventExist();
    tag_event popEvent();
    Cache_handler(uint32_t live_time);
    ~Cache_handler();
    void addEvent(uint8_t tag_id[4], uint8_t node);
    uint8_t checkCard(uint8_t tag_id[4]);
    void addCard(uint8_t tag_id[4], uint8_t status);
private:
    uint32_t _live_time;
    typedef struct {
        uint8_t tag_id[4];
        uint32_t expr_time;
        uint8_t status;
    } card_status;



    card_status card_cache[CARD_CACHE_SIZE];
    //Queue<tag_cache_entry, 100> tag_cache;

    Queue<tag_event, EVENT_CACHE_SIZE> tag_events;
};
