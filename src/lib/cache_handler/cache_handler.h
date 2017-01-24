#pragma once
#pragma pack(1)


#include <stdint.h>
#include "queue.h"
#include "config.h"

//10 hours.
#define EXPIRATION_TIME 36000000
#define EVENT_CACHE_SIZE 100
#define CARD_CACHE_SIZE 100

#define ACCESS_DENIED 0
#define ACCESS_GRANTED 1
#define CURRENTLY_UNKNOWN 4


#define NOT_CACHED 2
#define CACHED 3
#define IN_PROGRESS 5

typedef struct {
    uint8_t tag_id[4];
    uint32_t event_time;
    uint8_t node;
    uint8_t access_result;
    uint8_t cache_status;
    uint8_t needs_validation;
} tag_event;


extern "C" {
    extern volatile uint32_t ticks;
}

class Cache_handler {
public:

    uint8_t eventExist();
    tag_event popOldestEvent();
    Cache_handler(uint32_t live_time);
    ~Cache_handler();
    void addEvent(uint8_t tag_id[4], uint8_t node, uint8_t access_result, uint8_t cache_status, uint32_t time, uint8_t needs_validation);
    void updateEvent(uint8_t tag_id[4], uint8_t node, uint8_t access_result, uint32_t time, uint8_t needs_validation);
    uint8_t checkCard(uint8_t tag_id[4]);
    void addCard(uint8_t tag_id[4], uint8_t status);
    void deleteEvent(uint8_t tag_id[4], uint8_t node,uint32_t time);
    void forceInvalidateStuckEntries();
    uint8_t currentlyProcessing();
private:
    uint32_t time_of_proc_start = 0;
    uint16_t current_processing = EVENT_CACHE_SIZE + 1;
    uint32_t _live_time;
    typedef struct {
        uint8_t tag_id[4];
        uint32_t expr_time;
        uint8_t status;
    } card_status;



    card_status card_cache[CARD_CACHE_SIZE];
    tag_event event_cache[EVENT_CACHE_SIZE] = {0};
    //Queue<tag_cache_entry, 100> tag_cache;

    //Queue<tag_event, EVENT_CACHE_SIZE> tag_events;
};
