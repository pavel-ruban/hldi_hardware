//
// Created by root on 09/12/16.
//

#include "cache_handler.h"


void Cache_handler::forceInvalidateStuckEntries() {
    if (currentlyProcessing() && time_of_proc_start + SERVER_CONNECT_TIMEOUT < ticks) {
        current_processing = EVENT_CACHE_SIZE + 1;
    }
    for (uint16_t i = 0; i < EVENT_CACHE_SIZE; ++i) {
        if (event_cache[i].access_result == CURRENTLY_UNKNOWN) {
            if (event_cache[i].event_time + SERVER_CONNECT_TIMEOUT < ticks) {
                event_cache[i].needs_validation = 1;
                event_cache[i].access_result = DEFAULT_NOT_CACHED_BEHAVIOUR;
            }
        }
    }
}

//to do: когда очищать очередь?
//очередь в каком-то говне, индексы летят не пойми как.
void Cache_handler::addEvent(uint8_t tag_id[4], uint8_t node, uint8_t access_result, uint8_t cache_status, uint32_t time, uint8_t needs_validation) {
    uint32_t lest_time = event_cache[0].event_time;
    uint32_t lest_time_index = EVENT_CACHE_SIZE;
    uint16_t index = EVENT_CACHE_SIZE;
    for (uint16_t i = 0; i < EVENT_CACHE_SIZE; i++) {
        if (event_cache[i].event_time <= lest_time) {
            lest_time = event_cache[i].event_time;
            lest_time_index = i;
        }
        if (event_cache[i].needs_validation == 0 && event_cache[i].access_result != CURRENTLY_UNKNOWN)
        {
            index = i;
            break;
        }

        if (event_cache[i].event_time + _live_time < ticks)
        {
            index = i;
            break;
        }

    }
    if (index == EVENT_CACHE_SIZE) {
        index = lest_time_index;
    }
    event_cache[index].event_time = time;
    event_cache[index].access_result = access_result;
    event_cache[index].cache_status = cache_status;
    event_cache[index].node = node;
    event_cache[index].needs_validation = needs_validation;
    for (uint8_t i1 = 0; i1 < 4; ++i1) {
        event_cache[index].tag_id[i1] = tag_id[i1];
    }
//    if (!tag_events.full) {
//        tag_event event;
//        event.event_time = time;
//        event.node = node;
//        event.access_result = access_result;
//        event.cache_status = cache_status;
//        for (uint8_t i = 0; i < 4; ++i) {
//            event.tag_id[i] = tag_id[i];
//        }
//        tag_events.push_back(event);
//    }
}

void Cache_handler::updateEvent(uint8_t tag_id[4], uint8_t node, uint8_t access_result, uint32_t time, uint8_t needs_validation) {
    for(uint16_t i = 0; i < EVENT_CACHE_SIZE; i++) {
        if (event_cache[i].tag_id[1]  == tag_id[1] && event_cache[i].tag_id[2]  == tag_id[2] && event_cache[i].tag_id[3]  == tag_id[3]
            && event_cache[i].tag_id[4]  == tag_id[4] && event_cache[i].node == node && event_cache[i].event_time == time) {
            event_cache[i].access_result = access_result;
            event_cache[i].needs_validation = needs_validation;
            //event_cache[i].cache_status = NOT_CACHED;
            return;
        }
    }
}

void Cache_handler::deleteEvent(uint8_t tag_id[4], uint8_t node, uint32_t time) {
    for(uint16_t i = 0; i < EVENT_CACHE_SIZE; i++) {
        if (event_cache[i].tag_id[1]  == tag_id[1] && event_cache[i].tag_id[2]  == tag_id[2] && event_cache[i].tag_id[3]  == tag_id[3]
            && event_cache[i].tag_id[0]  == tag_id[0] && event_cache[i].node == node && event_cache[i].event_time == time) {
            event_cache[i].access_result = 0;
            event_cache[i].needs_validation = 0;
            for (uint8_t il = 0; il < 4; ++il) {
                event_cache[i].tag_id[il] = 0;
            }
            event_cache[i].cache_status = 0;
            event_cache[i].node = 0;
            event_cache[i].event_time = 0;
            if (current_processing == i) {
                current_processing = EVENT_CACHE_SIZE + 1;
            }
            return;
        }
    }
}

uint8_t Cache_handler::eventExist() {
    for(uint16_t i = 0; i < EVENT_CACHE_SIZE; i++) {
        if (event_cache[i].event_time + _live_time >= ticks && event_cache[i].needs_validation == 1)
        {
            return true;
        }
    }
    return false;
}

uint8_t Cache_handler::currentlyProcessing() {
    return (current_processing != EVENT_CACHE_SIZE + 1);
}

tag_event Cache_handler::popOldestEvent() {
    uint32_t most_time = 0;
    uint32_t most_time_index = EVENT_CACHE_SIZE;
    for(uint16_t i = 0; i < EVENT_CACHE_SIZE; i++) {
        if (event_cache[i].event_time + _live_time >= ticks && event_cache[i].needs_validation == 1 && event_cache[i].event_time >= most_time)
        {
            most_time_index = i;
        }
    }
    time_of_proc_start = ticks;
    current_processing = most_time_index;
    return event_cache[most_time_index];
}

void Cache_handler::addCard(uint8_t tag_id[4], uint8_t status) {
    uint16_t i;
    uint16_t first_empty = 0;
    for (i = 0; i < CARD_CACHE_SIZE; ++i) {
        if (card_cache[i].tag_id[0] == tag_id[0] && card_cache[i].tag_id[1] == tag_id[1] &&
            card_cache[i].tag_id[2] == tag_id[2] && card_cache[i].tag_id[3] == tag_id[3]) {
            if (card_cache[i].status == status) {
                break;
            } else {
                card_cache[i].status = status;
            }
        }
    }
    if (i == CARD_CACHE_SIZE) {
        for (first_empty = 0; first_empty < CARD_CACHE_SIZE; ++first_empty) {
            if (card_cache[first_empty].expr_time <= ticks) {
                break;
            }
        }
        card_cache[first_empty].expr_time = status != IN_PROGRESS ? ticks + _live_time : ticks + SERVER_CONNECT_TIMEOUT;
        card_cache[first_empty].status = status;
        for (uint8_t il = 0; il < 4; ++il) {
            card_cache[first_empty].tag_id[il] = tag_id[il];
        }
    }
}

uint8_t Cache_handler::checkCard(uint8_t tag_id[4]) {
    for (uint16_t i = 0; i < CARD_CACHE_SIZE; ++i) {
        if (card_cache[i].tag_id[0] == tag_id[0] && card_cache[i].tag_id[1] == tag_id[1] &&
            card_cache[i].tag_id[2] == tag_id[2] && card_cache[i].tag_id[3] == tag_id[3]) {
            if (card_cache[i].expr_time >= ticks) {
                return card_cache[i].status;
            } else
                return NOT_CACHED;

        }
    }
    return NOT_CACHED;
}

Cache_handler::Cache_handler(uint32_t live_time) {
    _live_time = live_time;
   // tag_events.override_data = 1;
}

Cache_handler::~Cache_handler() {

}