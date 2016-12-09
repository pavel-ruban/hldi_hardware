//
// Created by root on 09/12/16.
//

#include "cache_handler.h"


//to do: когда очищать очередь?
void Cache_handler::addEvent(uint8_t tag_id[4], uint8_t node) {

    if (!tag_events.full) {
        tag_event event;
        event.event_time = ticks;
        event.node = node;
        for (uint8_t i = 0; i < 4; ++i) {
            event.tag_id[i] = tag_id[i];
        }
        tag_events.push_back(event);
    }

}

uint8_t Cache_handler::eventExist() {
    return !tag_events.empty();
}

tag_event Cache_handler::popEvent() {
    tag_event buf = tag_events.front();
    tag_events.pop_front();
    return buf;
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
            if (card_cache[first_empty].expr_time <= 0) {
                break;
            }
        }
        card_cache[first_empty].expr_time = ticks + _live_time;
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
            return card_cache[i].status;
        }
    }
}

Cache_handler::Cache_handler(uint32_t live_time) {
    _live_time = live_time;
}

Cache_handler::~Cache_handler() {

}