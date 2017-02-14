//
// Created by root on 23/09/16.
//
#pragma once


#include "machine_state.h"
extern Scheduler<Event<led>, 100> led_scheduler;
extern Scheduler<Event<Machine_state>, 100> state_scheduler;

Machine_state::Machine_state() {

}

void Machine_state::bind_wifi(Esp8266 *wifi) {
    _wifi = wifi;
}

Machine_state::Machine_state(led **leds) {
    Machine_state::leds = leds;
    state.calling = 0;
    state.connected_to_ap = 0; // 2 - Connecting.
    state.connected_to_server = 0; // 2 - Connecting.
    state.other_nodes_unreachable = 2; // 2 - Unknown, redefining true and false is bad idea.
    state.lock_is_open = 0;
}



Machine_state::~Machine_state() {

}


uint8_t Machine_state::get_state() {
    return current_state;
}

void Machine_state::return_to_correct_state(uint32_t delay, uint8_t _state){
    Event<Machine_state> _event(state_scheduler.get_current_time() + delay - 10, this, &Machine_state::set_state_dummy);
    state_scheduler.push(_event);
    switch (_state) {
        case MACHINE_STATE_IDLE: {
            Event<Machine_state> _event1(state_scheduler.get_current_time() + delay, this,
                                         &Machine_state::set_state_idle);
            state_scheduler.push(_event1);
            break;
        }
        //Call can be used only if state idle i.e. network is FULLY fine.
        case MACHINE_STATE_GUEST_CALL: {
            Event<Machine_state> _event1(state_scheduler.get_current_time() + delay, this,
                                         &Machine_state::set_state_idle);
            state_scheduler.push(_event1);
            break;
        }
        case MACHINE_STATE_OTHER_NETWORK_PROBLEM: {
            Event<Machine_state> _event2(state_scheduler.get_current_time() + delay, this,
                                         &Machine_state::set_state_other_network_problem);
            state_scheduler.push(_event2);
            break;
        }
        case MACHINE_STATE_NETWORK_PROBLEM: {
            Event<Machine_state> _event3(state_scheduler.get_current_time() + delay, this,
                                         &Machine_state::set_state_network_problem);
            state_scheduler.push(_event3);
            break;
        }
        case MACHINE_STATE_SERVER_PROBLEM: {
            Event<Machine_state> _event4(state_scheduler.get_current_time() + delay, this,
                                         &Machine_state::set_state_server_problem);
            state_scheduler.push(_event4);
            break;
        }
        case MACHINE_STATE_SERVER_CONNECTING: {
            Event<Machine_state> _event5(state_scheduler.get_current_time() + delay, this,
                                         &Machine_state::set_state_server_connecting);
            state_scheduler.push(_event5);
            break;
        }
        case MACHINE_STATE_AP_CONNECTING: {
            Event<Machine_state> _event6(state_scheduler.get_current_time() + delay, this,
                                         &Machine_state::set_state_ap_connecting);
            state_scheduler.push(_event6);
            break;
        }

    }
}

void Machine_state::set_state_dummy() {
    if (current_state == MACHINE_STATE_LOCK_OPEN || current_state == MACHINE_STATE_GUEST_CALL || current_state == MACHINE_STATE_ACCESS_DENIED) {
        previous_state = current_state;
        current_state = MACHINE_STATE_DUMMY;
    } else
        return;
}

void Machine_state::set_state_idle() {
    if (current_state == MACHINE_STATE_LOCK_OPEN || current_state == MACHINE_STATE_GUEST_CALL || current_state == MACHINE_STATE_ACCESS_DENIED) {
        return_to_correct_state(CLOSE_TIME + 10, MACHINE_STATE_IDLE);
        return;
    }
    previous_state = current_state;
    current_state = MACHINE_STATE_IDLE;
    led_scheduler.invalidate(leds[LED_STATE_INDICATOR]);
    leds[LED_STATE_INDICATOR]->set_color(LED_COLOR_WHITE);
    leds[LED_STATE_INDICATOR]->set_blink(0,1000,1000);
    leds[LED_STATE_INDICATOR]->on();
    GPIO_SetBits(EM_LOCK_PORT,EM_LOCK_PIN);
}

void Machine_state::set_state_other_network_problem() {
    if (current_state == MACHINE_STATE_LOCK_OPEN || current_state == MACHINE_STATE_GUEST_CALL || current_state == MACHINE_STATE_ACCESS_DENIED) {
        return_to_correct_state(CLOSE_TIME + 10, MACHINE_STATE_OTHER_NETWORK_PROBLEM);
        return;
    }
    previous_state = current_state;
    current_state = MACHINE_STATE_OTHER_NETWORK_PROBLEM;
    led_scheduler.invalidate(leds[LED_STATE_INDICATOR]);
    leds[LED_STATE_INDICATOR]->set_color(LED_COLOR_YELLOW);
    leds[LED_STATE_INDICATOR]->set_blink(0,1000,1000);
    leds[LED_STATE_INDICATOR]->on();
    GPIO_SetBits(EM_LOCK_PORT,EM_LOCK_PIN);
}

void Machine_state::set_state_lock_open(uint8_t reason) {
        //reasons[reasons_counter++] = reason;
    if (current_state == MACHINE_STATE_LOCK_OPEN)
        return;
    current_state = MACHINE_STATE_LOCK_OPEN;
    led_scheduler.invalidate(leds[LED_STATE_INDICATOR]);
    leds[LED_STATE_INDICATOR]->set_color(LED_COLOR_GREEN);
    leds[LED_STATE_INDICATOR]->set_blink(0,1000,1000);
    leds[LED_STATE_INDICATOR]->on();
    state_scheduler.invalidate(this);
    return_to_correct_state(CLOSE_TIME, previous_state);
    GPIO_ResetBits(EM_LOCK_PORT,EM_LOCK_PIN);
}

void Machine_state::set_state_access_denied() {
    if (current_state == MACHINE_STATE_ACCESS_DENIED)
        return;
    previous_state = current_state;
    current_state = MACHINE_STATE_ACCESS_DENIED;
    led_scheduler.invalidate(leds[LED_STATE_INDICATOR]);
    leds[LED_STATE_INDICATOR]->set_color(LED_COLOR_RED);
    leds[LED_STATE_INDICATOR]->set_blink(0,1000,1000);
    leds[LED_STATE_INDICATOR]->on();
    state_scheduler.invalidate(this);
    return_to_correct_state(CLOSE_TIME, previous_state);
    GPIO_ResetBits(EM_LOCK_PORT,EM_LOCK_PIN);
}

void Machine_state::set_state_guest_call() {
    if (current_state != MACHINE_STATE_IDLE)
        return;
    previous_state = current_state;
    current_state = MACHINE_STATE_GUEST_CALL;
    led_scheduler.invalidate(leds[LED_STATE_INDICATOR]);
    leds[LED_STATE_INDICATOR]->set_color(LED_COLOR_BLUE);
    leds[LED_STATE_INDICATOR]->set_blink(1,200,200);
    leds[LED_STATE_INDICATOR]->on();
    state_scheduler.invalidate(this);
    return_to_correct_state(RING_TIME, previous_state);
    GPIO_SetBits(EM_LOCK_PORT,EM_LOCK_PIN);
}

void Machine_state::set_state_network_problem() {
    if (current_state == MACHINE_STATE_LOCK_OPEN || current_state == MACHINE_STATE_GUEST_CALL || current_state == MACHINE_STATE_ACCESS_DENIED) {
        return_to_correct_state(CLOSE_TIME + 10, MACHINE_STATE_NETWORK_PROBLEM);
        return;
    }
    previous_state = current_state;
    current_state = MACHINE_STATE_NETWORK_PROBLEM;
    led_scheduler.invalidate(leds[LED_STATE_INDICATOR]);
    leds[LED_STATE_INDICATOR]->set_color(LED_COLOR_RED);
    leds[LED_STATE_INDICATOR]->set_blink(1,700,700);
    leds[LED_STATE_INDICATOR]->on();
    GPIO_SetBits(EM_LOCK_PORT,EM_LOCK_PIN);
}

void Machine_state::set_state_ap_connecting() {
    if (current_state == MACHINE_STATE_LOCK_OPEN || current_state == MACHINE_STATE_GUEST_CALL || current_state == MACHINE_STATE_ACCESS_DENIED) {
        return_to_correct_state(CLOSE_TIME + 10, MACHINE_STATE_AP_CONNECTING);
        return;
    }
    previous_state = current_state;
    current_state = MACHINE_STATE_AP_CONNECTING;
    led_scheduler.invalidate(leds[LED_STATE_INDICATOR]);
    leds[LED_STATE_INDICATOR]->set_color(LED_COLOR_PURPLE);
    leds[LED_STATE_INDICATOR]->set_blink(1,700,700);
    leds[LED_STATE_INDICATOR]->on();
    GPIO_SetBits(EM_LOCK_PORT,EM_LOCK_PIN);
}

void Machine_state::set_state_server_problem() {
    if (current_state == MACHINE_STATE_LOCK_OPEN || current_state == MACHINE_STATE_GUEST_CALL || current_state == MACHINE_STATE_ACCESS_DENIED) {
        return_to_correct_state(CLOSE_TIME + 10, MACHINE_STATE_SERVER_PROBLEM);
        return;
    }
    previous_state = current_state;
    current_state = MACHINE_STATE_SERVER_PROBLEM;
    led_scheduler.invalidate(leds[LED_STATE_INDICATOR]);
    leds[LED_STATE_INDICATOR]->set_color(LED_COLOR_RED);
    leds[LED_STATE_INDICATOR]->set_blink(1,200,200);
    leds[LED_STATE_INDICATOR]->on();
    GPIO_SetBits(EM_LOCK_PORT,EM_LOCK_PIN);
}

void Machine_state::set_state_server_connecting() {
    if (current_state == MACHINE_STATE_LOCK_OPEN || current_state == MACHINE_STATE_GUEST_CALL || current_state == MACHINE_STATE_ACCESS_DENIED) {
        return_to_correct_state(CLOSE_TIME + 10, MACHINE_STATE_SERVER_CONNECTING);
        return;
    }
    previous_state = current_state;
    current_state = MACHINE_STATE_SERVER_CONNECTING;
    led_scheduler.invalidate(leds[LED_STATE_INDICATOR]);
    leds[LED_STATE_INDICATOR]->set_color(LED_COLOR_PURPLE);
    leds[LED_STATE_INDICATOR]->set_blink(1,200,200);
    leds[LED_STATE_INDICATOR]->on();
    GPIO_SetBits(EM_LOCK_PORT,EM_LOCK_PIN);
}
