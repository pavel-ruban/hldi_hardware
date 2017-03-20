//
// Created by root on 23/09/16.
//
#pragma once

#include <utils.h>
#include <stdint.h>
//#include <stm32f10x_conf.h>
#include "config.h"
#include <scheduler/include/scheduler.h>
#include <led/led.hpp>
#include <config.h>
#include <../esp8266/esp8266.h>
#include "binds.h"

#define MACHINE_STATE_IDLE 0
#define MACHINE_STATE_LOCK_OPEN 1
#define MACHINE_STATE_GUEST_CALL 2
#define MACHINE_STATE_AP_CONNECTING 3
#define MACHINE_STATE_NETWORK_PROBLEM 4
#define MACHINE_STATE_SERVER_CONNECTING 5
#define MACHINE_STATE_SERVER_PROBLEM 6
#define MACHINE_STATE_OTHER_NETWORK_PROBLEM 7
#define MACHINE_STATE_DUMMY 8
#define MACHINE_STATE_ACCESS_DENIED 9
#define CLOSE_TIME 3000
#define RING_TIME 5000
#define NETWORK_DELAY_DECLINE_TIME 5000

#define RECONNECT_TIME 50000

class Esp8266;

class Machine_state{
public:
    struct State {
        uint8_t state_changed;
        uint8_t connected_to_ap;
        uint8_t connected_to_server;
        uint8_t other_nodes_unreachable;
        uint8_t lock_is_open;
        uint8_t calling;
    } state;
    void open();
    void call();
    void invalidate();
    void bind_wifi(Esp8266 *wifi);
    void return_to_correct_state(uint32_t delay, uint8_t _state);
    void set_state(uint8_t state, uint32_t delay);
    void set_state_idle();
    void set_state_lock_open(uint8_t reason);
    void set_state_guest_call();
    void set_state_ap_connecting();
    void set_state_server_problem();
    void set_state_server_connecting();
    void set_state_network_problem();
    void set_state_other_network_problem();
    void set_state_dummy();
    void set_state_access_denied();
    uint8_t get_state();
    Esp8266 *_wifi;
    Machine_state();
    ~Machine_state();
    Machine_state(led **leds);
    uint8_t reasons[100] = {0};
    uint8_t reasons_counter = 0;
private:
    uint8_t lock_is_open = 0;
    uint32_t close_time = 0;
    uint8_t previous_state;
    led **leds;
    uint32_t delay;
    uint8_t current_state = 0;
};
