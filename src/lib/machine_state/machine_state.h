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
#include "binds.h"

#define MACHINE_STATE_IDLE 0
#define MACHINE_STATE_LOCK_OPEN 1
#define MACHINE_STATE_GUEST_CALL 2
#define MACHINE_STATE_AP_CONNECTING 3
#define MACHINE_STATE_NETWORK_PROBLEM 4
#define MACHINE_STATE_SERVER_CONNECTING 5
#define MACHINE_STATE_SERVER_PROBLEM 6

class Machine_state{
public:
    void set_state(uint8_t state, uint32_t delay);
    void set_state_idle();
    void set_state_lock_open(uint32_t delay);
    void set_state_guest_call(uint32_t delay);
    void set_state_ap_connecting(uint32_t delay);
    void set_state_server_problem();
    void set_state_server_connecting(uint32_t delay);
    void set_state_network_problem();
    uint8_t get_state();
    Machine_state();
    ~Machine_state();
    Machine_state(led **leds);

private:
    led **leds;
    uint32_t delay;
    uint8_t current_state;
};
