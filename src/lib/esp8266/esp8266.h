#pragma once
#pragma pack(1)
#include <stdint.h>
#include <binds.h>
#include "uart/uart.h"
#include "config.h"
#include <utils.h>
#include <scheduler/include/scheduler.h>

#define BUFFER_SIZE 100

#define STRING_IS_NOT_READY 1
#define STRING_READY 1

//Esp8266 states.
#define STATE_READY 0
#define STATE_WAITING_MODE_CHANGE 1
#define STATE_WAITING_WIFI_CONNECT 2
#define STATE_WAITING_IP_CONNECT 3
#define STATE_WAITING_RESPONSE 5
#define STATE_RESETTING 4
#define AP_CONNECT_TIMEOUT 10000
#define SERVER_CONNECT_TIMEOUT 10000

//Esp8266

//Esp8266 response types
#define INTERNAL_RESPONSE 1
#define EXTERNAL_RESPONSE 0

class CmdHandler {
private:

    Uart *_uart;

public:
    char command[60];
    char test_ = 0;
    CmdHandler();
    ~CmdHandler();
    void bind_uart(Uart *uart);
    void handle_uart_queue();
};

class Esp8266{
private:
    CmdHandler hndl;
    char last_string[RECV_STRING_MAX_SIZE];
    char buffer_string[BUFFER_SIZE];
    void send_request_to_connect();
    char buf[10] = {0};
public:
    void invoke_uart_handler();
    uint8_t disconnect_from_server();
    uint8_t set_server_timeout(uint8_t seconds);
    void reset();
    void change_mode();
    char* int_to_string(uint32_t i);
    void clear_buffer();
    char* strcat(char *dest, const char *src);
    uint8_t refresh_status();
    uint8_t connect_to_ip(char* ip, char* port);
    char * ssid;
    char * password;
    uint8_t is_connected_to_wifi;
    uint8_t is_connected_to_server;
    uint8_t current_state;
    uint8_t busy;
    uint8_t is_authorized;
    uint8_t message_sent;
    char* strstr(char *haystack, const char *needle);
    Uart *_uart;
    Esp8266(Uart *uart);
    ~Esp8266();
    uint8_t wifi_connected;
    uint32_t ip_address;
    uint8_t handle_response();
    uint8_t recieve_string();
    void send_request(char* request);
    void Delay(uint32_t nCount);

    void connect_to_wifi(char* ssid, char* password);
    void connect_to_wifi();
};
