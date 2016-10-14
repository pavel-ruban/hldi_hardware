#pragma once
#pragma pack(1)
#include <stdint.h>
#include "uart/uart.h"
#include "config.h"
#include "binds.h"

#define BUFFER_SIZE 100

#define STRING_IS_NOT_READY 1
#define STRING_READY 1

//Esp8266 states.
#define STATE_READY 0
#define STATE_WAITING_MODE_CHANGE 1
#define STATE_WAITING_WIFI_CONNECT 2
#define STATE_WAITING_IP_CONNECT 3
#define STATE_WAITING_RESPONSE 5
#define STATE_BUSY 4;

//Esp8266

//Esp8266 response types
#define INTERNAL_RESPONSE 1
#define EXTERNAL_RESPONSE 0

class Esp8266{
private:
    char last_string[RECV_STRING_MAX_SIZE];
    char buffer_string[BUFFER_SIZE];
    void send_request_to_connect();
    char buf[10] = {0};
public:
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

    void connect_to_wifi(char* ssid, char* password);
};
