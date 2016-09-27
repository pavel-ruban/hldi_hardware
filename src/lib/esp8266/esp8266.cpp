#include "esp8266.h"

Esp8266::Esp8266(Uart *uart){
    _uart = uart;
    is_connected_to_wifi = 0;
}

Esp8266::~Esp8266(){

}

char* Esp8266::strstr(char *haystack, const char *needle) {
    if (haystack == NULL || needle == NULL) {
        return NULL;
    }
    for ( ; *haystack; haystack++) {
        const char *h, *n;
        for (h = haystack, n = needle; *h && *n && (*h == *n); ++h, ++n) {
        }
        if (*n == '\0') {
            return haystack;
        }
    }
    return NULL;
}

uint8_t Esp8266::connect_to_ip(char* ip, char* port) {
    if (current_state == STATE_READY) {
        _uart->send("AT+CIPSTART=\"TCP\",\"172.217.22.46\",80\r\n");
        current_state = STATE_WAITING_IP_CONNECT;
    } else return current_state;
}

uint8_t Esp8266::recieve_string() {
    if (_uart->last_string_ready) {
       // last_string = _uart->last_string;
        return STRING_READY;
    } else return STRING_IS_NOT_READY;
}

uint8_t Esp8266::refresh_status() {
    if (current_state == STATE_READY) {
        _uart->send("AT+CIPSTATUS\r\n");
    } else return current_state;
}

void Esp8266::send_request_to_connect() {
    _uart->send("AT+CWJAP=\"i20.pub\",\"i20biz2015\"\r\n");
    current_state = STATE_WAITING_WIFI_CONNECT;
}

uint8_t Esp8266::handle_responce() {
    recieve_string();
    if (current_state == STATE_WAITING_MODE_CHANGE) {
        if (strstr(last_string, "OK")) {
            send_request_to_connect();
            return INTERNAL_RESPONCE;
        }
    }
    if (current_state == STATE_WAITING_WIFI_CONNECT) {
        if (strstr(last_string, "OK")) {
            is_connected_to_wifi = 1;
            current_state = STATE_READY;
            return INTERNAL_RESPONCE;
        }
    }
    if (current_state == STATE_WAITING_IP_CONNECT) {
        if (strstr(last_string, "OK")) {
            is_connected_to_server = 1;
            current_state = STATE_READY;
            return INTERNAL_RESPONCE;
        }
    }
    if (is_connected_to_wifi) {
        if (strstr(last_string, "CLOSED")) {
            is_connected_to_server = 0;
            return INTERNAL_RESPONCE;
        }
    }
    return EXTERNAL_RESPONCE;
}


void Esp8266::connect_to_wifi(char* ssid, char* password) {
    this->ssid = ssid;
    this->password = password;
    _uart->send("AT+CWMODE=1\r\n");
    current_state = STATE_WAITING_MODE_CHANGE;
}
