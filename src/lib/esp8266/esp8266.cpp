#include "esp8266.h"

Esp8266::Esp8266(Uart *uart) {
    _uart = uart;
    is_connected_to_wifi = 0;
    message_sent = 0;
    is_authorized = 0;
}

Esp8266::~Esp8266() {

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

char* Esp8266::strcat(char *dest, const char *src) {
    uint32_t i,j;
    for (i = 0; dest[i] != '\0'; i++)
        ;
    for (j = 0; src[j] != '\0'; j++)
        dest[i+j] = src[j];
    dest[i+j] = '\0';
    return dest;
}

char* Esp8266::int_to_string(uint32_t i) {
    uint8_t loop = 0;
    uint32_t buf_i = i;
    if (i / 10 == 0) {
        buf[0] = i + 48;
        buf[1] = 0;
        return buf;
    }
    for (loop = 0; buf_i >= 10; loop++) {
        buf_i = buf_i / 10;
    }
    buf[loop + 1] = 0;
    for (loop; loop > 0; loop--) {
        buf[loop] = i % 10 + 48;
        i = i / 10;
    }
    buf[0] = i + 48;
    return buf;
}

void Esp8266::clear_buffer() {
    for (uint16_t i = 0; i < BUFFER_SIZE; i++) {
        buffer_string[i] = 0;
    }
}

void Esp8266::send_request(char* request) {
    message_sent = 0;

    clear_buffer();
    strcat(buffer_string, "AT+CIPSEND=");
    strcat(buffer_string, int_to_string(strlen(request) + 41));
    strcat(buffer_string, "\r\n");
    _uart->send(buffer_string);
    Delay(10000);
    clear_buffer();
    strcat(buffer_string, "node_id: ");
    strcat(buffer_string, NODE_ID);
    strcat(buffer_string, "\n");
    strcat(buffer_string, request);
    _uart->send(buffer_string);
    current_state = STATE_WAITING_RESPONSE;

}

uint8_t Esp8266::connect_to_ip(char* ip, char* port) {
    if (current_state == STATE_READY) {
        clear_buffer();
        strcat(buffer_string, "AT+CIPSTART=\"TCP\",\"");
        strcat(buffer_string, ip);
        strcat(buffer_string, "\",");
        strcat(buffer_string, port);
        strcat(buffer_string, "\r\n");
        _uart->send(buffer_string);
        current_state = STATE_WAITING_IP_CONNECT;
    } else return current_state;
}


uint8_t Esp8266::recieve_string() {
    if (_uart->last_string_ready) {
        for (uint16_t i = 0; i < strlen(_uart->last_string); i++) {
            last_string[i] = _uart->last_string[i];
    }
        return STRING_READY;
    } else return STRING_IS_NOT_READY;
}

uint8_t Esp8266::refresh_status() {
    if (current_state == STATE_READY) {
        _uart->send("AT+CIPSTATUS\r\n");
    } else return current_state;
}

void Esp8266::send_request_to_connect() {
    strcat(buffer_string, "AT+CWJAP=\"");
    strcat(buffer_string, ssid);
    strcat(buffer_string, "\",\"");
    strcat(buffer_string, password);
    strcat(buffer_string, "\"\r\n");
    _uart->send(buffer_string);
    current_state = STATE_WAITING_WIFI_CONNECT;
}

uint8_t Esp8266::handle_response() {
    recieve_string();
    if (current_state == STATE_WAITING_MODE_CHANGE) {
        if (strstr(last_string, "OK")) {
            send_request_to_connect();
            return INTERNAL_RESPONSE;
        }
    }
    if (current_state == STATE_WAITING_WIFI_CONNECT) {
        if (strstr(last_string, "OK")) {
            is_connected_to_wifi = 1;
            current_state = STATE_READY;
            return INTERNAL_RESPONSE;
        }
    }
    if (current_state == STATE_WAITING_IP_CONNECT) {
        if (strstr(last_string, "OK")) {
            is_connected_to_server = 1;
            current_state = STATE_READY;
            return INTERNAL_RESPONSE;
        }
    }

    if (current_state == STATE_WAITING_RESPONSE) {
        if (strstr(last_string, "SEND OK")) {
            message_sent = 1;
            current_state = STATE_READY;
            return INTERNAL_RESPONSE;
        }
    }

    if (is_connected_to_wifi) {
        if (strstr(last_string, "CLOSED")) {
            is_connected_to_server = 0;
            return INTERNAL_RESPONSE;
        }
    }
    if (strstr(last_string, "STATUS:5")) {
        is_connected_to_wifi = 1;
        return INTERNAL_RESPONSE;
    }
    if (strstr(last_string, "STATUS:4")) {
        is_connected_to_wifi = 0;
        return INTERNAL_RESPONSE;
    }
    return EXTERNAL_RESPONSE;
}

void Esp8266::connect_to_wifi(char* ssid, char* password) {
    this->ssid = ssid;
    this->password = password;
    _uart->send("AT+CWMODE=1\r\n");
    current_state = STATE_WAITING_MODE_CHANGE;
}
