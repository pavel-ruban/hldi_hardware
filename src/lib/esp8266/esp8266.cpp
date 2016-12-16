#include "esp8266.h"

    CmdHandler::CmdHandler() {
    }
    void CmdHandler::bind_uart(Uart *uart) {
        _uart = uart;
    }

    void CmdHandler::bind_esp(Esp8266 *esp) {
        _esp = esp;
    }

    CmdHandler::~CmdHandler() {

    }

    uint8_t CmdHandler::parse_command() {
        // Resetting and client mode engaging block.
        if (_esp->current_state == STATE_RESETTING) {
            if (strstr(command, "ready")) {
                _esp->change_mode();
                return INTERNAL_RESPONSE;
            }
        }

        if (_esp->current_state == STATE_WAITING_MODE_CHANGE) {
            if (strstr(command, "OK")) {
                _esp->is_ready_to_connect_to_hotspot = 1;
                if (_esp->connect_after_reset) {
                    _esp->connect_after_reset = 0;
                    _esp->send_request_to_connect();
                } else {
                    _esp->current_state = STATE_READY;
                    _esp->busy = 0;
                }
                return INTERNAL_RESPONSE;
            }
        }
        // HS/server connections and error handlers
        if (_esp->current_state == STATE_WAITING_WIFI_CONNECT) {
            if (strstr(command, "OK")) {
                _esp->is_connected_to_wifi = 1;
                _esp->current_state = STATE_READY;
                _esp->busy = 0;
                return INTERNAL_RESPONSE;
            }
        }

        if (_esp->current_state == STATE_WAITING_WIFI_CONNECT) { // Only wrong creditals tested, others PROBABLY same, but dunno.
            if (strstr(command, "FAIL")) {
                _esp->is_connected_to_wifi = 0;
                _esp->current_state = STATE_READY;
                _esp->busy = 0;
                return INTERNAL_RESPONSE;
            }
        }

        if (_esp->current_state == STATE_WAITING_IP_CONNECT) {
            if (strstr(command, "OK")) {
                _esp->is_connected_to_server = 1;
                _esp->current_state = STATE_READY;
                _esp->busy = 0;
                return INTERNAL_RESPONSE;
            }
        }

        if (_esp->current_state == STATE_WAITING_IP_CONNECT) { //Only dns fail tested, others PROBABLY same, but dunno.
            if (strstr(command, "ERROR") || strstr(command, "CLOSED")) {
                _esp->is_connected_to_server = 0;
                _esp->current_state = STATE_READY;
                _esp->busy = 0;
                return INTERNAL_RESPONSE;
            }
        }
        // Status and error handling.
        if (_esp->is_connected_to_wifi) {
            if (strstr(command, "CLOSED")) {
                _esp->is_connected_to_server = 0;
                return INTERNAL_RESPONSE;
            }
        }
        if (strstr(command, "STATUS:3")) {
            _esp->is_connected_to_wifi = 1;
            _esp->is_connected_to_server = 1;
            _esp->busy = 0;
            return INTERNAL_RESPONSE;
        }
        if (strstr(command, "STATUS:5")) {
            _esp->is_connected_to_wifi = 1;
            _esp->is_connected_to_server = 0;
            _esp->busy = 0;
            return INTERNAL_RESPONSE;
        }
        if (strstr(command, "STATUS:4")) {
            _esp->is_connected_to_wifi = 0;
            _esp->is_connected_to_server = 0;
            _esp->busy = 0;
            return INTERNAL_RESPONSE;
        }

        if (strstr(command, "status: 0") && _esp->current_state == STATE_WAITING_RESPONSE) {
            _esp->_machine_state->set_state_other_network_problem();
            _esp->current_state = STATE_READY;
            return INTERNAL_RESPONSE;
        }
        //Open trigger.
        if (strstr(command, "bo_test") && _esp->current_state == STATE_WAITING_RESPONSE) {
         //   if (strstr(command, "200")) {
                _esp->_cache_handler->addCard(_esp->last_tag_id, ACCESS_GRANTED);
                _esp->_machine_state->set_state_lock_open();
         //   }
//            if (strstr(command, "403")) {
//                _esp->_cache_handler->addEvent(_esp->last_tag_id, _esp->last_pcb_id, ACCESS_DENIED);
//            }
            if (strstr(command, "200") && strstr(command, "cache dump"))
            {
                //
            }
            _esp->current_state = STATE_READY;
            return INTERNAL_RESPONSE;
        }


        if (strstr(command, "biba") && _esp->current_state == STATE_WAITING_RESPONSE) {
            // _esp->_machine_state->set_state_lock_open();
            _esp->current_state = STATE_READY;
            return INTERNAL_RESPONSE;
        }
        //Коннект к серваку - статус 3.
        //Коннект к точке, без сервака - статус 5.
        //Нету коннекта к точке, (и к серваку, очевидно) - статус 4.
    }
//char commandbuf[50][COMMAND_SIZE];
//int commandpos[50][2];
//int test_count = 0;
    void CmdHandler::handle_uart_queue() {
        uint8_t buf = 0;
        uint8_t buf_start = 0;
        memset(command,'\0',COMMAND_SIZE);
        uint16_t iter = 0;
        //_uart->cyclo_buffer.back() == '\n'
        if (true) {
            typename Queue<uint8_t, 200>::iterator it = _uart->cyclo_buffer.begin();
            int buf_start = _uart->cyclo_buffer.start_index;
            int buf_end = _uart->cyclo_buffer.end_index;
            while (it.index != _uart->cyclo_buffer.end_index) {
                buf = it.index;
                command[iter] = *it;
                ++it;
                if(buf > it.index)
                {
                } else
                    iter++;
            }
            _uart->cyclo_buffer.start_index = it.index;
            _uart->last_string_ready = 0;
//            if (command[0]) {
//                commandpos[test_count][0] = buf_start;
//                commandpos[test_count][1] = buf_end;
//                strcpy(commandbuf[test_count], command);
//                if (test_count >= 49) {
//                    int dgd = 0;
//                }
//                test_count++;
//            }

            if (command[0]) {
                parse_command();
            }
        }
    }

Esp8266::Esp8266(Uart *uart, Machine_state *machine_state, Cache_handler *cache_handler) {
    _cache_handler = cache_handler;
    _machine_state = machine_state;
    _uart = uart;
    is_connected_to_wifi = 0;
    is_connected_to_server = 0;
    is_ready_to_connect_to_hotspot = 0;
    message_sent = 0;
    is_authorized = 0;
    busy = 0;
    hndl.bind_uart(uart);
    hndl.bind_esp(this);
}

Esp8266::~Esp8266() {

}

void Esp8266::invoke_uart_handler() {

    hndl.handle_uart_queue();
}

void Esp8266::Delay(uint32_t nCount)
{
    for(; nCount != 0; nCount--);
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

void Esp8266::reset() {
    _uart->cyclo_buffer.clear();
    busy = 1;
    current_state = STATE_RESETTING;
    GPIO_ResetBits(ESP8266_RESET_PORT,ESP8266_RESET_PIN);
    Delay(100);
    GPIO_SetBits(ESP8266_RESET_PORT,ESP8266_RESET_PIN);

}

void Esp8266::send_request(char* request) {
    message_sent = 0;
    clear_buffer();
    strcat(buffer_string, "AT+CIPSEND=");
    strcat(buffer_string, int_to_string(strlen(request) + strlen(NODE_ID) + 10));
    strcat(buffer_string, "\r\n");
    _uart->send(buffer_string);
    Delay(30000);
    clear_buffer();
    strcat(buffer_string, "node_id: ");
    strcat(buffer_string, NODE_ID);
    strcat(buffer_string, "\n");
    strcat(buffer_string, request);
    _uart->send(buffer_string);
    current_state = STATE_WAITING_RESPONSE;

}

void Esp8266::send_event(uint8_t tag_id[], uint8_t rc522_number, uint32_t time) { //Untested, ctrlc-ctrlv
    char test_buf[100];
    memset(test_buf, '\0', 100);
    if (is_connected_to_server && is_connected_to_wifi && current_state == STATE_READY) {
        strcat(test_buf,"action: event dump\nuid: ");
        strcat(test_buf, int_to_string(tag_id[0]));
        strcat(test_buf, ":");
        strcat(test_buf, int_to_string(tag_id[1]));
        strcat(test_buf, ":");
        strcat(test_buf, int_to_string(tag_id[2]));
        strcat(test_buf, ":");
        strcat(test_buf, int_to_string(tag_id[3]));
        strcat(test_buf, "\nnode_number: ");
        strcat(test_buf, int_to_string(rc522_number));
        strcat(test_buf, "\ntime: ");
        strcat(test_buf, int_to_string(time));
        strcat(test_buf, "\n\n\n");
        send_request(test_buf);
    }
}

void Esp8266::send_access_request(uint8_t tag_id[], uint8_t rc522_number) {
    char test_buf[100];
    memset(test_buf, '\0', 100);
    if (is_connected_to_server && is_connected_to_wifi) {
        strcat(test_buf,"action: access request\nuid: ");
        strcat(test_buf, int_to_string(tag_id[0]));
        strcat(test_buf, ":");
        strcat(test_buf, int_to_string(tag_id[1]));
        strcat(test_buf, ":");
        strcat(test_buf, int_to_string(tag_id[2]));
        strcat(test_buf, ":");
        strcat(test_buf, int_to_string(tag_id[3]));
        strcat(test_buf, "\n\n\n");
        for (uint8_t il = 0; il < 4; ++il) {
            last_tag_id[il] = tag_id[il];
        }
        last_pcb_id = rc522_number;
        send_request(test_buf);
    }
}

uint8_t Esp8266::connect_to_ip(char* ip, char* port) {

    if (current_state == STATE_READY) {
        current_state = STATE_WAITING_IP_CONNECT;
        busy = 1;
        is_connected_to_server = 0;
        clear_buffer();
        strcat(buffer_string, "AT+CIPSTART=\"TCP\",\"");
        strcat(buffer_string, ip);
        strcat(buffer_string, "\",");
        strcat(buffer_string, port);
        strcat(buffer_string, "\r\n");
        _uart->send(buffer_string);
    } else return current_state;
}

uint8_t Esp8266::set_server_timeout(uint8_t seconds) {
    if (current_state == STATE_READY) {
        clear_buffer();
        strcat(buffer_string, "AT+CIPSTO=");
        strcat(buffer_string, int_to_string(seconds));
        strcat(buffer_string, "\r\n");
        _uart->send(buffer_string);
        current_state = STATE_WAITING_RESPONSE;
    } else return current_state;
}

uint8_t Esp8266::disconnect_from_server() {
    if (current_state == STATE_READY) {
        clear_buffer();
        strcat(buffer_string, "AT+CIPCLOSE");
        strcat(buffer_string, "\r\n");
        _uart->send(buffer_string);
        current_state = STATE_WAITING_RESPONSE;
    } else return current_state;
}

uint8_t Esp8266::refresh_status() {
    busy = 1;
    //if (current_state == STATE_READY) {
        _uart->send("AT+CIPSTATUS\r\n");
   // } else return current_state;
}

void Esp8266::send_request_to_connect() {
    current_state = STATE_WAITING_WIFI_CONNECT;
    clear_buffer();
    strcat(buffer_string, "AT+CWJAP=\"");
    strcat(buffer_string, ssid);
    strcat(buffer_string, "\",\"");
    strcat(buffer_string, password);
    strcat(buffer_string, "\"\r\n");
    _uart->send(buffer_string);
}

void Esp8266::change_mode() {
    current_state = STATE_WAITING_MODE_CHANGE;
    _uart->send("AT+CWMODE=1\r\n");

}
void Esp8266::save_creditals(char* ssid, char* password) {
    this->ssid = ssid;
    this->password = password;
}


void Esp8266::connect_to_wifi_by_creditals(char* ssid, char* password) {
    this->ssid = ssid;
    this->password = password;
    is_connected_to_server = 0;
    is_connected_to_wifi = 0;
    connect_after_reset = 1;
    reset();
}

void Esp8266::connect_to_wifi() {
    is_connected_to_server = 0;
    is_connected_to_wifi = 0;
    connect_after_reset = 1;
    reset();
}
