#include "esp8266.h"

int strtoint( const char * str )
{
    int val = 0;
    while( *str ) {
        val = val*10 + (*str++ - '0');
    }
    return val;
}

//char int2hex_char(uint8_t byte) {
//    uint8_t first_digit = byte >>
//}



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

//    uint16_t CmdHandler::find_first_crlf() {
//        uint16
//    }
//
//    uint16_t CmdHandler::find_last_crlf() {
//
//    }
    uint8_t CmdHandler::parse_param(char* param, char* retval) {
        char* start_index = _esp->strstr_b(command, param, COMMAND_SIZE);
        if (!start_index)
            return 0;
    while(*start_index != ':') {
        start_index++;
    }
    start_index++;
    start_index++;
        while(*start_index != '\n') {
            *retval = *start_index;
            retval++;
            start_index++;
        }
    }

    uint8_t CmdHandler::parse_uid(char* uid_string, uint8_t* uid_bytes) {
        char bytes[4];
        for (uint8_t i = 0; i < 4; ++i) {
            while (*uid_string && *uid_string != '-') {
                uid_bytes[i] *= 10;
                uid_bytes[i] += (*uid_string - '0');
                uid_string++;
            }
            uid_string++;
        }

    }

    uint8_t CmdHandler::parse_command() {
        // Resetting and client mode engaging block.
        if (_esp->current_state == STATE_RESETTING) {
            if (_esp->strstr_b(command, "ready", COMMAND_SIZE)) {
                _esp->awaiting_system_answer = 0;
                _esp->change_mode();
                return INTERNAL_RESPONSE;
            }
        }

        if (_esp->current_state == STATE_WAITING_MODE_CHANGE) {
            if (_esp->strstr_b(command, "OK", COMMAND_SIZE)) {
                _esp->awaiting_system_answer = 0;
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
            if (_esp->strstr_b(command, "OK", COMMAND_SIZE)) {
                //_uart->crlf_count = 0;
                _esp->awaiting_system_answer = 0;
                _esp->is_connected_to_wifi = 1;
                _esp->current_state = STATE_READY;
                _esp->busy = 0;
                return INTERNAL_RESPONSE;
            }
        }

        if (_esp->current_state == STATE_WAITING_WIFI_CONNECT) { // Only wrong creditals tested, others PROBABLY same, but dunno.
            if (_esp->strstr_b(command, "FAIL", COMMAND_SIZE)) {
                _esp->awaiting_system_answer = 0;
                _esp->is_connected_to_wifi = 0;
                _esp->current_state = STATE_READY;
                _esp->busy = 0;
                return INTERNAL_RESPONSE;
            }
        }

        if (_esp->current_state == STATE_WAITING_IP_CONNECT) {
            if (_esp->strstr_b(command, "OK", COMMAND_SIZE)) {
                _esp->awaiting_system_answer = 0;
                _esp->attempts_done = 0;
                _esp->is_connected_to_server = 1;
                _esp->current_state = STATE_READY;
                _esp->busy = 0;
                return INTERNAL_RESPONSE;
            }
        }

        if (_esp->current_state == STATE_WAITING_IP_CONNECT) { //Only dns fail tested, others PROBABLY same, but dunno.
            if (_esp->strstr_b(command, "ERROR", COMMAND_SIZE) || _esp->strstr_b(command, "CLOSED", COMMAND_SIZE)) {
                _esp->awaiting_system_answer = 0;
                _esp->is_connected_to_server = 0;
                _esp->current_state = STATE_READY;
                _esp->busy = 0;
                return INTERNAL_RESPONSE;
            }
        }
        // Status and error handling.
        if (_esp->is_connected_to_wifi) {
            if (_esp->strstr_b(command, "CLOSED", COMMAND_SIZE)) {
                _esp->is_connected_to_server = 0;
                return INTERNAL_RESPONSE;
            }
        }
        if (_esp->strstr_b(command, "STATUS:3", COMMAND_SIZE)) {
            _esp->is_connected_to_wifi = 1;
            _esp->is_connected_to_server = 1;
            _esp->busy = 0;
            return INTERNAL_RESPONSE;
        }
        if (_esp->strstr_b(command, "STATUS:5", COMMAND_SIZE)) {
            _esp->is_connected_to_wifi = 1;
            _esp->is_connected_to_server = 0;
            _esp->busy = 0;
            return INTERNAL_RESPONSE;
        }
        if (_esp->strstr_b(command, "STATUS:4", COMMAND_SIZE)) {
            _esp->is_connected_to_wifi = 0;
            _esp->is_connected_to_server = 0;
            _esp->busy = 0;
            return INTERNAL_RESPONSE;
        }

        if (_esp->strstr_b(command, "status: 0", COMMAND_SIZE) && _esp->current_state == STATE_WAITING_RESPONSE) {
            _esp->_machine_state->set_state_other_network_problem();
            _esp->current_state = STATE_READY;
            return INTERNAL_RESPONSE;
        }
        //Open trigger.
        if (_esp->strstr_b(command, "bo_test", COMMAND_SIZE) && _esp->current_state == STATE_WAITING_RESPONSE) {
            if (_esp->strstr_b(command, "action: access request", COMMAND_SIZE) && !_esp->strstr_b(command, "action: cache dump", COMMAND_SIZE)) {
                uint8_t int_access_result = 3;
                char time[13] = {0};
                parse_param("time", time);
                char uid[17] = {0};
                parse_param("uid", uid);
                uint8_t uid_bytes[4] = {0};
                parse_uid(uid, uid_bytes);
                char pcd_number[2] = {0};
                parse_param("pcd-number", pcd_number);
                char access_result[3] = {0};
                parse_param("status", access_result);

                if (access_result[0] == '2' && access_result[1] == '0' && access_result[2] == '0') {
                    if (_esp->strstr_b(command, "access: granted", COMMAND_SIZE))
                        int_access_result = ACCESS_GRANTED;
                    if (_esp->strstr_b(command, "access: denied", COMMAND_SIZE))
                        int_access_result = ACCESS_DENIED;
                } else if (access_result[0] == '4' && access_result[1] == '0' && access_result[2] == '3') {
                    int_access_result = DEFAULT_NOT_CACHED_BEHAVIOUR;
                }


                _esp->_cache_handler->deleteEvent(uid_bytes, strtoint(pcd_number), strtoint(time));
                _esp->_cache_handler->addCard(_esp->last_tag_id, int_access_result);
                if (int_access_result == ACCESS_GRANTED)
                    _esp->_machine_state->set_state_lock_open(1);
                else
                    _esp->_machine_state->set_state_access_denied();
            }

            if (_esp->strstr_b(command, "dump", COMMAND_SIZE)) {
                uint8_t int_access_result = 3;
                char time[13] = {0};
                parse_param("time", time);
                char uid[17] = {0};
                parse_param("uid", uid);
                uint8_t uid_bytes[4] = {0};
                parse_uid(uid, uid_bytes);
                char pcd_number[2] = {0};
                parse_param("pcd-number", pcd_number);
                char access_result[3] = {0};
                parse_param("status", access_result);

                if (access_result[0] == '2' && access_result[1] == '0' && access_result[2] == '0') {
                    _esp->_cache_handler->deleteEvent(uid_bytes, strtoint(pcd_number), strtoint(time));
                    //_esp->_cache_handler->addCard(_esp->last_tag_id, int_access_result);
                }
            }

            if (_esp->strstr_b(command, "action: open", COMMAND_SIZE)) {
                _esp->_machine_state->set_state_lock_open(7);
            }

            if (_esp->strstr_b(command, "time-sync", COMMAND_SIZE)) {
                if (_esp->strstr_b(command, "status: 200", COMMAND_SIZE)) {
                    _esp->time_synced = TIME_SYNCED;
                }
            }

            _esp->current_state = STATE_READY;
            return INTERNAL_RESPONSE;
        }


        if (_esp->strstr_b(command, "biba", COMMAND_SIZE) && _esp->current_state == STATE_WAITING_RESPONSE) {
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
            typename Queue<uint8_t, USART_RING_BUFFER_SIZE>::iterator it = _uart->cyclo_buffer.begin();
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
            _uart->crlf_count = 0;
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



char* Esp8266::strstr_b(char *haystack, const char *needle, uint16_t size) {
    if (haystack == NULL || needle == NULL) {
        return NULL;
    }
    for (uint16_t passed = 0 ; passed < size; passed++, haystack++) {
        const char *h, *n;
        for (h = haystack, n = needle; *h && *n && (*h == *n); ++h, ++n) {
        } //
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
    awaiting_system_answer = 1;
    busy = 1;
    attempts_done = 0;
    reset_time = 0;
    current_state = STATE_RESETTING;
    GPIO_ResetBits(ESP8266_RESET_PORT,ESP8266_RESET_PIN);
    Delay(100);
    GPIO_SetBits(ESP8266_RESET_PORT,ESP8266_RESET_PIN);
}

void Esp8266::send_request(char* request, uint8_t w8resp) {
    current_state = STATE_BUSY;
    clear_buffer();
    strcat(buffer_string, "AT+CIPSEND=");
    strcat(buffer_string, int_to_string(strlen(request) + strlen(NODE_ID) + 10));
    strcat(buffer_string, "\r\n");
    _uart->send(buffer_string);
    Delay(30000);
    clear_buffer();
    strcat(buffer_string, "node-id: ");
    strcat(buffer_string, NODE_ID);
    strcat(buffer_string, "\n");
    strcat(buffer_string, request);
    _uart->send(buffer_string);
    if (w8resp) {
        current_state = STATE_WAITING_RESPONSE;
        request_time = ticks;
    }
    else
        current_state = STATE_READY;

}

void Esp8266::sync_time() {
    if (time_synced)
        return;
    time_synced = TIME_SYNC_IN_PROGRESS;
    char buf[40] = {0};
    strcat(buf, "time: ");
    strcat(buf, int_to_string(ticks));
    strcat(buf, "\naction: time-sync");
    strcat(buf, "\n\n\n");
    send_request(buf, 1);
}

void Esp8266::send_event(uint8_t tag_id[], uint8_t rc522_number, uint32_t time, uint8_t access_result, uint8_t cache_status) { //Untested, ctrlc-ctrlv
    char test_buf[150];
    memset(test_buf, '\0', 150);
    if (is_connected_to_server && is_connected_to_wifi && current_state == STATE_READY) {
        strcat(test_buf,"action: event dump\nuid: ");
        strcat(test_buf, int_to_string(tag_id[0]));
        strcat(test_buf, "-");
        strcat(test_buf, int_to_string(tag_id[1]));
        strcat(test_buf, "-");
        strcat(test_buf, int_to_string(tag_id[2]));
        strcat(test_buf, "-");
        strcat(test_buf, int_to_string(tag_id[3]));
        strcat(test_buf, "\npcd-number: ");
        strcat(test_buf, int_to_string(rc522_number));
        strcat(test_buf, "\ntime: ");
        strcat(test_buf, int_to_string(time));
        strcat(test_buf, "\naccess: ");
        strcat(test_buf, int_to_string(access_result));
        strcat(test_buf, "\ncached: ");
        strcat(test_buf, int_to_string(cache_status));
        strcat(test_buf, "\n\n\n");
        send_request(test_buf, 1);
    }
}

void Esp8266::send_access_request(uint8_t tag_id[], uint8_t rc522_number, uint32_t time) {
    char test_buf[100];
    memset(test_buf, '\0', 100);
    if (is_connected_to_server && is_connected_to_wifi) {
        strcat(test_buf,"action: access request\nuid: ");
        strcat(test_buf, int_to_string(tag_id[0]));
        strcat(test_buf, "-");
        strcat(test_buf, int_to_string(tag_id[1]));
        strcat(test_buf, "-");
        strcat(test_buf, int_to_string(tag_id[2]));
        strcat(test_buf, "-");
        strcat(test_buf, int_to_string(tag_id[3]));
        strcat(test_buf, "\ntime: ");
        strcat(test_buf, int_to_string(time));
        strcat(test_buf, "\npcd-number: ");
        strcat(test_buf, int_to_string(rc522_number));
        strcat(test_buf, "\n\n\n");
        for (uint8_t il = 0; il < 4; ++il) {
            last_tag_id[il] = tag_id[il];
        }
        last_pcb_id = rc522_number;
        send_request(test_buf, 1);
    }
}

uint8_t Esp8266::connect_to_ip(char* ip, char* port) {

    attempts_done++;

    if (current_state == STATE_READY) {
        awaiting_system_answer = 1;
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
        request_time = ticks;
    } else return current_state;
}

uint8_t Esp8266::set_server_timeout(uint8_t seconds) {
    if (current_state == STATE_READY) {
        clear_buffer();
        strcat(buffer_string, "AT+CIPSTO=");
        strcat(buffer_string, int_to_string(seconds));
        strcat(buffer_string, "\r\n");
        _uart->send(buffer_string);
        request_time = ticks;
        current_state = STATE_WAITING_RESPONSE;
    } else return current_state;
}

uint8_t Esp8266::disconnect_from_server() {
    if (current_state == STATE_READY) {
        clear_buffer();
        strcat(buffer_string, "AT+CIPCLOSE");
        strcat(buffer_string, "\r\n");
        _uart->send(buffer_string);
        request_time = ticks;
        current_state = STATE_WAITING_RESPONSE;
    } else return current_state;
}

uint8_t Esp8266::refresh_status() {
    busy = 1;
    awaiting_system_answer = 1;
    //if (current_state == STATE_READY) {
        _uart->send("AT+CIPSTATUS\r\n");
   // } else return current_state;
}

void Esp8266::send_request_to_connect() {
    awaiting_system_answer = 1;
    current_state = STATE_WAITING_WIFI_CONNECT;
    clear_buffer();
    strcat(buffer_string, "AT+CWJAP=\"");
    strcat(buffer_string, ssid);
    strcat(buffer_string, "\",\"");
    strcat(buffer_string, password);
    strcat(buffer_string, "\"\r\n");
    _uart->send(buffer_string);
    request_time = ticks;
}

void Esp8266::change_mode() {
    awaiting_system_answer = 1;
    current_state = STATE_WAITING_MODE_CHANGE;
    _uart->send("AT+CWMODE=1\r\n");
    request_time = ticks;

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

void Esp8266::timeout_invalidation() {
    if (current_state == STATE_WAITING_RESPONSE && request_time + SERVER_CONNECT_TIMEOUT < ticks) {
        _uart->cyclo_buffer.clear();
        clear_buffer();
        memset(hndl.command,'\0',COMMAND_SIZE);
        current_state = STATE_READY;
        disconnect_from_server();
    }
}