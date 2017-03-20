/******************** (C) COPYRIGHT 2016 Pavel Ruban ********************
 * File Name          : c_entry.c
 * Author             : Pavel Ruban
 * Version            : V1.0
 * Date               : 12/06/2016
 * Description        : Main program body
 ******************************************************************************/

#pragma pack(1)

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>
#include <stdlib.h>

extern "C" {
	#include <stm32f10x_conf.h>
    #include <binds.h>
    #include <string.h>
}

#include "lib/rc522/mfrc522.h"
#include "include/utils.h"
#include <led/led.hpp>
#include "machine_state/machine_state.h"
#include <scheduler/include/scheduler.h>
#include <config.h>
#include "lib/led/led.hpp"
#include "uart/uart.h"
#include "lib/cache_handler/cache_handler.h"
#include "esp8266/esp8266.h"
#include <stdio.h>



#include "include/queue.h"
#include "lib/array/array.h"

#define EVENT_ACCESS_REQ 1
#define ACCESS_GRANTED 0x1
#define ACCESS_DENIED 0x0

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
SPI_InitTypeDef SPI_InitStructure;
GPIO_InitTypeDef GPIO_InitStructure;
ErrorStatus HSEStartUpStatus;
led *leds[LED_QUANTITY] = {NULL};
Machine_state machine_state(leds);


extern uint8_t mac_addr[6];
extern uint8_t enc28j60_revid;
volatile uint32_t ticks;
int timerValue = 0;


Scheduler<Event<led>, 100> led_scheduler;
Cache_handler cache_handler(EXPIRATION_TIME);
uint32_t check_amount = 0;
uint64_t tim4_counter = 0;

Scheduler<Event<Machine_state>, 100> state_scheduler;
Scheduler<Event<Esp8266>, 5> connection_scheduler;
Uart uart(UART1, 115200);
Uart uart3(UART3, 115200);
uint8_t test_flag = 0;


Esp8266 wifi(&uart, &machine_state, &cache_handler);
/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);
void NVIC_Configuration(void);
//void Delay(vu32 nCount);
extern "C" void custom_asm();
//void rc522_irq_prepare();
void RTC_Configuration();

extern "C" void reset_asm();

void Delay(uint32_t nCount)
{
    for(; nCount != 0; nCount--);
}

uint32_t test_ip_conn_counter = 0;
void rc522_irq_prepare()
{
    mfrc522_write(BitFramingReg, 0x07); // TxLastBists = BitFramingReg[2..0]	???

    // Clear all interrupts flags.
    mfrc522_write(ComIrqReg, (uint8_t) ~0x80);
    uint8_t status = mfrc522_read(Status1Reg);
    // Start timer.
    mfrc522_write(ControlReg, 1 << TStartNow);
}

char* strstr(char *haystack, const char *needle) {
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

extern "C" void __initialize_hardware_early()
{
	/* Configure the system clocks */
	RCC_Configuration();

	/* NVIC Configuration */
	NVIC_Configuration();

	/* Set up real time clock */
	RTC_Configuration();
}

void RTC_Configuration()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

	PWR_BackupAccessCmd(ENABLE);

	RCC_BackupResetCmd(ENABLE);
	RCC_BackupResetCmd(DISABLE);

	RCC_LSICmd(ENABLE);
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
	RCC_RTCCLKCmd(ENABLE);

	RCC_LSEConfig(RCC_LSE_ON);
}

uint8_t get_pcd_id()
{
    return rc522_select == rc522_2_select ? RC522_PCD_2 : RC522_PCD_1;
}

uint8_t somi_access_check(uint8_t tag_id[], uint8_t node_id[], uint8_t pcd_id)
{
    // @todo use here network request.
    return 0xff;
}

uint8_t somi_access_check_1(uint8_t tag_id[], uint8_t rc522_number)
{
    if (!(machine_state.get_state() == MACHINE_STATE_ACCESS_DENIED || machine_state.get_state() == MACHINE_STATE_LOCK_OPEN)) {
        led_scheduler.invalidate(leds[LED_STATE_INDICATOR]);
        leds[LED_STATE_INDICATOR]->set_color(LED_COLOR_BLUE);
        leds[LED_STATE_INDICATOR]->set_blink(0,1000,1000);
        leds[LED_STATE_INDICATOR]->on();
    }


    if (cache_handler.checkCard(tag_id) == NOT_CACHED) {
        //machine_state.set_state_lock_open();
        if (wifi.is_connected_to_server && wifi.is_connected_to_wifi) {
            //cache_handler.addCard(tag_id, IN_PROGRESS);
            //Used for saving current time in sync usage. Between adding event and requesting some interrupts could be invoked.
            uint32_t trigger_time = ticks;
            cache_handler.addEvent(tag_id, rc522_number, CURRENTLY_UNKNOWN, NOT_CACHED, trigger_time, 0);
            wifi.send_access_request(tag_id, rc522_number, trigger_time);

            Event<Machine_state> deny_access(state_scheduler.get_current_time() + SERVER_CONNECT_TIMEOUT, &machine_state, &Machine_state::set_state_access_denied);
            state_scheduler.push(deny_access);

        } else {
            if (!(wifi.is_connected_to_server && wifi.is_connected_to_wifi)) {
                cache_handler.addEvent(tag_id, rc522_number, DEFAULT_NOT_CACHED_BEHAVIOUR, NOT_CACHED, ticks, 1);
                if (DEFAULT_NOT_CACHED_BEHAVIOUR == ACCESS_DENIED)
                    machine_state.set_state_access_denied();
                else
                    machine_state.set_state_lock_open(2);
            }
        }
        return DEFAULT_NOT_CACHED_BEHAVIOUR;
    }


    if (cache_handler.checkCard(tag_id) == ACCESS_GRANTED) {
        machine_state.set_state_lock_open(3);
        cache_handler.addEvent(tag_id, rc522_number, ACCESS_GRANTED, CACHED, ticks, 1);
        return ACCESS_GRANTED;
    }

    if (cache_handler.checkCard(tag_id) == ACCESS_DENIED) {
        machine_state.set_state_access_denied();
            cache_handler.addEvent(tag_id, rc522_number, ACCESS_DENIED, CACHED, ticks, 1);
        return ACCESS_DENIED;
    }

//    if (cache_handler.checkCard(tag_id) == NOT_CACHED) {
//        if (!(wifi.is_connected_to_server && wifi.is_connected_to_wifi)) {
//            cache_handler.addEvent(tag_id, rc522_number, DEFAULT_NOT_CACHED_BEHAVIOUR, NOT_CACHED);
//            machine_state.set_state_access_denied();
//        }
//        return ACCESS_DENIED;
//    }


    return 0xff;
}

void access_denied_signal()
{
    for(;;)
    {}
}

void open_node()
{
//    for(;;)
//    {}
//    // @todo implement here hardware EMI lock control.
//    static uint8_t x = 0;
//    x ^= 1;
//
//    if (x) GPIO_SetBits(GPIOA, GPIO_Pin_1);
//    else access_denied_signal();
}

void somi_event_handler(uint8_t event, uint8_t flags)
{
    switch (event)
    {
        case EVENT_ACCESS_REQ:
            if (flags & ACCESS_GRANTED) {
                open_node();
            }
            else {
                access_denied_signal();
            }
            break;
    }
}

uint32_t last_time_triggered = 0;
void rfid_irq_tag_handler()
{
    __disable_irq();

    uint8_t tag_id[16];
    memset(tag_id, 0, sizeof(tag_id));
    uint8_t status = mfrc522_get_card_serial(tag_id);
    __enable_irq();

    if (status == CARD_FOUND) {
        if (ticks - last_time_triggered > 800) //Delay between riggers; 600?
        {
            last_time_triggered = ticks;
            somi_access_check_1(tag_id, get_pcd_id());
            //somi_access_check_1(tag_id, get_pcd_id());
        }
    }

    // Activate timer.
    rc522_irq_prepare();
}

extern "C" void WRONG_IRQ_EXCEPTION()
{
	while (1) {}
}

extern "C" void EXTI4_IRQHandler()
{
    EXTI_ClearITPendingBit(EXTI_Line4);
}


extern "C" void TIM2_IRQHandler()
{
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
}

extern "C" void TIM3_IRQHandler()
{
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
}

extern "C" void TIM4_IRQHandler()
{
    ++tim4_counter;
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);

    if (uart.last_char_timing != 0 && (ticks - uart.last_char_timing) >= 4
    	&& uart.cyclo_buffer.end_index != uart.cyclo_buffer.start_index ) {

        wifi.invoke_uart_handler();
        check_amount++;
    }

    if (tim4_counter % 32 == 0) {
        cache_handler.forceInvalidateStuckEntries();
        if (wifi.current_state == STATE_READY && (ticks - uart.last_char_timing) >= 4
		&& wifi.is_connected_to_server) {

            if (cache_handler.eventExist() && !cache_handler.currentlyProcessing()) {
                tag_event buf_event = cache_handler.popOldestEvent();
                // wifi.send_access_request(buf_event.tag_id, buf_event.node, buf_event.event_time);
                wifi.send_event(
			buf_event.tag_id,
			buf_event.node,
			buf_event.event_time,
			buf_event.access_result,
			buf_event.cache_status
		);
            }
        }

        wifi.timeout_invalidation();
    }
}


extern "C" void EXTI2_IRQHandler()
{

}



extern "C" void USART3_IRQHandler()
{
    if (USART_GetITStatus(USART3, USART_IT_TC) != RESET)
    {
        USART_ClearITPendingBit(USART3, USART_IT_TC);
    }
}


extern "C" void USART1_IRQHandler()
{
    // Receive Data register not empty interrupt.
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        uart.prev_byte = uart.last_byte;
        uart.last_byte = USART_ReceiveData(USART1);
        //GPIO_SetBits(EM_LOCK_PORT, EM_LOCK_PIN);
        //counter++;
//        if (uart.prev_byte == '\r' && uart.last_byte == '\n') {
//            uart.crlf_count++;
//        }
//        if (uart.prev_byte == '+' && uart.last_byte == 'I') {
//            uart.crlf_count++;
//        }
        uart.last_char_timing = ticks;
        uart.cyclo_buffer.push_back(uart.last_byte);
        if (uart.last_byte == '\n') {
            uart.last_string_ready = 1;
        }
    }
    // Transmission complete interrupt.
    GPIO_ResetBits(EM_LOCK_PORT, EM_LOCK_PIN);

    if (USART_GetITStatus(USART1, USART_IT_TC) != RESET)
    {
        USART_ClearITPendingBit(USART1, USART_IT_TC);
    }
}

char test_string;
extern "C" void SysTick_Handler(void)
{
    led_scheduler.handle();
    state_scheduler.handle();
    connection_scheduler.handle();
    //&& wifi._uart->crlf_count >= 2
    ticks++;
}

extern "C" void EXTI0_IRQHandler()
{
    if (!(machine_state.get_state() == MACHINE_STATE_LOCK_OPEN)) {
        machine_state.set_state_lock_open(4);
    }
	EXTI_ClearITPendingBit(EXTI_Line0);

}

extern "C" void EXTI1_IRQHandler()
{
    if (!(machine_state.get_state() == MACHINE_STATE_GUEST_CALL)) {
        machine_state.set_state_guest_call();
    }
    EXTI_ClearITPendingBit(EXTI_Line1);
}


extern "C" void EXTI15_10_IRQHandler()
{

}

extern "C" void EXTI9_5_IRQHandler()
{
    if (EXTI_GetITStatus(EXTI_Line8) == SET)
    {
        rc522_pcd_select(RC522_PCD_1);
    }
    else if (EXTI_GetITStatus(EXTI_Line9) == SET)
    {
        rc522_pcd_select(RC522_PCD_2);
    }

    // Get active interrupts from RC522.
    uint8_t mfrc522_com_irq_reg = mfrc522_read(ComIrqReg);

    // If some PICC answered handle it to retrieve additional data from it.
    if (mfrc522_com_irq_reg & (1 << RxIEn)) {
        //static uint8_t led_state = 0;

        // Acknowledge receive irq.
        mfrc522_write(ComIrqReg, 1 << RxIEn);

        // Attempt to retrieve tag ID and in case of success check node access.
        rfid_irq_tag_handler();

        rc522_irq_prepare();
    }
        // If it's timer IRQ then request RC522 to start looking for CARD again
        // and back control to the main thread.
    else if (mfrc522_com_irq_reg & TimerIEn) {
        // Down timer irq.
        mfrc522_write(ComIrqReg, TimerIEn);

        // As was checked, after Transceive_CMD the FIFO is emptied, so we need put PICC_REQALL
        // here again, otherwise PICC won't be able response to RC522.
        mfrc522_write(FIFOLevelReg, mfrc522_read(FIFOLevelReg) | 0x80); // flush FIFO data
        mfrc522_write(FIFODataReg, PICC_REQALL);

        // Unfortunately Transceive seems not terminates receive stage automatically if PICC doesn't respond.
        // so we again need activate command to enter transmitter state to pass PICC_REQALL cmd to PICC
        // otherwise it won't response due to the ISO 14443 standard.
        mfrc522_write(CommandReg, Transceive_CMD);

        // When data and command are correct issue the transmit operation.
        mfrc522_write(BitFramingReg, mfrc522_read(BitFramingReg) | 0x80);

        // Start timer. When it will end it again cause this IRQ handler to search the PICC.

        // Clear STM32 irq bit.
        if (rc522_select == rc522_1_select)
        {
            EXTI_ClearITPendingBit(EXTI_Line8);
        }
        else if (rc522_select == rc522_2_select)
        {
            EXTI_ClearITPendingBit(EXTI_Line9);
        }

        mfrc522_write(ControlReg, 1 << TStartNow);
        return;
    }

    if (rc522_select == rc522_1_select)
    {
        EXTI_ClearITPendingBit(EXTI_Line8);
    }
    else if (rc522_select == rc522_2_select)
    {
        EXTI_ClearITPendingBit(EXTI_Line9);
    }
}


extern "C" void interrupt_initialize();
extern "C" void initialize_systick();

extern "C" void __initialize_hardware()
{

    rc522_set_pins();
    rc522_2_set_pins();
    //enc28j60_set_pins();

    set_spi_registers();
    set_spi2_registers();



    // Bind GPIOA, GPIOB to APB2 bus.
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    // Configure GPIO.
    // RGB led pins.
    GPIO_InitStructure.GPIO_Pin = LED_INDICATOR_PIN_RED | LED_INDICATOR_PIN_GREEN | LED_INDICATOR_PIN_BLUE;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(LED_INDICATOR_PORT, &GPIO_InitStructure);

    //----------------test
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    /* Configure SCK and MOSI pins as Alternate Function Push Pull */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    //----------------end test

    // Buttons pins (BTN_OPEN, BTN_CALL).
    GPIO_InitStructure.GPIO_Pin = BTN_OPEN_PIN | BTN_CALL_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(BTN_CALL_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // Em lock pin.
    GPIO_InitStructure.GPIO_Pin = EM_LOCK_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(EM_LOCK_PORT, &GPIO_InitStructure);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    GPIO_InitStructure.GPIO_Pin = ESP8266_RESET_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(ESP8266_RESET_PORT, &GPIO_InitStructure);
   // GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    GPIO_ResetBits(ESP8266_RESET_PORT,ESP8266_RESET_PIN);
    GPIO_SetBits(ESP8266_RESET_PORT,ESP8266_RESET_PIN);

	initialize_systick();
}

extern "C" void __reset_hardware()
{
	reset_asm();
}

void GetClocks() {
    RCC_ClocksTypeDef clocks;
    RCC_GetClocksFreq(&clocks);

    uint32_t APB1_TIMCLK_Frequency;
    uint32_t APB2_TIMCLK_Frequency;

    if (clocks.HCLK_Frequency == clocks.PCLK1_Frequency)
        APB1_TIMCLK_Frequency = clocks.PCLK1_Frequency;
    else
        APB1_TIMCLK_Frequency = 2 * clocks.PCLK1_Frequency;

    if (clocks.HCLK_Frequency == clocks.PCLK2_Frequency)
        APB2_TIMCLK_Frequency = clocks.PCLK2_Frequency;
    else
        APB2_TIMCLK_Frequency = 2 * clocks.PCLK2_Frequency;
    //APB1, APB2 - 72MHz;
}

void InitializeTimer()
{
    RCC_ClocksTypeDef RCC_Clocks;
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //Подвесили таймер 3 на ABP1 шину
    RCC_GetClocksFreq(&RCC_Clocks);
    TIM_TimeBaseInitTypeDef timerInitStructure;
//    timerInitStructure.TIM_Prescaler = RCC_Clocks.HCLK_Frequency / 10000;
//    timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
//    timerInitStructure.TIM_Period = 10000;
//    timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
//    timerInitStructure.TIM_RepetitionCounter = 0;
//    TIM_TimeBaseInit(TIM3, &timerInitStructure);
//    TIM_Cmd(TIM3, ENABLE);
//    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);


//20 msec?
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    timerInitStructure.TIM_Prescaler = 720000-1;
    timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    timerInitStructure.TIM_Period = 10;
    timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    timerInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &timerInitStructure);
    TIM_Cmd(TIM4, ENABLE);
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

}

void connect_to_wifi (uint16_t connection_timeout, char* ssid, char* password) {
    wifi.save_creditals(ssid, password);
    wifi.connect_to_wifi();
    state_scheduler.invalidate(&machine_state);
    machine_state.set_state_ap_connecting();
    connection_scheduler.invalidate(&wifi);
    Event<Esp8266> try2connect(connection_scheduler.get_current_time() + connection_timeout, &wifi, &Esp8266::connect_to_wifi);
    connection_scheduler.push(try2connect);
    Event<Esp8266> try2connect1(connection_scheduler.get_current_time() + connection_timeout * 2, &wifi, &Esp8266::connect_to_wifi);
    connection_scheduler.push(try2connect1);
    Event<Esp8266> try2connect2(connection_scheduler.get_current_time() + connection_timeout * 3, &wifi, &Esp8266::connect_to_wifi);
    connection_scheduler.push(try2connect2);
}

void connect_to_server (uint16_t connection_timeout, char* ip, char* port) {
    //wifi.set_server_timeout(connection_timeout);
    if (wifi.attempts_done == 10) {
        wifi.attempts_done = 255;
        wifi.reset_time = ticks + TO_SERVER_PROBLEM;
        machine_state.set_state_server_problem();
        return;
    }
    if (wifi.attempts_done == 255 && wifi.reset_time > ticks) {
        connect_to_wifi(AP_CONNECT_TIMEOUT, "i20.biz", "BetFua2Feg");
    }
    if (machine_state.get_state() != MACHINE_STATE_SERVER_CONNECTING)
        machine_state.set_state_server_connecting();
    connection_scheduler.invalidate(&wifi);
    wifi.connect_to_ip(ip, port);
}


int *xx;
extern "C" int
main(void)
{
    uint64_t global_counter;
//    std::string ss("gxfdsfs");
//    int x = 0;
//    Uart uart2(UART2, 115200);
//
//    xx = (int *) malloc(334);
//    while (1) {
//        char s1[] = "Hello green cube";
//        char s2[] = "Lets go further";
//        int z = strlen(s1);
//        int z2 = strlen(s2);
//
//        char s4[100];
//
//        sprintf(s4, "Hello my greeting is %s Usually I say it %d times!!!\n\n", s1, 5);
//
//        uart2.send(s4);
//        ++x;
//        int y = 1;
//    }
    machine_state.bind_wifi(&wifi);
    led rgb_led(LED_TYPE_RGB, GPIOA, GPIO_Pin_1, GPIO_Pin_2, GPIO_Pin_3, LED_COLOR_WHITE);
    leds[LED_STATE_INDICATOR] =  &rgb_led;
    leds[LED_STATE_INDICATOR]->on();
    InitializeTimer();
    interrupt_initialize();
	__enable_irq();
    rc522_pcd_select(RC522_PCD_1);
//    __disable_irq();
//    set_spi2_registers();
    set_spi_registers();
    rc522_set_pins();
    mfrc522_init();
    rc522_irq_prepare();
    cache_handler.deleteEventById(10);
    wifi.Delay(0);
    uart.cyclo_buffer.data();

    connect_to_wifi(AP_CONNECT_TIMEOUT, "i20.biz", "BetFua2Feg");
    Delay(5000000);
    while (1)
	{
        global_counter++;
        if (!wifi.is_connected_to_wifi && connection_scheduler.size() <= 1) {
            Event<Esp8266> try2connect(connection_scheduler.get_current_time() + AP_CONNECT_TIMEOUT, &wifi, &Esp8266::connect_to_wifi);
            connection_scheduler.push(try2connect);
        }

        if (wifi.is_connected_to_wifi && !wifi.is_connected_to_server && ticks % 2000 == 0) {
             connection_scheduler.invalidate(&wifi);
             wifi.time_synced = 0;
             connect_to_server(10, "192.168.1.113", "2252");
        }
        if (wifi.is_connected_to_wifi && wifi.is_connected_to_server) {
            connection_scheduler.invalidate(&wifi);

            wifi.sync_time();
            if (cache_handler.eventExist() && wifi.current_state == STATE_READY && ticks % SERVER_CONNECT_TIMEOUT == 10000) {

                //tag_event buf = cache_handler.popOldestEvent();
                //wifi.send_event(buf.tag_id, buf.node, buf.event_time, buf.access_result, buf.cache_status);
            }
            if (machine_state.get_state() == MACHINE_STATE_SERVER_CONNECTING)
                machine_state.set_state_idle();
             //wifi.refresh_status();

        }
        cache_handler.deleteEventById(10);
        machine_state.get_state();
        wifi.Delay(0);
        Delay(1000);
	}
}

void interrupt_initialize()
{
	__disable_irq();
	EXTI_InitTypeDef EXTI_InitStructure;
	// NVIC structure to set up NVIC controller
	NVIC_InitTypeDef NVIC_InitStructure;

	// GPIO structure used to initialize Button pins
	// Connect EXTI Lines to Button Pins
	// BTN_OPEN.
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);
    // BTN_CALL.
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource4);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource10);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource8);

	// IRQ Driven Button BTN_OPEN.
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0B;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0B;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // IRQ Driven Button BTN_CALL.
    EXTI_InitStructure.EXTI_Line = EXTI_Line1;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0A;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0A;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
//
//    EXTI_InitStructure.EXTI_Line = EXTI_Line4;
//    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
//    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//    EXTI_Init(&EXTI_InitStructure);
//
//    NVIC_InitStructure.NVIC_IRQChannel =  EXTI4_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x08;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x08;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);
    //----------------------------------------------
    EXTI_InitStructure.EXTI_Line = EXTI_Line8;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    // RC522 Timer And PICC Receive Interrupt.
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x09;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x09;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
//----------------------------------------------
//    EXTI_InitStructure.EXTI_Line = EXTI_Line10;
//    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
//    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//    EXTI_Init(&EXTI_InitStructure);
//
//    // RC522 Timer And PICC Receive Interrupt.
//    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x09;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x09;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);
//----------------------------------------------
	// TIM2 timer, used as on second watchdog for enc28j60, rc522.
	// Also do some periodical not priority calls.
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0f;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0f;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

    //USART handler timer
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x05;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x05;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

	// Systick timer, used for milliseconds granularity
	// and milliseconds set timeouts.
	NVIC_InitStructure.NVIC_IRQChannel = SysTick_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn; //канал
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01; //приоритет
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;//приоритет субгруппы
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //включаем канал
    NVIC_Init(&NVIC_InitStructure); //инициализируем
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

extern "C" void initialize_systick()
{
   RCC_ClocksTypeDef RCC_Clocks;
   RCC_GetClocksFreq(&RCC_Clocks);

   // 1 millisecond tick.
   SysTick_Config(RCC_Clocks.HCLK_Frequency / (1000));
}

/*******************************************************************************
 * Function Name  : RCC_Configuration
 * Description    : Configures the different system clocks.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void RCC_Configuration(void)
{
	/* RCC system reset(for debug purpose) */
	RCC_DeInit();

	/* Enable HSE */
	RCC_HSEConfig(RCC_HSE_ON);

	/* Wait till HSE is ready */
	HSEStartUpStatus = RCC_WaitForHSEStartUp();

	if(HSEStartUpStatus == SUCCESS)
	{
		/* Enable Prefetch Buffer */
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

		/* Flash 2 wait state */
		FLASH_SetLatency(FLASH_Latency_2);

		/* HCLK = SYSCLK */
		RCC_HCLKConfig(RCC_SYSCLK_Div1);

		/* PCLK2 = HCLK */
		RCC_PCLK2Config(RCC_HCLK_Div1);

		/* PCLK1 = HCLK/2 */
		RCC_PCLK1Config(RCC_HCLK_Div2);

		/* PLLCLK = 8MHz * 9 = 72 MHz */
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

		/* Enable PLL */
		RCC_PLLCmd(ENABLE);

		/* Wait till PLL is ready */
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) {}

		/* Select PLL as system clock source */
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

		/* Wait till PLL is used as system clock source */
		while(RCC_GetSYSCLKSource() != 0x08) {}
	}

	/* PCLK2 = HCLK/2 */
	RCC_PCLK2Config(RCC_HCLK_Div2);

	//enable AFIO clock
    	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	/* Enable GPIO clock for SPIz */
	RCC_APB2PeriphClockCmd(SPIz_GPIO_CLK | RCC_APB2Periph_AFIO, ENABLE);

	/* Enable SPIz Periph clock */
	RCC_APB1PeriphClockCmd(SPIz_CLK, ENABLE);
}

/*******************************************************************************
 * Function Name  : NVIC_Configuration
 * Description    : Configures Vector Table base location.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void NVIC_Configuration(void)
{
#ifdef  VECT_TAB_RAM
	/* Set the Vector Table base location at 0x20000000 */
	NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
#else  /* VECT_TAB_FLASH  */
	/* Set the Vector Table base location at 0x08000000 */
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
#endif
}


#ifdef  DEBUG
/*******************************************************************************
 * Function Name  : assert_failed
 * Description    : Reports the name of the source file and the source line number
 *                  where the assert_param error has occurred.
 * Input          : - file: pointer to the source file name
 *                  - line: assert_param error line source number
 * Output         : None
 * Return         : None
 *******************************************************************************/
void assert_failed(uint8_t *file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{
	}
}
#endif

/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/

