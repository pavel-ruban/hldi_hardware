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
#include "esp8266/esp8266.h"
#include <stdio.h>


#include "include/queue.h"
#include "lib/array/array.h"

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
uint8_t machine_status;

typedef struct {
	uint8_t tag_id[4];
	uint8_t flags;
	uint32_t cached;
} tag_cache_entry;

typedef struct {
	uint8_t tag_id[4];
	uint32_t event_time;
	uint32_t node;
} tag_event;

Queue<tag_cache_entry, 100> tag_cache;
Queue<tag_event, 100> tag_events;
Scheduler<Event<led>, 100> led_scheduler;
Scheduler<Event<Machine_state>, 100> state_scheduler;
Uart uart(UART1, 115200);
Esp8266 wifi(&uart);

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

extern "C" void SysTick_Handler(void)
{
    led_scheduler.handle();
    state_scheduler.handle();
	ticks++;
}

extern "C" void EXTI2_IRQHandler()
{

}


void Responce_Handler() {
    if (!uart.last_string_ready)
        return;
}

extern "C" void USART1_IRQHandler()
{
    // Receive Data register not empty interrupt.
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        uart.last_byte = USART_ReceiveData(USART1);
        GPIO_SetBits(EM_LOCK_PORT, EM_LOCK_PIN);

        if (!uart.last_string_ready) {
            uart.last_string[uart.last_char] = uart.last_byte;
            uart.last_char++;
        } else {
            for (uint16_t i = 0; i < RECV_STRING_MAX_SIZE; ++i) {
                uart.last_string[i] = 0;
            }
            uart.last_string_ready = 0;
            uart.last_string[0] = uart.last_byte;
            uart.last_char = 1;
        }

        if (uart.last_byte == '\n') {
            uart.last_string_ready = 1;
            wifi.handle_response();
        }
    }
    // Transmission complete interrupt.
    GPIO_ResetBits(EM_LOCK_PORT, EM_LOCK_PIN);

    if (USART_GetITStatus(USART1, USART_IT_TC) != RESET)
    {
        USART_ClearITPendingBit(USART1, USART_IT_TC);
    }
}

extern "C" void EXTI0_IRQHandler()
{
    if (!(machine_state.get_state() == MACHINE_STATE_LOCK_OPEN)) {
        machine_state.set_state_lock_open(MACHINE_STATE_LOCK_OPEN_TIME);
    }
	EXTI_ClearITPendingBit(EXTI_Line0);

}

extern "C" void EXTI1_IRQHandler()
{
    if (!(machine_state.get_state() == MACHINE_STATE_GUEST_CALL)) {
        machine_state.set_state_guest_call(MACHINE_STATE_GUEST_CALL_TIME);
    }
    EXTI_ClearITPendingBit(EXTI_Line1);
}


extern "C" void EXTI15_10_IRQHandler()
{
    for (;;)
    {}
}

extern "C" void EXTI9_5_IRQHandler()
{
    for (;;)
    {}
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

void InitializeTimer()
{
    RCC_ClocksTypeDef RCC_Clocks;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //Подвесили таймер 3 на ABP1 шину
    RCC_GetClocksFreq(&RCC_Clocks);
    TIM_TimeBaseInitTypeDef timerInitStructure;
    timerInitStructure.TIM_Prescaler = RCC_Clocks.HCLK_Frequency / 10000;
    timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    timerInitStructure.TIM_Period = 10000;
    timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    timerInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &timerInitStructure);
    TIM_Cmd(TIM3, ENABLE);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
}

int *xx;
extern "C" int
main(void)
{
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


    led rgb_led(LED_TYPE_RGB, GPIOA, GPIO_Pin_1, GPIO_Pin_2, GPIO_Pin_3, LED_COLOR_WHITE);
    leds[LED_STATE_INDICATOR] =  &rgb_led;
    leds[LED_STATE_INDICATOR]->on();
    //machine_state.set_state_idle();
    interrupt_initialize();
	//__enable_irq();
    InitializeTimer();
    GPIO_ResetBits(EM_LOCK_PORT, EM_LOCK_PIN);
    //SPI_InitTypeDef * biba;
    //uint8_t pcd = RC522_PCD_1;
    //rc522_pcd_select(pcd);
    //mfrc522_init();

    rc522_pcd_select(RC522_PCD_1);
    mfrc522_init();

    __enable_irq();

    rc522_irq_prepare();
//    rc522_irq_prepare();
  //  Delay(7000000);
  //  wifi.connect_to_wifi("i20.pub", "i20biz2015");
    //int_to_string(4235353);
    int i1 = 10;
    while (1)
	{

//        if (wifi.is_connected_to_wifi && !wifi.is_connected_to_server) {
//            wifi.connect_to_ip("192.168.1.141", "332");
//           // Delay(10000);
//
//        }
//        if (wifi.is_connected_to_server) {
//            i1 = strlen("BIBA\n");
//
//            wifi.send_request("BIBA\n");
//        }
        Delay(700000);
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

	// IRQ Driven Button BTN_OPEN.
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // IRQ Driven Button BTN_CALL.
    EXTI_InitStructure.EXTI_Line = EXTI_Line1;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x04;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x04;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    EXTI_InitStructure.EXTI_Line = EXTI_Line4;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel =  EXTI4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x08;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x08;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
//----------------------------------------------
    EXTI_InitStructure.EXTI_Line = EXTI_Line10;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    // RC522 Timer And PICC Receive Interrupt.
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x09;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x09;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
//----------------------------------------------
	// TIM2 timer, used as on second watchdog for enc28j60, rc522.
	// Also do some periodical not priority calls.
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0f;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0f;
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
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x05; //приоритет
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x05;//приоритет субгруппы
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

