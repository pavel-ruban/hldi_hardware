/******************** (C) COPYRIGHT 2016 Pavel Ruban ********************
 * File Name          : c_entry.c
 * Author             : Pavel Ruban
 * Version            : V1.0
 * Date               : 12/06/2016
 * Description        : Main program body
 ********************************************************************************

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stm32f10x_conf.h>
#include <led.hpp>
#pragma pack(1)

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
GPIO_InitTypeDef GPIO_InitStructure;
TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
TIM_OCInitTypeDef TIM_OCInitStructure;
ErrorStatus HSEStartUpStatus;

/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);
void NVIC_Configuration(void);
void Delay(vu32 nCount);
extern "C" void custom_asm();
void RTC_Configuration();

extern "C" void reset_asm();

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
	while (1) {}
}


extern "C" void EXTI15_10_IRQHandler()
{
	while (1) {}
}



extern "C" void interrupt_initialize();

extern "C" void __initialize_hardware()
{
	/* Enable GPIOC clock */
	interrupt_initialize();
}

extern "C" void __reset_hardware()
{
	reset_asm();
}
extern "C" int main(void)
{
	//rmfrc522_init();

	// Check if timer started.
	//uint8_t status = mfrc522_read(Status1Reg);

	__enable_irq();

	//rc522_irq_prepare();

	test_f(0xDEADBE, 0xFA, 0x1, 0x2,0x3,0x4,0x5,0x6);
	led my_led(LED_TYPE_GREEN, GPIOA, GPIO_Pin_1, 0x7);

	while (1)
	{
		uint32_t clock = RTC_GetCounter();
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
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource10);

	// Select EXTI line0
	EXTI_InitStructure.EXTI_Line = EXTI_Line10;
	//select interrupt mode
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	//generate interrupt on rising edge
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	//send values to registers
	EXTI_Init(&EXTI_InitStructure);

	// Clear STM32 irq bit.
	EXTI_ClearITPendingBit(EXTI_Line10);

	// Configure NVIC
	// Select NVIC channel to configure
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	// Set priority to lowest
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x08;
	// Set subpriority to lowest
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x08;
	// Enable IRQ channel
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	// Update NVIC registers
	NVIC_Init(&NVIC_InitStructure);

	NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
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

/*******************************************************************************
 * Function Name  : Delay
 * Description    : Inserts a delay time.
 * Input          : nCount: specifies the delay time length.
 * Output         : None
 * Return         : None
 *******************************************************************************/
void Delay(vu32 nCount)
{
	for(; nCount != 0; nCount--);
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
void assert_failed(u8* file, u32 line)
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

