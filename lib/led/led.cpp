
class rgb
{
	uint32_t time_units;
	uint32_t on_interval;
	uint32_t off_interval;
	uint32_t color;
	bool blink;

	rgb(GPIO_TypeDef *led_port, uint16_t led_pin)
	{
		// Need to enable clock on APB2 before use
		GPIO_InitTypeDef led_init;
		GPIO_InitStructure.GPIO_Pin = led_pin;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(led_port, &led_init);
		blink = 0;
	}

	void on()
	{
	}

	void off()
	{
	}



}


