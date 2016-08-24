#ifndef LED_HPP
#define LED_HPP

// This class works with TIM2 and GPIOA pins PA1 PA2 PA3
#pragma pack(1)

#define TIM_PRESCALER 720
#define TIM_PERIOD 100

#define LED_TYPE_RED 2
#define LED_TYPE_GREEN 1
#define LED_TYPE_BLUE 0
#define LED_TYPE_RGB 3

class led
{
	uint32_t on_interval;
	uint32_t off_interval;
	uint32_t color;
	bool blink;
private:
	uint8_t type;
	uint16_t pin_r;
	uint16_t pin_g;
	uint16_t pin_b;

	void _init_output_channel(uint32_t intensity, uint16_t led_pin);
public:
	led(uint8_t led_type, GPIO_TypeDef *led_port, uint16_t led_pin, uint8_t intensity);
//	led(uint8_t led_type, GPIO_TypeDef *led_port, uint16_t led_pin_r, uint16_t led_pin_g, uint16_t led_pin_b, uint32_t color = 0xFFFFFF);
	void on();
	void off();
	void set_color(uint32_t color);
};
void test_f(uint32_t tp32,uint8_t tp8, uint32_t tp32_1, uint32_t tp32_2, uint32_t tp32_3, uint32_t tp32_4, uint32_t tp32_5, uint32_t tp32_6);
#endif
