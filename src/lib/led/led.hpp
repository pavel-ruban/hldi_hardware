#ifndef LED_HPP
#define LED_HPP

// This class works with TIM2 and GPIOA pins PA1 PA2 PA3
#pragma pack(1)

#define TIM_PRESCALER 700
#define TIM_PERIOD 100

#define LED_TYPE_RED 2
#define LED_TYPE_GREEN 1
#define LED_TYPE_BLUE 0
#define LED_TYPE_RGB 3

class Color{
public:
    uint8_t Red = 0;
    uint8_t Green = 0;
    uint8_t Blue = 0;
    uint8_t Uncolored = 0;
    Color(){

    }
    Color(uint8_t intencity){
        Uncolored = intencity;
    }
    Color(uint8_t red, uint8_t green, uint8_t blue){
        Red = red;
        Green = green;
        Blue = blue;
    }
    Color(uint32_t color){
        Red = (color >> 16) & 0xFF;
        Green = (color >> 8) & 0xFF;
        Blue = (color >> 0) & 0xFF;
    }
};

class led
{
	uint32_t on_interval;
	uint32_t off_interval;
    Color color;
	//uint32_t color;
	uint8_t s_led_intensity;
	bool blink;
private:
	uint8_t type;
	uint16_t pin_r;
	uint16_t pin_g;
	uint16_t pin_b;
	uint16_t pin_s;

	void _init_output_channel(uint32_t intensity, uint16_t led_pin);
public:
	led(uint8_t led_type, GPIO_TypeDef *led_port, uint16_t led_pin, uint8_t intensity = 0x2F);
	led(uint8_t led_type, GPIO_TypeDef *led_port, uint16_t led_pin_r, uint16_t led_pin_g, uint16_t led_pin_b, uint32_t color = 0xFFFFFF);
	void on();
	void off();
    void set_color(Color color);
	void set_color(uint32_t color);
    void set_intensity(uint8_t);
};
//void test_f(uint32_t tp32,uint8_t tp8, uint32_t tp32_1, uint32_t tp32_2, uint32_t tp32_3, uint32_t tp32_4, uint32_t tp32_5, uint32_t tp32_6);
#endif
