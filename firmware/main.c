#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/cm3/systick.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pindef.h"
#include "tlc59711.h"
#include "ds3234.h"
#include "bcd_maths.h"

void init(void);
void _delay_ms(const uint32_t delay);
void reinit_spi_dma(uint8_t *mem_addr, uint16_t bytes);
void increment_digit(void);
void reset_digits(void);
void wait_button_release(void);
void update_leds(void);
uint8_t wait_button_timeout(void);

//const uint8_t buff_test[] = {0xFF, 8, 8, 0, 0xFF, 8, 0, 8, 0xFF, 8};

uint8_t buff_led[56];

uint32_t led_values[] = {0x00FFFF, 0x00FFFF, 0x00FFFF, 0x00FFFF, 0x00FFFF, 0x00FFFF};

static volatile uint16_t led_h = 0;
static volatile uint16_t led_s = 180;
static volatile uint8_t led_s_dir = 0;
static volatile uint16_t led_s_prescale = 0;
static volatile uint8_t led_change_count = 0;
static volatile uint8_t led_pause = 0;
#define LED_FADE_MODE_MAX 5
static volatile uint8_t led_fade_mode = 0;

uint8_t led_mask = 0x3F;


volatile uint8_t digit_pos = 0;


volatile uint32_t digits_time;
volatile uint32_t digits_aux;
volatile uint32_t digits_date;
volatile uint32_t digits_set_time;
volatile uint32_t digits_set_date;
volatile uint8_t digits_colons = 0x8;

volatile uint16_t date_countdown = 0;
volatile uint32_t delay_countdown = 0;
volatile uint16_t colour_countdown = 0;

const uint8_t gb_r = 0x5;
const uint8_t gb_g = 0x2;
const uint8_t gb_b = 0x5;


typedef enum {TIME, TIME_AUX, DATE, SET_TIME, SET_DATE} ui_main_t;
ui_main_t ui_main = TIME;
uint8_t set_digit_pair = 0;


int main(void)
{
	init();



	//GPIO_BRR(HV_PORT) = HV_PIN;	//Turn on HV
	GPIOB_BSRR = (1<<5)<<16;
	//gpio_clear(HV_PORT,HV_PIN);	//Turn on HV
//	GPIO_BSRR(CC1_PORT) = CC1_PIN;
//	GPIO_BSRR(CC2_PORT) = CC2_PIN;


	update_leds();
    if(ds3234_check_valid_time() == 0)
    	ds3234_write_time_bcd(0x100110);
    if(ds3234_check_valid_date() == 0)
		ds3234_write_date_bcd(0x010415);


	digits_time = ds3234_read_time_bcd();
	digits_date = ds3234_read_date_bcd();

    uint8_t fast_inc = 0;
    while(1)
	{

    	switch(ui_main){
    		case TIME_AUX:
    		case TIME:
    			if (gpio_get(S3_PORT,S3_PIN)){         //DATE/TIME button
    				ui_main = DATE;
    				date_countdown = 5000;
    			}
				else if (gpio_get(S2_PORT,S2_PIN)) {  //SET button
					ui_main = SET_TIME;
					set_digit_pair = 0;
					digits_set_time = digits_time;
					led_mask = 0x30;
					update_leds();
					wait_button_release();
				}
				else if (gpio_get(S1_PORT,S1_PIN)) {  //pause button
					led_pause = 1-led_pause;
					wait_button_release();
				}else if (gpio_get(S4_PORT,S4_PIN)) //colour button
		    	{
		    		led_fade_mode++;
		    		if (led_fade_mode >= LED_FADE_MODE_MAX)
		    			led_fade_mode = 0;
		    		led_pause = 0;
		    		wait_button_release();
		    		colour_countdown = 1000;
		    		ui_main = TIME_AUX;
		    		digits_aux = (0xFF<<16) | ((led_fade_mode+1) << 8) | (0xFF);
		    	}
    			break;
    		case DATE:
    			if (gpio_get(S2_PORT,S2_PIN)) {  //SET button
					ui_main = SET_DATE;
					set_digit_pair = 0;
					digits_set_date = digits_date;
					led_mask = 0x30;
					update_leds();
					wait_button_release();
				}
    			break;
    		case SET_TIME:
    			if (gpio_get(S2_PORT,S2_PIN)) {  //SET button
					set_digit_pair++;
					led_mask >>= 2;
					if (set_digit_pair == 3){
						ui_main = TIME;
						ds3234_write_time_bcd(digits_set_time);
						digits_time = digits_set_time;
						led_mask = 0x3F;
					}
					update_leds();
					wait_button_release();
				}else if(gpio_get(S1_PORT,S1_PIN)) //INC
				{
					digits_set_time = bcd_time_inc(digits_set_time,0x10000>>(set_digit_pair*8),0);
					if (fast_inc == 0)
						fast_inc = wait_button_timeout();
					else
						_delay_ms(100);
				}
				else
					fast_inc = 0;

    			break;

    		case SET_DATE:
    			if (gpio_get(S2_PORT,S2_PIN)) {  //SET button
					set_digit_pair++;
					led_mask >>= 2;
					if (set_digit_pair == 3){
						ui_main = DATE;
						date_countdown = 5000;
						ds3234_write_date_bcd(digits_set_date);
						digits_date = digits_set_date;
						led_mask = 0x3F;
					}
					update_leds();
					wait_button_release();
				}else if(gpio_get(S1_PORT,S1_PIN)) //INC
				{
					digits_set_date = bcd_date_inc(digits_set_date,0x10000>>(set_digit_pair*8),0);
					if (fast_inc == 0)
						fast_inc = wait_button_timeout();
					else
						_delay_ms(100);
				}
				else
					fast_inc = 0;
    			break;
    		default:
    			break;
    	}

	}
}

uint8_t wait_button_timeout(void)
//returns 1 if timeout has occured
{
	uint8_t c=0;
	uint8_t t = 200;
	while((c < 3) && (t > 0))
	{
		if ((gpio_get(S3_PORT,S3_PIN)>0) || (gpio_get(S2_PORT,S2_PIN)>0) || (gpio_get(S1_PORT,S1_PIN)>0))
			c = 0;
		else
			c++;
		_delay_ms(5);
		t--;
	}
	if (t == 0)
		return 1;
	else
		return 0;
}

void wait_button_release(void)
{
	uint8_t c=0;
	while(c < 3)
	{
		if ((gpio_get(S3_PORT,S3_PIN)>0) || (gpio_get(S2_PORT,S2_PIN)>0) || (gpio_get(S4_PORT,S4_PIN)>0) || (gpio_get(S1_PORT,S1_PIN)>0))
			c = 0;
		else
			c++;
		_delay_ms(5);
	}
}

void update_leds(void)
{
	format_packet_nixie(buff_led, gb_r, gb_g, gb_b, led_values, led_mask);
	reinit_spi_dma(buff_led, 56);
	spi_enable_tx_dma(LED_SPI);
}

void sys_tick_handler(void)
{
	increment_digit();

	if (date_countdown){
		date_countdown--;
		if ((date_countdown == 0) &&(ui_main == DATE))
			ui_main = TIME;
	}
	if (colour_countdown){
		colour_countdown--;
		if ((colour_countdown == 0) &&(ui_main = TIME_AUX))
			ui_main = TIME;
	}

	if (delay_countdown)
		delay_countdown--;


	if (led_change_count == 0)
	{
		led_change_count = 100;

		uint8_t i;
		uint16_t v,led_s_lim;
		uint32_t v1;

		led_h += 1;
		if (led_h >= 360)
			led_h = led_h-360;
		switch(led_fade_mode)
		{
			case 0:                 //sweeping gradient
				v = led_h;
				for (i = 0; i < 6; i++){
					led_values[i] = hsv_to_rgb((uint16_t)v,255,255);
					v+= 14;
					if (v >= 360)
						v -= 360;
				}
				break;
			case 2:               //continuous gradient
				//led_s_prescale++;
				//if (led_s_prescale > 3){
					led_s_prescale = 0;
					if (led_s_dir)
						led_s+=3;
					else
						led_s-=3;

				//}
				if (led_s < 100){
					led_s = 100;
					led_s_dir = 100;
				}
				if (led_s > 400){
					led_s = 400;
					led_s_dir = 0;
				}

				led_s_lim = led_s;
				if (led_s > 255)
					led_s_lim = 255;
				if (led_s < 160)
					led_s_lim = 160;


				//if (led_h == 61){
				//	if (led_s < 200)
				//		led_s = 220;
				//	else
				//		led_s = 180;
				//}
			case 1:
				if (led_fade_mode == 1)
					v1 = hsv_to_rgb(led_h,255,255);
				else
					v1 = hsv_to_rgb(led_h,led_s_lim,255);
				for (i = 0; i < 6; i++)
					led_values[i] = v1;
				break;
			case 3:
				v = led_h;
				for (i = 0; i < 6; i++){
					led_values[i] = hsv_to_rgb((uint16_t)v,255,255);
					v+= 60;
					if (v >= 360)
						v -= 360;
				}
				break;
			case 4:
				v = led_h;
				for (i = 0; i < 6; i++){
					led_values[i] = hsv_to_rgb((uint16_t)v,255,255);
					v+= 180;
					if (v >= 360)
						v -= 300;
				}
				break;
		}

		//led_values[5] = hsv_to_rgb(led_h,255,255);
		update_leds();

	}
	if (led_pause == 0)
		led_change_count--;
}

void exti4_15_isr(void)
{
	if ((EXTI_PR & (GPIO8)) != 0)
	{
		digits_time = bcd_time_inc(digits_time,1,1);


		//digits_time = ds3234_read_time_bcd();

		if (digits_time == 0x000000)
			digits_date = ds3234_read_date_bcd();

		if ((EXTI_PR & (GPIO8)) != 0)
			EXTI_PR |= (GPIO8);
	}
}


void increment_digit(void)
{

	GPIO_BSRR(NCLK_PORT) = NCLK_PIN<<16;
	GPIO_BSRR(NDAT_PORT) = NDAT_PIN<<16;
	digit_pos++;
	GPIO_ODR(A_PORT) = (GPIO_ODR(A_PORT) & ~ABCD_MASK);
	gpio_clear(CC1_PORT,CC1_PIN | CC2_PIN);

	if (digit_pos >= 6)
	{
		digit_pos = 0;
		GPIO_BSRR(NDAT_PORT) = NDAT_PIN;
		__asm__("nop");
		__asm__("nop");
	}


	GPIO_BSRR(NCLK_PORT) = NCLK_PIN;
	__asm__("nop");
	GPIO_BSRR(NDAT_PORT) = NDAT_PIN<<16;


	uint32_t digits_bcd;
	digits_colons = 8;
	switch(ui_main){
		case TIME_AUX:
			digits_bcd = digits_aux;
			digits_colons = 0;
			break;
		case TIME:
			digits_bcd = digits_time;
			break;
		case DATE:
			digits_bcd = digits_date;
			break;
		case SET_TIME:
			digits_bcd = digits_set_time;
			break;
		case SET_DATE:
			digits_bcd = digits_set_date;
			break;
		default:
			digits_bcd = 0xFFFFFF;
			break;
	}

	uint32_t digit = (digits_bcd >> (4*(5-digit_pos))) & 0xF;
	digit = (9-digit) << 6;

/*	*/if (digit_pos == 1){
		GPIO_BSRR(CC1_PORT) = (CC1_PIN | CC2_PIN) << 16;
		if (digits_colons & 0x8)
			GPIO_BSRR(CC1_PORT) = CC1_PIN;
		if (digits_colons & 0x4)
			GPIO_BSRR(CC2_PORT) = CC2_PIN;
	}else if (digit_pos == 3){
		GPIO_BSRR(CC1_PORT) = (CC1_PIN | CC2_PIN) << 16;
		if (digits_colons & 0x2)
			GPIO_BSRR(CC1_PORT) = CC1_PIN;
		if (digits_colons & 0x1)
			GPIO_BSRR(CC2_PORT) = CC2_PIN;
	}
	else
		GPIO_BSRR(CC1_PORT) = (CC1_PIN | CC2_PIN) << 16;
	//	GPIO_ODR(B_PORT) = (GPIO_ODR(B_PORT) & ~EF_MASK) | ((digits_colons&0xC) << 10);
	//else if (digit_pos == 3)
	//	GPIO_ODR(B_PORT) = (GPIO_ODR(B_PORT) & ~EF_MASK) | ((digits_colons&0x3) << 12);
	////else
	////	GPIO_ODR(B_PORT) = (GPIO_ODR(B_PORT) & ~EF_MASK);

	GPIO_ODR(A_PORT) = (GPIO_ODR(A_PORT) & ~ABCD_MASK) | digit;


}

void reset_digits(void)
{
	GPIO_BSRR(NDAT_PORT) = NDAT_PIN<<16;
	GPIO_BSRR(NCLK_PORT) = NCLK_PIN<<16;
	uint8_t i;
	for(i = 0; i < 8; i++)
	{
		GPIO_BSRR(NCLK_PORT) = NCLK_PIN;
		__asm__("nop");
		__asm__("nop");
		GPIO_BSRR(NCLK_PORT) = NCLK_PIN<<16;
		__asm__("nop");
		__asm__("nop");
	}
}

void init(void)
{

	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);

#if defined(STM32F0)
	rcc_periph_clock_enable(RCC_DMA);
	//GPIOB outputs
	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, CC1_PIN | CC2_PIN
			| NCLK_PIN | NDAT_PIN | HV_PIN | R_MO_PIN | R_CLK_PIN | BUZZ_PIN
			| A_PIN | B_PIN | C_PIN | D_PIN);
	GPIO_BSRR(HV_PORT) = HV_PIN;	//Turn off HV

	//GPIOA inputs
	gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, S1_PIN | S2_PIN
			| R_MI_PIN | RX_PIN);
	gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, IR_PIN  | PPS_PIN);

	//GPIOA outputs
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, RTCCS_PIN);
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, TX_PIN | MOSI_PIN
			| SCLK_PIN);
	gpio_set_af(GPIOA, GPIO_AF0, MOSI_PIN | SCLK_PIN);

	//GPIOB inputs
	gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, S3_PIN
			| S4_PIN);

	//SPI port (LEDs)
	rcc_periph_clock_enable(RCC_SPI1);

	spi_reset(LED_SPI);
	spi_init_master(LED_SPI, SPI_CR1_BAUDRATE_FPCLK_DIV_64,
			SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
			SPI_CR1_CPHA_CLK_TRANSITION_1,
			SPI_CR1_CRCL_8BIT,
			SPI_CR1_MSBFIRST);
	// NSS must be set high for the peripheral to function
	spi_enable_software_slave_management(LED_SPI);
	spi_set_nss_high(LED_SPI);
	//spi_enable_tx_dma(LED_SPI);
	spi_enable(LED_SPI);


	//enable IRQ on square wave input
	//enable interrupts (just on one input)
	RCC_APB2ENR |= (1<<0);
	exti_select_source(EXTI8, GPIOA);
	exti_set_trigger(EXTI8, EXTI_TRIGGER_RISING);
	exti_enable_request(EXTI8);
	nvic_enable_irq(NVIC_EXTI4_15_IRQ);

	rcc_clock_setup_in_hsi_out_8mhz();

	//systick
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
	systick_set_reload(7999);
	systick_interrupt_enable();
	systick_counter_enable();





#elif defined(STM32F1)
	rcc_periph_clock_enable(RCC_DMA1);
	//free up PB4
	rcc_periph_clock_enable(RCC_AFIO);
	GPIO_ODR(BUZZ_PORT) = 0;
	AFIO_MAPR |= AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON;

	//GPIOB outputs
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
			CC1_PIN | CC2_PIN | NCLK_PIN | NDAT_PIN | HV_PIN | R_MO_PIN
			| R_CLK_PIN | BUZZ_PIN);
	GPIO_BSRR(HV_PORT) = HV_PIN;	//Turn off HV
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
			A_PIN | B_PIN | C_PIN | D_PIN);


	//GPIOA inputs
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, IR_PIN
			| S1_PIN | S2_PIN | R_MI_PIN | RX_PIN);
	GPIO_ODR(IR_PORT) |= IR_PIN;  //enable pullup

	//GPIOA outputs
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, RTCCS_PIN);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
			TX_PIN | MOSI_PIN);// | SCLK_PIN);

	//GPIOB inputs
	gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN,
			S3_PIN | S4_PIN);

	//SPI port (LEDs)
	rcc_periph_clock_enable(RCC_SPI1);
	spi_reset(LED_SPI);
	spi_init_master(LED_SPI, SPI_CR1_BAUDRATE_FPCLK_DIV_64,
			SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
			SPI_CR1_CPHA_CLK_TRANSITION_1,
			SPI_CR1_DFF_8BIT,
			SPI_CR1_MSBFIRST);
	// NSS must be set high for the peripheral to function
	spi_enable_software_slave_management(LED_SPI);
	spi_set_nss_high(LED_SPI);
	spi_enable(LED_SPI);
#else
#       error "stm32 family not defined."
#endif

	reset_digits();
	ds3234_enable_1hz_out();

}

void reinit_spi_dma(uint8_t *mem_addr, uint16_t bytes)
{
	//SPI DMA setup
	dma_channel_reset(DMA1,3);
//	nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ);
	dma_set_peripheral_address(DMA1,3,(uint32_t)&SPI_DR(LED_SPI));
	dma_set_memory_address(DMA1,3,(uint32_t)mem_addr);
	dma_set_number_of_data(DMA1,3,bytes);
//	dma_disable_circular_mode(DMA1,3);
	dma_set_read_from_memory(DMA1,3);
	dma_enable_memory_increment_mode(DMA1,3);
	dma_set_peripheral_size(DMA1,3,DMA_CCR_PSIZE_8BIT);
	dma_set_memory_size(DMA1,3,DMA_CCR_MSIZE_8BIT);
	dma_set_priority(DMA1,3,DMA_CCR_PL_HIGH);
//	dma_enable_half_transfer_interrupt(DMA1,3);
//	dma_enable_transfer_complete_interrupt(DMA1,3);
	dma_enable_channel(DMA1,3);
}

void _delay_ms(const uint32_t delay)
{
	delay_countdown = delay;
	while(delay_countdown);
}
