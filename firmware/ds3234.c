#include <pindef.h>
#include "ds3234.h"

static void clock_out(uint8_t value);
static uint8_t clock_in(void);

void init_ds3234(void)
{


}

void ds3234_enable_1hz_out(void)
{
	ds3234_write(0xe,0);
}

uint8_t ds3234_check_valid_time(void)
{
	uint32_t in = ds3234_read_time_bcd();
	//sec
	if ((in & 0xF) > 9){
		return 0;
	}
	if ((in & 0xF0) > 0x50){
		return 0;
	}

	//min
	if ((in & 0xF00) > 0x900){
		return 0;
	}
	if ((in & 0xF000) > 0x5000){
		return 0;
	}

	//hour
	if ((in & 0xF0000) > 0x90000){
		return 0;
	}

	if ((in & 0xFF0000) > 0x230000){
		return 0;
	}

	return 1;

}

uint8_t ds3234_check_valid_date(void)
{
	uint32_t in = ds3234_read_date_bcd();
	//yr
	if ((in & 0xF) > 9){
		return 0;
	}
	if ((in & 0xF0) > 0x90){
		return 0;
	}

	//mon
	if ((in & 0xF00) > 0x900){
		return 0;
	}
	if ((in & 0xFF00) > 0x1200){
		return 0;
	}

	//day
	if ((in & 0xF0000) > 0x90000){
		return 0;
	}

	if ((in & 0xFF0000) > 0x310000){
		return 0;
	}

	return 1;

}

uint32_t ds3234_read_time_bcd(void)
{
	uint8_t t = ds3234_read(2);
	ds3234_write(2,t&0xBF);

	uint32_t out = 0;
	out = ds3234_read(0);
	out |= ds3234_read(1)<<8;
	out |= (ds3234_read(2)&0x3F)<<16;
	return out;
}
uint32_t ds3234_read_date_bcd(void)
{
	uint32_t out = 0;
	out = ds3234_read(6);
	out |= (ds3234_read(5)&0x1F)<<8;
	out |= (ds3234_read(4)&0x3F)<<16;
	return out;
}

void ds3234_write_time_bcd(uint32_t t)
{
	uint8_t w;
	w = (t & 0xFF0000) >> 16;
	ds3234_write(2,w&0x3F);
	w = (t & 0xFF00) >> 8;
	ds3234_write(1,w);

	w = (t & 0xFF);
	ds3234_write(0,w);
}
void ds3234_write_date_bcd(uint32_t t)
{
	uint8_t w;
	w = (t & 0xFF0000) >> 16;
	ds3234_write(4,w&0x3F);
	w = (t & 0xFF00) >> 8;
	ds3234_write(5,w&0x1F);
	w = (t & 0xFF);
	ds3234_write(6,w);
}

uint8_t ds3234_read(uint8_t addr)
{
	uint8_t out;
	gpio_clear(RTCCS_PORT, RTCCS_PIN);
	clock_out(addr & 0x7F);
	out = clock_in();
	gpio_set(RTCCS_PORT, RTCCS_PIN);
	return out;
}

void ds3234_write(uint8_t addr, uint8_t data)
{
	gpio_clear(RTCCS_PORT, RTCCS_PIN);
	clock_out(addr | 0x80);
	clock_out(data);
	gpio_set(RTCCS_PORT, RTCCS_PIN);
}

static void clock_out(uint8_t value)
{
	uint8_t i;
	for (i = 0; i < 8; i++)
	{
		if (value & 0x80)
			gpio_set(R_MO_PORT,R_MO_PIN);
		else
			gpio_clear(R_MO_PORT,R_MO_PIN);

		gpio_set(R_CLK_PORT, R_CLK_PIN);
		value = value << 1;
		gpio_clear(R_CLK_PORT, R_CLK_PIN);
	}
}

static uint8_t clock_in(void)
{
	uint8_t i,out;
	out = 0;
	for (i = 0; i < 8; i++)
	{
		gpio_set(R_CLK_PORT, R_CLK_PIN);
		out = out << 1;
		if (gpio_port_read(R_MI_PORT)&R_MI_PIN)
			out |= 0x1;
		gpio_clear(R_CLK_PORT, R_CLK_PIN);
	}
	return out;
}
