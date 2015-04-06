#include <inttypes.h>

//                                            1    2   3    4    5    6    7    8    9    10   11   12
static const uint8_t month_len_lut_bcd[] = {0x31,0x00,0x31,0x30,0x31,0x30,0x31,0x31,0x30,0x31,0x30,0x31};



static uint8_t get_month_len_bcd(uint8_t month, uint8_t yr)
{
	//convert bcd month into dec
	if (month > 9)
		month = month - 6;


	if (month > 12)
		return 0;
	if (month != 2)
		return month_len_lut_bcd[month-1];
	if ((yr & 0x3) == 0)
		return 0x29;
	else
		return 0x28;
}

uint32_t bcd_time_inc(uint32_t in, uint32_t in2, uint8_t overflow)
{
	in = in + in2;
	//sec
	if ((in & 0xF) > 9){
		in = in - 0xA;
		in = in + 0x10;
	}
	if ((in & 0xF0) > 0x50){
		in = in - 0x60;
		if (overflow)
			in = in + 0x100;
	}

	//min
	if ((in & 0xF00) > 0x900){
		in = in - 0xA00;
		in = in + 0x1000;
	}
	if ((in & 0xF000) > 0x5000){
		in = in - 0x6000;
		if (overflow)
			in = in + 0x10000;
	}

	//hour
	if ((in & 0xF0000) > 0x90000){
		in = in - 0xA0000;
		in = in + 0x100000;
	}

	if ((in & 0xFF0000) > 0x230000){
		in = in - 0x240000;
	}
	return in;
}

uint32_t bcd_date_inc(uint32_t in, uint32_t in2, uint8_t overflow)
{
	in = in + (in2&0xFF0000);

	//day
	if ((in & 0xF0000) > 0x90000){
		in = in - 0xA0000;
		in = in + 0x100000;
	}
	uint32_t mon_len = get_month_len_bcd((in & 0xFF00) >> 8, in&0xFF)<<16;
	if ((in & 0xFF0000) > mon_len){
		in = in - mon_len;
		if (overflow)
			in = in + 0x100;
	}

	in = in + (in2&0xFF00);
	//mon
	if ((in & 0xF00) > 0x900){
		in = in - 0xA00;
		in = in + 0x1000;
	}
	if ((in & 0xFF00) > 0x1200){
		in = in - 0x1200;
		if (overflow)
			in = in + 0x1;
	}
	//check the day is still valid
	mon_len = get_month_len_bcd((in & 0xFF00) >> 8, in&0xFF)<<16;
	if ((in & 0xFF0000) > mon_len){
		in = (in & 0xFFFF) + mon_len;
	}

	in = in + (in2&0xFF);
	//year
	if ((in & 0xF) > 9){
		in = in - 0xA;
		in = in + 0x10;
	}
	if ((in & 0xF0) > 0x90){
		in = in - 0x90;
	}
	return in;
}
