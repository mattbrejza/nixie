#include <inttypes.h>
#include <string.h>

//this function is only 8 bit, and reorders for the leds on the board.
//buffer needs to fit packets for both packets
void format_packet_nixie(uint8_t *buff, uint8_t bc_blue, uint8_t bc_green, uint8_t bc_red, uint32_t *led_values, uint8_t mask)
{
	memset((void *)buff,0,28*2*sizeof(uint8_t));
	uint8_t i,a,m;
	buff[1] = 0;
	buff[0] = 0x25 << 2;	//write command
	buff[0] |= 2;           //OUTTMG
//	buff[0] |= 1;           //EXTCLK
	buff[1] |= (1<<7);      //TMGRST
	buff[1] |= (1<<6);      //DSPRPT
//	buff[1] |= (1<<5);      //BLANK

	buff[1+28] = 0;
	buff[0+28] = 0x25 << 2;	//write command
	buff[0+28] |= 2;           //OUTTMG
//	buff[0+28] |= 1;           //EXTCLK
	buff[1+28] |= (1<<7);      //TMGRST
	buff[1+28] |= (1<<6);      //DSPRPT
//	buff[1+28] |= (1<<5);      //BLANK

	buff[1] |= (bc_blue & 0x7C) >> 2;
	buff[2] = (bc_blue & 0x3) << 6;
	buff[2] |= (bc_green & 0x7E) >> 1;
	buff[3] = (bc_green & 0x1) << 7;
	buff[3] |= bc_red & 0x7F;

	buff[1+28] |= (bc_blue & 0x7C) >> 2;
	buff[2+28] = (bc_blue & 0x3) << 6;
	buff[2+28] |= (bc_green & 0x7E) >> 1;
	buff[3+28] = (bc_green & 0x1) << 7;
	buff[3+28] |= bc_red & 0x7F;

	const uint8_t buff_mappings[] = {10,22,16,28+10,28+4,28+22};

	i = 0;
	m = 0x20;
	for (i = 0; i < 6; i++)
	{
		if (mask & m){
			a = buff_mappings[i];
			buff[a] = led_values[i] & 0xFF;
			a+=2;
			buff[a] = (led_values[i]>>8) & 0xFF;
			a+=2;
			buff[a] = (led_values[i]>>16) & 0xFF;
		}
		m >>= 1;

	}
}

void format_packet(uint8_t *buff, uint8_t bc_blue, uint8_t bc_green, uint8_t bc_red, uint16_t *blues , uint16_t *greens , uint16_t *reds)
{
	uint8_t i,j;
	buff[1] = 0;
	buff[0] = 0x25 << 2;	//write command

	buff[0] |= 2;           //OUTTMG
//	buff[0] |= 1;           //EXTCLK
	buff[1] |= (1<<7);      //TMGRST
	buff[1] |= (1<<6);      //DSPRPT
//	buff[1] |= (1<<5);      //BLANK


	buff[1] |= (bc_blue & 0x7C) >> 2;
	buff[2] = (bc_blue & 0x3) << 6;
	buff[2] |= (bc_green & 0x7E) >> 1;
	buff[3] = (bc_green & 0x1) << 7;
	buff[3] |= bc_red & 0x7F;

	j = 4;
	for (i = 0; i < 4; i++)
	{
		buff[j] = blues[i] >> 8;
		j++;
		buff[j] = blues[i] & 0xFF;
		j++;

		buff[j] = greens[i] >> 8;
		j++;
		buff[j] = greens[i] & 0xFF;
		j++;

		buff[j] = reds[i] >> 8;
		j++;
		buff[j] = reds[i] & 0xFF;
		j++;
	}

}

uint32_t hsv_to_rgb(uint16_t h, uint8_t s, uint8_t v)
{
	uint16_t seg = h/60;
	uint32_t slope = h-(seg*60);
	if (seg & 1)
		slope = 60 - slope;
	slope = slope * 4;
	slope = slope * (v*s); //slope now in the range 0-2^24
	slope >>= 16;
	uint8_t base = ((v*(255-s))>>8);
	slope = slope + base;


	uint32_t out;
	switch (seg)
	{
	case 0:
		out = (240 << 16) | (slope<<8) | base; break;
	case 1:
		out = (slope<<16) | (240 << 8) | base; break;
	case 2:
		out = (base <<16) | (240 << 8) | (slope); break;
	case 3:
		out = (base <<16) | (slope<<8) | (240); break;
	case 4:
		out = (slope<<16) | (base<< 8) | (240); break;
	default:
		out = (240 << 16) | (base <<8) | (slope); break;
	}
	return out;

}
