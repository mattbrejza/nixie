#include <inttypes.h>

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
	}
	j = 12;
	for (i = 0; i < 4; i++)
	{
		buff[j] = greens[i] >> 8;
		j++;
		buff[j] = greens[i] & 0xFF;
		j++;
	}
	j = 20;
	for (i = 0; i < 4; i++)
	{
		buff[j] = reds[i] >> 8;
		j++;
		buff[j] = reds[i] & 0xFF;
		j++;
	}


}
