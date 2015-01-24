#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/dma.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <pindef.h>
#include <tlc79711.h>

void init(void);
void _delay_ms(const uint32_t delay);
void reinit_spi_dma(uint8_t *mem_addr, uint16_t bytes);
void increment_digit(void);
void reset_digits(void);

const uint8_t buff_test[] = {0xFF, 8, 8, 0, 0xFF, 8, 0, 8, 0xFF, 8};

uint8_t buff_led[56];

//const uint8_t led_lut[] = {9,8,7,6,5,4,};
volatile uint32_t digits_bcd = 0;
volatile uint8_t digit_pos = 0;

int main(void)
{
	init();

	digits_bcd = 0x263134;

	//GPIO_BRR(HV_PORT) = HV_PIN;	//Turn on HV
	GPIOB_BSRR = (1<<5)<<16;
	//gpio_clear(HV_PORT,HV_PIN);	//Turn on HV
	GPIO_BSRR(CC1_PORT) = CC1_PIN;



    //while(1)
    {
    	//void format_packet(uint8_t *buff, uint8_t bc_blue, uint8_t bc_green, uint8_t bc_red,
    	//		uint16_t *blues , uint16_t *greens , uint16_t *reds);

    	uint16_t ledsr[] = {0xFFFF,0xFFFF,0xFFFF,0xFFFF};
    	uint16_t ledsg[] = {0xFFFF,0xFFFF,0xFFFF,0xFFFF};
    	uint16_t ledsb[] = {0xFFFF,0xFFFF,0xFFFF,0xFFFF};

    	format_packet(buff_led, 0x2, 0x2F, 0x2F, ledsb,ledsg,ledsr);
    	format_packet(&buff_led[28], 0x2F, 0x2F, 0x2F, ledsb,ledsg,ledsr);
    	//spi_test(0x5F);
    	reinit_spi_dma(buff_led, 56);
    	spi_enable_tx_dma(LED_SPI);
  //  	GPIO_BSRR(BUZZ_PORT) = BUZZ_PIN | R_MO_PIN;
    	_delay_ms(100);
  //  	GPIO_BRR(BUZZ_PORT) = BUZZ_PIN | R_MO_PIN;
    	_delay_ms(1);


    }

    while(1)
	{

		increment_digit();
		_delay_ms(1);
	}
}


void increment_digit(void)
{
	GPIO_BSRR(NDAT_PORT) = NDAT_PIN<<16;
	digit_pos++;
	uint8_t i;
	GPIO_ODR(A_PORT) = (GPIO_ODR(A_PORT) & ~ABCD_MASK);

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
	GPIO_BSRR(NCLK_PORT) = NCLK_PIN<<16;

//	for (i = 0; i < 15; i++)
//		__asm__("nop");

	uint32_t digit = (digits_bcd >> (4*(5-digit_pos))) & 0xF;
	digit = (9-digit) << 6;

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

	//rcc_clock_setup_in_hsi_out_24mhz();

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
	gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, IR_PIN );

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
    uint32_t i, j;

    for(i=0; i< delay; i++)
        for(j=0; j<600; j++)
            __asm__("nop");
}
