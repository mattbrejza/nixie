#include <libopencm3/stm32/gpio.h>

#if defined(STM32F0)
#define LED_SPI SPI1_I2S1_BASE
#elif defined(STM32F1)
#define LED_SPI SPI1
#else
#error "stm32 family not defined."
#endif


#define TEMP_PORT  GPIOA
#define TEMP_PIN   GPIO0

#define RTCCS_PORT GPIOA
#define RTCCS_PIN  GPIO2

#define IR_PORT    GPIOA
#define IR_PIN     GPIO3

#define SCLK_PORT  GPIOA
#define SCLK_PIN   GPIO5

#define R_MI_PORT  GPIOA
#define R_MI_PIN   GPIO6

#define MOSI_PORT  GPIOA
#define MOSI_PIN   GPIO7

#define PPS_PORT   GPIOA
#define PPS_PIN    GPIO8

#define TX_PORT    GPIOA
#define TX_PIN     GPIO9

#define RX_PORT    GPIOA
#define RX_PIN     GPIO10

#define S1_PORT    GPIOA
#define S1_PIN     GPIO11

#define S2_PORT    GPIOA
#define S2_PIN     GPIO12

#define R_CLK_PORT GPIOB
#define R_CLK_PIN  GPIO0

#define R_MO_PORT  GPIOB
#define R_MO_PIN   GPIO1

#define BUZZ_PORT  GPIOB
#define BUZZ_PIN   GPIO4

#define HV_PORT    GPIOB
#define HV_PIN     GPIO5

#define A_PORT     GPIOB
#define A_PIN      GPIO6

#define B_PORT     GPIOB
#define B_PIN      GPIO7

#define C_PORT     GPIOB
#define C_PIN      GPIO8

#define D_PORT     GPIOB
#define D_PIN      GPIO9

#define ABCD_MASK  (GPIO6 | GPIO7 | GPIO8 | GPIO9)
#define EF_MASK (GPIO12 | GPIO13)

#define NDAT_PORT  GPIOB
#define NDAT_PIN   GPIO10

#define NCLK_PORT  GPIOB
#define NCLK_PIN   GPIO11

#define CC1_PORT   GPIOB
#define CC1_PIN    GPIO12

#define CC2_PORT   GPIOB
#define CC2_PIN    GPIO13

#define S3_PORT    GPIOB
#define S3_PIN     GPIO14

#define S4_PORT    GPIOB
#define S4_PIN     GPIO15



