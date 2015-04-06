void format_packet(uint8_t *buff, uint8_t bc_blue, uint8_t bc_green, uint8_t bc_red, uint16_t *blues , uint16_t *greens , uint16_t *reds);
void format_packet_nixie(uint8_t *buff, uint8_t bc_blue, uint8_t bc_green, uint8_t bc_red, uint32_t *led_values, uint8_t mask);
uint32_t hsv_to_rgb(uint16_t h, uint8_t s, uint8_t v);

#define RED 0xFF0000
#define ORANGE 0xFF1500
#define YELLOW 0xFF8F00
#define GREEN 0x00FF00
#define CYAN 0x00FFFF
#define BLUE 0x0000FF
#define PURPLE 0xFF00FF
