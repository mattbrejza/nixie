#include <pindef.h>

void init_ds3234(void);
uint8_t ds3234_read(uint8_t addr);
void ds3234_write(uint8_t addr, uint8_t data);
void ds3234_enable_1hz_out(void);
uint32_t ds3234_read_time_bcd(void);
void ds3234_write_time_bcd(uint32_t t);
uint8_t ds3234_check_valid_time(void);
uint32_t ds3234_read_date_bcd(void);
void ds3234_write_date_bcd(uint32_t t);
uint8_t ds3234_check_valid_date(void);
