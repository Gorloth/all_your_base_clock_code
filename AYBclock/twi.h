/*
* twi.h
*/

#include <stdlib.h>
#include <stdbool.h>

void TWI_init(void);

uint8_t TWI_start(void);

uint8_t TWI_write(uint8_t data);

uint8_t TWI_read(bool ack);

void TWI_stop(void);
