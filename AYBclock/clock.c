/*
* clock.c
*/

#include <stdlib.h>

#include "clock.h"
#include "math.h"
#include "twi.h"

#define CLK_ADDR_READ     0xDF
#define CLK_ADDR_WRITE    0xDE

void CLK_init(void)
{
//Start Clock
TWI_start();
TWI_write(CLK_ADDR_WRITE);
TWI_write(0x00);
TWI_write(0x80);
TWI_stop();

//Enable 1Hz outptut
TWI_start();
TWI_write(CLK_ADDR_WRITE);
TWI_write(0x07);
TWI_write((1 << 6));
TWI_stop();
}

CLK_time_type CLK_time_get(void)
{
uint8_t         second;
uint8_t         minute;
uint8_t         hour;
CLK_time_type   time;

time.hour = 0;
time.minute = 0;

TWI_start();
TWI_write(CLK_ADDR_WRITE);
TWI_write(0x00);
TWI_start();
TWI_write(CLK_ADDR_READ);
second = TWI_read(true);
minute = TWI_read(true);
hour = TWI_read(false);
TWI_stop();

time.second = MTH_bcd_to_int(second & 0x7f);
time.minute = MTH_bcd_to_int(minute & 0x7F);
time.hour = MTH_bcd_to_int(hour & 0x3f);

return(time);
}

void CLK_time_set( CLK_time_type time )
{
TWI_start();
TWI_write(CLK_ADDR_WRITE);
TWI_write(0x01);
TWI_write(MTH_int_to_bcd(time.minute));
TWI_write(MTH_int_to_bcd(time.hour));
TWI_stop();
}
