/*
* clock.h
*/

#include <stdint.h>

typedef struct{
    uint8_t  hour;
    uint8_t  minute;
    uint8_t  second;
    } CLK_time_type;

CLK_time_type CLK_time_get(void);

void CLK_init(void);

void CLK_time_set( CLK_time_type time );
