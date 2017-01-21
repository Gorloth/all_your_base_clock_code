/*
* clock.h
*/

#include <stdint.h>

typedef struct{
    int8_t  hour;
    int8_t  minute;
    int8_t  second;
    } CLK_time_type;

CLK_time_type CLK_time_get(void);

void CLK_init(void);

void CLK_time_set( CLK_time_type time );
