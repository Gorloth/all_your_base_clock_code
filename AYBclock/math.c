/*
* math.c
*/

#include <stdlib.h>

#include "math.h"

uint8_t MTH_bcd_to_int(uint8_t data)
{
return((data & 0x0F) + ((data & 0xF0) >> 4) * 10);
}

uint8_t MTH_int_to_bcd(uint8_t data)
{
return( ( data % 10 ) | ( ( data / 10 ) << 4 ) );
}

uint8_t MTH_convert_to_base
    (
    int8_t      number,
    int8_t      base,
    int8_t     *output,
    uint8_t     out_sz
    )
{
uint8_t         i;

i = 0;

for(i = 0; i < out_sz && number; i++)
    {
    *output = number % base;
    number = number / base;
    if(*output < 0)
        {
        *output += abs(base);
        number++;
        }
    output++;
    }

return(i);
}

//The regular MTH_convert_to_base() function has issues with numbers >127
//The unsigned versions has issues with negative bases, currently the only
//place something >127 is displayed is in settings where the base is always >0
uint8_t MTH_convert_to_base_unsigned(
    uint8_t     number,
    int8_t      base,
    int8_t     *output,
    uint8_t     out_sz
    )
{
uint8_t         i;

i = 0;

for(i = 0; i < out_sz && number; i++)
    {
    *output = number % base;
    number = number / base;
    if(*output < 0)
        {
        *output += abs(base);
        number++;
        }
    output++;
    }

return(i);
}


