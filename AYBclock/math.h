/*
* math.c
*/

#include <stdint.h>

uint8_t MTH_convert_to_base(
    int8_t      number,
    int8_t      base,
    int8_t     *output,
    uint8_t     out_sz
    );

uint8_t MTH_bcd_to_int(uint8_t data);

uint8_t MTH_int_to_bcd(uint8_t data);
