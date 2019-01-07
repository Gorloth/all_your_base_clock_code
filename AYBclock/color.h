/*
* color.h
*/

#include <stdint.h>

typedef struct
    {
    uint8_t r;
    uint8_t g;
    uint8_t b;
    } CLR_rgb;

CLR_rgb CLR_hue_to_rgb( uint8_t hue );
