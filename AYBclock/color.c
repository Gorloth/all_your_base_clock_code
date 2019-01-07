/*
* color.c
*/

#include "color.h"

CLR_rgb CLR_hue_to_rgb( uint8_t hue )
{
CLR_rgb rgb;

uint8_t h;
uint8_t x;

h = hue / 42;

if( h % 2 )
    {
    x = 255 - 6 * ( hue - 42 * h );
    }
else
    {
    x = 0 + 6 * ( hue - 42 * h );
    }

switch(h)
    {
    case 0:
    rgb.r = 0xFF;
    rgb.g = x;
    rgb.b = 0;
    break;
        
    case 1:
    rgb.r = x;
    rgb.g = 0xFF;
    rgb.b = 0;
    break;
        
    case 2:
    rgb.r = 0;
    rgb.g = 0xFF;
    rgb.b = x;
    break;
        
    case 3:
    rgb.r = 0;
    rgb.g = x;
    rgb.b = 0xFF;
    break;
        
    case 4:
    rgb.r = x;
    rgb.g = 0;
    rgb.b = 0xFF;
    break;
        
    default:
    rgb.r = 0xFF;
    rgb.g = 0;
    rgb.b = x;
    break;
    }

return rgb;
}
