/*
* encoder.c
*/

#include <avr/io.h>

#include "encoder.h"
#include "color.h"

#define ENCODER_IN_PINS     (1<<PORTD1|1<<PORTD2)
#define ENCODER_SHIFT       1
#define ENCODER_MASK        0x03

#define RED     OCR1B
#define GREEN   OCR0B
#define BLUE    OCR0A

void ENC_init( void )
{
DDRD |= 1 << 5 | 1 << 6;
DDRB |= 1 << 2;
}

//******************************************************************************
//                          ENC_read
// reads the value from the encoder calculates the direction of turning by
// checking if the correct sequence is seen. Once a turn starting in that 
// direction is seen (0b01 or 0b10) all future inputs will be ignored unless 
// they progress the turn sequence or they indicate the knob is back in a 
// divot (0b00).
ENC_dir ENC_read( void )
{
static const uint8_t    expected[2][4] = { { 0b10, 0b00, 0b01, 0b11 },   //CCW
                                           { 0b01, 0b00, 0b10, 0b11 } }; //CW
static uint8_t          index;
static uint8_t          dir_index;
uint8_t                 data_in;
ENC_dir                 dir;

data_in = ((PIND & ENCODER_IN_PINS) >> ENCODER_SHIFT ) & ENCODER_MASK;

dir = ENC_NONE;

if( data_in == 3 )
    {
    if( index == 3 )
        {
        if( dir_index == 0 )
            {
            dir = ENC_CCW;
            }
        else
            {
            dir = ENC_CW;
            }
        }
    index = 0;
    }
else if( index == 0 )
    {
    if( data_in == expected[0][0] )
        {
        dir_index = 0;
        index++;
        }
    else if( data_in == expected[1][0] )
        {
        dir_index = 1;
        index++;
        }
    }
else if( data_in == expected[dir_index][index] )
    {
    index++;
    }

return(dir);
}

//******************************************************************************
//                          ENC_set_color
// Sets the color of the encoder knob to the given hue
void ENC_set_color( uint8_t hue )
{
CLR_rgb     rgb;

rgb = CLR_hue_to_rgb( hue );

ENC_init();

RED = rgb.r;
GREEN = rgb.g;
BLUE = rgb.b;

}

//******************************************************************************
//                          ENC_clear_color
// Turns off the encoder LEDs entirely
void ENC_clear_color( void )
{
RED = 0;
GREEN = 0;
BLUE = 0;

DDRD &= ~( 1 << 5 | 1 << 6 );
}