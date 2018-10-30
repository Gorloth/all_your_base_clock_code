/*
* AYBclock.c
*
* Created: 2/10/2016 5:27:51 PM
*  Author: Kirby
*/
#define F_CPU 16000000

#include <avr/io.h>
#include <util/delay.h>
#include <util/twi.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdbool.h>

#include "clock.h"
#include "math.h"
#include "spi.h"
#include "twi.h"

#define DIGIT_COUNT 10

#define DISPLAY_TIMEOUT       ( 1000 * 5 )
#define QUICK_DISPLAY_TIMEOUT ( 1000 * 2 )

#define DISPLAY_RATE 50
#define DEBOUNCE_RATE 10

#define PWM_TOP 0x0010
#define PWM_TOGGLE ( PWM_TOP * .3f )
#define PWM_RED    ( PWM_TOP * .9f )
#define PWM_GREEN  ( PWM_TOP * .1f )

#define GREEN_EN_PIN        (1<<PORTC1)
#define RED_EN_PIN          (1<<PORTC2)

#define DISPLAY_ENABLE_PIN  (1<<PORTB0)

#define BUTTON_IN_PIN       (1<<PORTC0)

#define CLOCK_IN_PIN        (1<<PORTD7)

#define ENCODER_IN_PINS     (1<<PORTD2|1<<PORTD3)
#define ENCODER_SHIFT       2
#define ENCODER_MASK        0x03

#define A  (1<<0)
#define B  (1<<1)
#define C  (1<<2)
#define D  (1<<3)
#define E  (1<<4)
#define F  (1<<5)
#define G  (1<<6)
#define DP (1<<7)

typedef int8_t encoder_dir;
enum
    {
    ENCODER_CW = -1,
    ENCODER_CCW = 1,
    ENCODER_NONE = 0
    };

typedef uint8_t display_state;
enum
    {
    DISPLAY_TIME,
    DISPLAY_BASE,
    DISPLAY_SET_HOUR,
    DISPLAY_SET_MIN,

    DISPLAY_COUNT
    };


/*------------------------------------

  ---A---
 |       |
 F       B
 |       |
  ---G---
 |       |
 E       C
 |       |
  ---D---

------------------------------------*/

static const uint8_t digits[] =
    {
    (A|B|C|D|E|F),   //0
    (B|C),           //1
    (A|B|G|E|D),     //2
    (A|B|G|C|D),     //3
    (F|G|B|C),       //4
    (A|F|G|C|D),     //5
    (A|F|G|C|E|D),   //6
    (A|B|C),         //7
    (A|B|C|D|E|F|G), //8
    (A|B|C|D|F|G),   //9
    (A|B|C|E|F|G),   //A
    (F|G|C|D|E),     //B
    (G|E|D),         //C
    (E|G|B|C|D),     //D
    (A|F|G|E|D),     //E
    (A|F|G|E),       //F
    (A|C|D|E|F),     //G
    (B|C|E|F|G),     //H
    (A|B|C|D),       //I
    (B|C|D),         //J
    (B|C|E|F|G),     //K
    (D|E|F),         //L
    (A|C|E|G),       //M
    (C|E|G),         //N
    (A|B|C|D|E|F),   //O
    (A|B|E|F|G),     //P
    (A|B|C|F|G),     //Q
    (E|G),           //R
    (A|C|D|F|G),     //S
    (D|E|F|G),       //T
    (B|C|D|E|F),     //U
    (C|D|E),         //V
    (B|D|F),         //W
    (B|C|E|F|G),     //X
    (B|C|D|F|G),     //Y
    (A|B|D|E|G),     //Z
    (0)              //None
    };

static const uint8_t *numbers = digits;

static const uint8_t *letters = &digits[10];

static void format_display(int8_t base, CLK_time_type time);
static void init_timers(void);
static void display(void);
static encoder_dir encoder_read(void);
static void format_string( const char *string );
static void process_knob( CLK_time_type time );

static volatile uint16_t v_ms;
static volatile uint16_t v_time;

static volatile uint8_t v_disp_update_flag;

static volatile display_state v_disp_state;

static volatile uint16_t v_disp_timer;

static uint8_t  disp_red[DIGIT_COUNT];
static uint8_t  disp_green[DIGIT_COUNT];

static int8_t   s_base;

static void init_timers(void)
{
//PB1 output
//      clear PB1 on compare match,  Fast PWM mode
TCCR1A = 1 << COM1A1 | 0 << WGM11 | 0 << WGM10;
//       clock / 1024
TCCR1B = 1 << CS12 | 0 << CS11 | 1 << CS10 | 1 << WGM13 | 0 << WGM12;

//Top
ICR1 = PWM_TOP;

//Toggle at
OCR1A = PWM_TOGGLE;

OCR2A = 250 - 1;
TCCR2A = 1 << COM2A1 | 1 << WGM21;
TCCR2B = 1 << CS22;
TIMSK2 = 1 << OCIE2A;
}

ISR(TIMER2_COMPA_vect)
{
static uint16_t count;
static uint16_t disp_count;
static uint8_t  btn_count;
static uint8_t  btn_data;

count++;
v_ms++;
btn_count++;
if(btn_count == DEBOUNCE_RATE)
    {
    btn_count = 0;
    btn_data = btn_data << 1;
    btn_data |= (PINC & BUTTON_IN_PIN);
    if(btn_data == 0x80)
        {
        v_disp_state = (v_disp_state + 1) % DISPLAY_COUNT;
        v_disp_timer = DISPLAY_TIMEOUT;
        }
    }
if(v_disp_timer)
    {
    v_disp_timer--;
    if(!v_disp_timer)
        {
        v_disp_state = DISPLAY_TIME;
        OCR1A = PWM_TOGGLE;
        }
    }
if(disp_count++ == DISPLAY_RATE)
    {
    v_disp_update_flag = true;
    disp_count = 0;
    }
if(count == 1000)
    {
    count = 0;
    v_time++;
    if(v_time == 10000)
        {
        v_time = 0;
        }
    }
}

//******************************************************************************
//                          encoder_read
// reads the value from the encoder calculates the direction of turning by
// checking if the correct sequence is seen. Once a turn starting in that 
// direction is seen (0b01 or 0b10) all future inputs will be ignored unless 
// they progress the turn sequence or they indicate the knob is back in a 
// divot (0b00).
static encoder_dir encoder_read( void )
{
static const uint8_t    expected[2][4] = { { 0b01, 0b11, 0b10, 0b00 },   //CCW
                                           { 0b10, 0b11, 0b01, 0b00 } }; //CW
static uint8_t          index;
static uint8_t          dir_index;
uint8_t                 data_in;
encoder_dir             dir;

data_in = ((PIND & ENCODER_IN_PINS) >> ENCODER_SHIFT ) & ENCODER_MASK;

dir = ENCODER_NONE;

if( data_in == 0 )
    {
    if( index == 3 )
        {
        if( dir_index == 0 )
            {
            dir = ENCODER_CCW;
            }
        else
            {
            dir = ENCODER_CW;
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
//                          format_string
// Takes a string and adds it to the display in yellow starting at the left end
static void format_string
    (
    const char *string
    )
{
int8_t      i;
uint8_t     letter;

i = DIGIT_COUNT;

while( *string && i-- > 0 )
    {
    letter = ( *string & ~0x20 );
    letter = letters[letter - 'A'];
    disp_green[i] = letter;
    disp_red[i] = letter;
    string++;
    }
}

//******************************************************************************
//                          format_display
// formats the display based on the current display mode. Either shows the
// current time or one of the menus based on the current state
static void format_display
    (
    int8_t base,
    CLK_time_type time
    )
{
int8_t      sec_str[DIGIT_COUNT];
int8_t      min_str[DIGIT_COUNT];
int8_t      hr_str[DIGIT_COUNT];

uint8_t    *disp_ptr;

uint8_t     i;

uint8_t     sec_len;
uint8_t     min_len;
uint8_t     max_len;
uint8_t     hr_len;
uint8_t     offset;

bool        overlap;
bool        dp_blink;
bool        dp_fill;
uint8_t     dp_start;
uint8_t     dp_end;
uint8_t     dp_green;
uint8_t     dp_red;

for(i = 0; i < DIGIT_COUNT; i++)
    {
    disp_green[i] = 0;
    }

for(i = 0; i < DIGIT_COUNT; i++)
    {
    disp_red[i] = 0;
    }

if(v_disp_state == DISPLAY_BASE )
    {
    format_string( "BASE");

    min_len = MTH_convert_to_base(abs(base), 10, min_str, DIGIT_COUNT);

    if(base < 0)
        {
        disp_ptr = disp_red;
        }
    else
        {
        disp_ptr = disp_green;
        }

    for(i = 0; i < min_len; i++)
        {
        *disp_ptr = numbers[min_str[i]];
        disp_ptr++;
        }

    if(base < 0)
        {
        disp_red[i] = G;
        }
    }
else
    {
    overlap = false;
    dp_blink = ((PIND & CLOCK_IN_PIN) == CLOCK_IN_PIN);
    max_len = MTH_convert_to_base(59, base, hr_str, DIGIT_COUNT);

    sec_len = MTH_convert_to_base(time.second, base, sec_str, DIGIT_COUNT);
    for(i = sec_len; i < max_len; i++)
        {
        sec_str[i] = 0;
        }
    sec_len = max_len;

    min_len = MTH_convert_to_base(time.minute, base, min_str, DIGIT_COUNT);
    for(i = min_len; i < max_len; i++)
        {
        min_str[i] = 0;
        }
    min_len = max_len;

    max_len = MTH_convert_to_base(23, base, hr_str, DIGIT_COUNT);
    hr_len = MTH_convert_to_base(time.hour, base, hr_str, DIGIT_COUNT);
    for(i = hr_len; i < max_len; i++)
        {
        hr_str[i] = 0;
        }
    hr_len = max_len;

    // |Base| > 20 needs to use red/green for the digits, thus can't overlay
    if(base > 20 || base < -20)
        {
        dp_blink = true;
        for(i = 0; i < sec_len; i++)
            {
            if(sec_str[i] >= 40)
                {
                disp_red[i] = numbers[sec_str[i] - 40];
                }
            else if(sec_str[i] >= 20)
                {
                disp_green[i] = numbers[sec_str[i] - 20];
                disp_red[i] = numbers[sec_str[i] - 20];
                }
            else
                {
                disp_green[i] = numbers[sec_str[i]];
                }
            }
        offset = sec_len;
        for(i = 0; i < min_len; i++)
            {
            if(min_str[i] >= 40)
                {
                disp_red[i+offset] = numbers[min_str[i] - 40];
                }
            else if(min_str[i] >= 20)
                {
                disp_green[i+offset] = numbers[min_str[i] - 20];
                disp_red[i+offset] = numbers[min_str[i] - 20];
                }
            else
                {
                disp_green[i+offset] = numbers[min_str[i]];
                }
            }
        offset += min_len;
        for(i = 0; i < hr_len; i++)
            {
            if(hr_str[i] >= 40)
                {
                disp_red[i+offset] = numbers[hr_str[i] - 40];
                }
            else if(hr_str[i] >= 20)
                {
                disp_green[i+offset] = numbers[hr_str[i] - 20];
                disp_red[i+offset] = numbers[hr_str[i] - 20];
                }
            else
                {
                disp_green[i+offset] = numbers[hr_str[i]];
                }
            }
        }
    // if minutes and hours are too long to fit next to each other overlay
    // hours in red, minutes in green
    else if((min_len + hr_len) > DIGIT_COUNT)
        {
        overlap = true;
        sec_len = 0;
        for(i = 0; i < min_len; i++)
            {
            disp_green[i] = numbers[min_str[i]];
            }
        for(i = 0; i < hr_len; i++)
            {
            disp_red[i] = numbers[hr_str[i]];
            }
        }
    else
        {
        offset = 0;
        if( sec_len + min_len + hr_len <= DIGIT_COUNT )
            {
            for(i = 0; i < sec_len; i++)
                {
                disp_green[i] = numbers[sec_str[i]];
                disp_red[i] = numbers[sec_str[i]];
                }
            offset += sec_len;
            dp_blink = true;
            }
        else
            {
            sec_len = 0;
            }
        for(i = 0; i < min_len; i++)
            {
            disp_green[i+offset] = numbers[min_str[i]];
            }
        offset += min_len;
        for(i = 0; i < hr_len; i++)
            {
            disp_red[i+offset] = numbers[hr_str[i]];
            }
        }

    /*-------------------------------------------
    Calculate the decimal point placement, If the
    display mode is setting time highlight the
    decimal points for the numbers being set
    otherwise put it between the hour/min/sec
    divide
    -------------------------------------------*/
    dp_green = DP;
    dp_red = DP;
    
    switch( v_disp_state )
        {
        /*---------------------------------------
        Highlight the hours with decimal points
        while setting, use yellow if the hours
        are separate, otherwise use the hour
        color (red)
        ---------------------------------------*/
        case DISPLAY_SET_HOUR:
            if( overlap )
                {
                dp_green = 0;
                dp_start = sec_len;
                dp_end = sec_len + hr_len;
                dp_fill = true;
                }
            else
                {
                dp_start = sec_len + min_len;
                dp_end = sec_len + min_len + hr_len;
                dp_fill = true;
                }
            break;

        /*---------------------------------------
        Highlight the minutes with decimal points
        while setting, use yellow if the minutes
        are separate, otherwise use the minute
        color (green)
        ---------------------------------------*/
        case DISPLAY_SET_MIN:
            if( overlap )
                {
                dp_red = 0;
                }
            dp_start = sec_len;
            dp_end = sec_len + min_len;
            dp_fill = true;
            break;

        /*---------------------------------------
        If the hour and minutes are overlapping
        then put the flashing second at the far
        right. If there is no overlap put the dot
        between each group (H/M/S)
        ---------------------------------------*/
        default:
            dp_start = sec_len;
            if( overlap )
                {
                dp_end = dp_start;
                }
            else
                {
                dp_end = sec_len + min_len;
                }
            dp_fill = false;
            break;
        }

    if( dp_fill )
        {
        for( i = dp_start; i < dp_end; i++ )
            {
            if( dp_blink )
                {
                disp_green[i] |= dp_green;
                disp_red[i] |= dp_red;
                }
            }
        }
    else if( dp_blink )
        {
        disp_green[dp_start] |= dp_green;
        disp_red[dp_start] |= dp_red;
        disp_green[dp_end] |= dp_green;
        disp_red[dp_end] |= dp_red;
        }
    }
}

static void display(void)
{
int8_t      i;

PORTB |= DISPLAY_ENABLE_PIN;

for(i = DIGIT_COUNT - 1; i >= 0; i--)
    {
    spi(disp_green[i]);
    }
PORTC &= ~RED_EN_PIN;
PORTC |= RED_EN_PIN;

for(i = DIGIT_COUNT - 1; i >= 0; i--)
    {
    spi(disp_red[i]);
    }
PORTC &= ~GREEN_EN_PIN;
PORTC |= GREEN_EN_PIN;

PORTB &= ~DISPLAY_ENABLE_PIN;
}

static void process_knob( CLK_time_type time )
{
encoder_dir     dir;

dir = encoder_read();
if( dir == ENCODER_NONE )
    {
    return;
    }

switch( v_disp_state )
    {
    case DISPLAY_TIME:
        v_disp_state = DISPLAY_BASE;
        v_disp_update_flag = true;
        v_disp_timer = QUICK_DISPLAY_TIMEOUT;
        break;
        
    case DISPLAY_BASE:
        s_base += dir;
        if( s_base == 1 || s_base == -1 )
            {
            s_base = s_base + 3 * dir;
            }
        else if( s_base == 61 || s_base == -61 )
            {
            s_base -= dir;
            }
        v_disp_update_flag = true;
        v_disp_timer = QUICK_DISPLAY_TIMEOUT;
        break;

    case DISPLAY_SET_HOUR:
        time.hour = ( time.hour + 24 + dir) % 24;
        v_disp_update_flag = true;
        CLK_time_set( time );
        v_disp_timer = DISPLAY_TIMEOUT;
        break;

    case DISPLAY_SET_MIN:
        time.minute = ( time.minute + 60 + dir) % 60;
        v_disp_update_flag = true;
        CLK_time_set( time );
        v_disp_timer = DISPLAY_TIMEOUT;
        break;

    default:
        break;
    }
}

int main(void)
{
CLK_time_type   time;

DDRB = 0xFF;
DDRC = GREEN_EN_PIN | RED_EN_PIN;
PORTD = CLOCK_IN_PIN;
PORTC = GREEN_EN_PIN | RED_EN_PIN;

init_timers();
spi_init();
TWI_init();
CLK_init();

v_ms = 0;
time.hour = 12;
time.minute = 59;
s_base = 10;

sei();

while(1)
    {
    time = CLK_time_get();
    process_knob( time );
    format_display(s_base, time);

    if(v_disp_update_flag == true)
        {
        v_disp_update_flag = false;
        display();
        }

    }
}