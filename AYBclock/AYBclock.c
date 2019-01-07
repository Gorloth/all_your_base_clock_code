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
#include <stdint.h>
#include <stdbool.h>

#include "color.h"
#include "clock.h"
#include "encoder.h"
#include "math.h"
#include "spi.h"
#include "twi.h"

#define DIGIT_COUNT 10

#define DISPLAY_TIMEOUT       ( 1000 * 5 )
#define QUICK_DISPLAY_TIMEOUT ( 1000 * 2 )

#define DISPLAY_RATE 50
#define DEBOUNCE_RATE 10

#define PWM_TOP 0x00FF
#define PWM_TOGGLE ( PWM_TOP * .2f )
#define PWM_RED    ( PWM_TOP * .9f )
#define PWM_GREEN  ( PWM_TOP * .1f )

#define RAND_BASE_MAX       61

#define GREEN_EN_PIN        (1<<PORTC1)
#define RED_EN_PIN          (1<<PORTC2)

#define DISPLAY_ENABLE_PIN  (1<<PORTB0)

#define BUTTON_IN_PIN       (1<<PORTC0)

#define CLOCK_IN_PIN        (1<<PORTD7)

#define A  (1<<0)
#define B  (1<<1)
#define C  (1<<2)
#define D  (1<<3)
#define E  (1<<4)
#define F  (1<<5)
#define G  (1<<6)
#define DP (1<<7)

typedef uint8_t display_state;
enum
    {
    DISPLAY_TIME,
    DISPLAY_BASE,
    DISPLAY_SET_HOUR,
    DISPLAY_SET_MIN,
    DISPLAY_SETTINGS,
    DISPLAY_COLOR_BAL,
    DISPLAY_COLOR,
    DISPLAY_RAND_BASE,

    DISPLAY_COUNT
    };
    
typedef uint8_t setting_type;
enum
    {
    SETTING_MAIN_MENU,
    SETTING_SEC_DISP,
    SETTING_OVERLAP,
    SETTING_BASE_DISP,
    SETTING_12_HOUR,
    SETTING_ADVANCED,
    
    SETTING_COUNT
    };

typedef uint8_t color_type;
enum
    {
    COLOR_OFF,
    COLOR_BASE,
    COLOR_SEC,
    COLOR_MIN,
    COLOR_HR,
    COLOR_RAND,
    COLOR_FADE,
    
    COLOR_COUNT
    };
    
static const char *color_strings[COLOR_COUNT] =
    {
    "OFF",
    "BASE",
    "SEC",
    "MIN",
    "HOUR",
    "RAND",
    "FADE"
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
    (0),             //None
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
    (A|B|D|E|G)      //Z
    };

static const uint8_t *numbers = &digits[1];

static const uint8_t *letters = &digits[11];

static void display_setting( bool setting );
static void format_display(int8_t base, CLK_time_type time);
static void init_timers(void);
static void display(void);
static void format_menu_string( const char *string );
static void format_setting_string( const char *string );
static void format_setting_num( uint8_t num );
static void check_rand_base( CLK_time_type time );
static void process_knob( CLK_time_type time );
static void set_knob_color( CLK_time_type time );

static volatile uint16_t v_ms;
static volatile uint16_t v_time;

static volatile uint8_t v_disp_update_flag;

static volatile display_state v_disp_state;
static volatile setting_type v_settings_state;

static volatile uint16_t v_disp_timer;

static uint8_t  disp_red[DIGIT_COUNT];
static uint8_t  disp_green[DIGIT_COUNT];

static int8_t   s_base;
static bool     s_settings[SETTING_COUNT];

static bool             s_rand_init;
static unsigned int     s_rand_seed;

static color_type   s_color_setting;

static uint8_t      s_rand_base_interval;

static uint16_t s_yellow_point;

static const char *s_setting_strings[SETTING_COUNT] =
    {
    "Settings",
    "Seconds",
    "Overlap",
    "DSPBase",
    "12Hour",
    "ADVSetting"
    };

static void init_timers(void)
{
//PB1 output
//      clear PB1 on compare match,  Fast PWM mode
//PB2 output for Red encoder LED PWM, clear on match (inverted)
TCCR1A = 1 << COM1A1 | 1 << COM1B1 | 1 << COM1B0 | 0 << WGM11 | 0 << WGM10;
//       clock / 64
TCCR1B = 0 << CS12 | 1 << CS11 | 1 << CS10 | 1 << WGM13 | 0 << WGM12;

//Top
ICR1 = PWM_TOP;

//Toggle at
OCR1A = PWM_TOGGLE;
s_yellow_point = PWM_TOGGLE;

OCR2A = 250 - 1;
TCCR2A = 1 << COM2A1 | 1 << WGM21;
TCCR2B = 1 << CS22;
TIMSK2 = 1 << OCIE2A;

//Encoder LED PWM, clear on match (inverted)
TCCR0A = 1 << COM0A1 | 1 << COM0A0 | 1 << COM0B1 | 1 << COM0B0 | 1 << WGM01 | 1 << WGM00;
TCCR0B = 1 << CS00;

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
    if(btn_data == 0x7F)
        {
        if( v_disp_state == DISPLAY_SETTINGS )
            {
            if( v_settings_state == SETTING_MAIN_MENU )
                {
                v_disp_state = 0;
                }
            else if( v_settings_state == SETTING_ADVANCED )
                {
                v_disp_state++;
                }
            else
                {
                s_settings[v_settings_state] = !s_settings[v_settings_state];
                }
            }
        else
            {
            v_disp_state = (v_disp_state + 1) % DISPLAY_COUNT;
            if( v_disp_state == DISPLAY_SETTINGS )
                {
                v_settings_state = SETTING_MAIN_MENU;
                }
            v_disp_timer = DISPLAY_TIMEOUT;
            }
        }
    }
if(v_disp_timer)
    {
    v_disp_timer--;
    if(v_disp_timer == 0)
        {
        v_disp_state = DISPLAY_TIME;
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
//                          format_menu_string
// Takes a string and adds it to the display in yellow starting at the left end
static void format_menu_string
    (
    const char *string
    )
{
int8_t      i;
uint8_t     letter;

i = DIGIT_COUNT;

while( *string && i-- > 0 )
    {
    if( *string == ' ' )
        {
        letter = 0;
        }
    else if( *string >= '0' && *string <= '9' )
        {
        letter = numbers[*string - '0'];
        }
    else
        {
        letter = ( *string & ~0x20 );
        letter = letters[letter - 'A'];
        }
    disp_green[i] = letter;
    disp_red[i] = letter;
    string++;
    }
}

//******************************************************************************
//                          display_setting
// Displays the state of a boolean setting of 'On' or 'Off'
static void display_setting
    (
    bool    setting
    )
{
if( setting )
    {
    format_setting_string( "ON" );
    }
else
    {
    format_setting_string( "OFF" );
    }
}


//******************************************************************************
//                          format_setting_num
// Takes a number and adds it to the display in green starting at the right end
static void format_setting_num
    (
    uint8_t     num
    )
{
uint8_t         i;

i = 0;

do
    {
    disp_green[i++] = numbers[ num % 10 ];
    num = num / 10;
    } while( num );
}


//******************************************************************************
//                          format_setting_string
// Takes a string and adds it to the display in green starting at the right end
static void format_setting_string
    (
    const char *string
    )
{
int8_t      i;
uint8_t     letter;

i = 0;

while( *string++ )
    {
    i++;
    }

string -= i + 1;

while( *string && i-- > 0 )
    {
    if( *string == ' ' )
        {
        letter = 0;
        }
    else if( *string >= '0' && *string <= '9' )
        {
        letter = numbers[*string - '0'];
        }
    else
        {
        letter = ( *string & ~0x20 );
        letter = letters[letter - 'A'];
        }
    disp_green[i] = letter;
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
int8_t      temp_str[DIGIT_COUNT];

uint8_t    *disp_ptr;

uint8_t     i;

uint8_t     sec_len;
uint8_t     min_len;
uint8_t     hr_len;
uint8_t     max_len;
uint8_t     temp_len;
uint8_t     offset;

uint8_t     hour;

bool        overlap;
bool        dp_blink;
bool        dp_fill;
uint8_t     dp_start;
uint8_t     dp_end;
uint8_t     dp_green;
uint8_t     dp_red;
uint16_t    pwm_setting;

for(i = 0; i < DIGIT_COUNT; i++)
    {
    disp_green[i] = 0;
    }

for(i = 0; i < DIGIT_COUNT; i++)
    {
    disp_red[i] = 0;
    }
    
pwm_setting = s_yellow_point;

switch( v_disp_state )
    {
    case DISPLAY_BASE:
        format_menu_string( "BASE" );

        temp_len = MTH_convert_to_base(abs(base), 10, temp_str, DIGIT_COUNT);

        if(base < 0)
            {
            disp_ptr = disp_red;
            }
        else
            {
            disp_ptr = disp_green;
            }

        for(i = 0; i < temp_len; i++)
            {
            *disp_ptr = numbers[temp_str[i]];
            disp_ptr++;
            }

        if(base < 0)
            {
            disp_red[i] = G;
            }
        break;

    case DISPLAY_TIME:
    case DISPLAY_SET_HOUR:
    case DISPLAY_SET_MIN:
        overlap = false;
        dp_blink = ((PIND & CLOCK_IN_PIN) == CLOCK_IN_PIN);
        max_len = MTH_convert_to_base(59, base, min_str, DIGIT_COUNT);

        if( s_settings[SETTING_SEC_DISP] )
            {
            sec_len = MTH_convert_to_base(time.second, base, sec_str, DIGIT_COUNT);
            for(i = sec_len; i < max_len; i++)
                {
                sec_str[i] = 0;
                }
            sec_len = max_len;
            }
        else
            {
            sec_len = 0;
            }

        min_len = MTH_convert_to_base(time.minute, base, min_str, DIGIT_COUNT);
        for(i = min_len; i < max_len; i++)
            {
            min_str[i] = 0;
            }
        min_len = max_len;

        if( s_settings[SETTING_12_HOUR] )
            {
            max_len = MTH_convert_to_base(12, base, hr_str, DIGIT_COUNT);
            }
        else
            {
            max_len = MTH_convert_to_base(23, base, hr_str, DIGIT_COUNT);
            }
        
        hour = time.hour;
        if( s_settings[SETTING_12_HOUR] )
            {
            hour = hour % 12;
            if( hour == 0 )
                {
                hour = 12;
                }
            }
        hr_len = MTH_convert_to_base(hour, base, hr_str, DIGIT_COUNT);
        
        for(i = hr_len; i < max_len; i++)
            {
            //For 12 mode don't pad 0's on the hours (still keep the size for determining overlap though)
            if( s_settings[SETTING_12_HOUR] )
                {
                //the number array starts at the second element of
                //the digits array, the first element is a blank
                hr_str[i] = -1;
                }
            else
                {
                hr_str[i] = 0;
                }
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
            offset += hr_len;
            }
        // if minutes and hours are too long to fit next to each other overlay
        // hours in red, minutes in green
        else if((min_len + hr_len) > DIGIT_COUNT || s_settings[SETTING_OVERLAP] )
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
            if( min_len > hr_len )
                {
                offset = min_len;
                }
            else
                {
                offset = hr_len;
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
            offset += hr_len;
            }

        if( s_settings[SETTING_BASE_DISP] && offset < (DIGIT_COUNT-2) )
            {
            temp_len = MTH_convert_to_base(abs(base), 10, temp_str, DIGIT_COUNT);
            if(base < 0)
                {
                disp_ptr = disp_red;
                }
            else
                {
                disp_ptr = disp_green;
                }
                
            disp_ptr += (DIGIT_COUNT-2);

            for(i = 0; i < temp_len; i++)
                {
                *disp_ptr = numbers[temp_str[i]];
                disp_ptr++;
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
                pwm_setting = PWM_RED;
                if( overlap )
                    {
                    dp_green = 0;
                    dp_start = sec_len;
                    }
                else
                    {
                    dp_start = sec_len + min_len;
                    }
                dp_end = dp_start + hr_len;
                dp_fill = true;
                break;

            /*---------------------------------------
            Highlight the minutes with decimal points
            while setting, use yellow if the minutes
            are separate, otherwise use the minute
            color (green)
            ---------------------------------------*/
            case DISPLAY_SET_MIN:
                pwm_setting = PWM_GREEN;
                if( overlap )
                    {
                    dp_red = 0;
                    }
                dp_start = sec_len;
                dp_end = dp_start + min_len;
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
                    dp_end = dp_start + min_len;
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
            if( dp_start || s_settings[SETTING_SEC_DISP] )
                {
                disp_green[dp_start] |= dp_green;
                disp_red[dp_start] |= dp_red;
                }
            if( dp_end || s_settings[SETTING_SEC_DISP] )
                {
                disp_green[dp_end] |= dp_green;
                disp_red[dp_end] |= dp_red;
                }            
            }
        break;
        
        case DISPLAY_SETTINGS:
            format_menu_string( s_setting_strings[v_settings_state] );
            if( v_settings_state != SETTING_MAIN_MENU
             && v_settings_state != SETTING_ADVANCED )
                {
                display_setting( s_settings[v_settings_state] );
                }
            break;
            
        case DISPLAY_COLOR_BAL:
            format_menu_string( "YELLOW" );
            temp_len = MTH_convert_to_base_unsigned(s_yellow_point, 16, temp_str, DIGIT_COUNT);
            for(i = 0; i < temp_len; i++)
                {
                disp_red[i] = numbers[temp_str[i]];
                disp_green[i] = numbers[temp_str[i]];
                }
            break;
        
        case DISPLAY_COLOR:
            format_menu_string( "COLOR" );
            format_setting_string( color_strings[s_color_setting] );
            break;
            
        case DISPLAY_RAND_BASE:
            format_menu_string( "RANDBASE" );
            format_setting_num( s_rand_base_interval );
            break;
        
        default:
            format_menu_string( "ERROR 1776" );
            break;
    }

OCR1A = pwm_setting;
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


static void check_rand_base( CLK_time_type time )
{
static uint8_t  old_min;
uint16_t        temp;

if( s_rand_base_interval && v_disp_state == DISPLAY_TIME )
    {
    if( old_min != time.minute )
        {
        temp = time.minute + time.hour * 60;
        if( temp % s_rand_base_interval == 0 )
            {
            do
                {
                s_base = rand() % 120 - 60;
                } while( s_base < 2 && s_base > -2 );
            }
        }
    }
old_min = time.minute;
}

static void process_knob( CLK_time_type time )
{
ENC_dir     dir;

dir = ENC_read();
if( dir == ENC_NONE )
    {
    return;
    }
else if( !s_rand_init )
    {
    srand( s_rand_seed );
    s_rand_init = true;
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

    case DISPLAY_SETTINGS:
        v_settings_state = ( v_settings_state + SETTING_COUNT + dir ) % SETTING_COUNT;
        v_disp_timer = DISPLAY_TIMEOUT;
        break;
    
    case DISPLAY_COLOR_BAL:
        s_yellow_point = s_yellow_point + dir;
        if( s_yellow_point > PWM_TOP )
            {
            if( dir > 0 )
                {
                s_yellow_point = PWM_TOP;
                }
            else
                {
                s_yellow_point = 0;
                }
            }
        v_disp_timer = DISPLAY_TIMEOUT;
        break;
        
    case DISPLAY_COLOR:
        s_color_setting = ( s_color_setting + COLOR_COUNT + dir ) % COLOR_COUNT;
        v_disp_timer = DISPLAY_TIMEOUT;
        break;
        
    case DISPLAY_RAND_BASE:
        s_rand_base_interval = ( s_rand_base_interval + RAND_BASE_MAX + dir) % RAND_BASE_MAX;
        v_disp_timer = DISPLAY_TIMEOUT;
        break;
        
    default:
        format_menu_string( "Error 2718" );
        break;
    }
}

static void set_knob_color( CLK_time_type time )
{
static bool     update;

switch( s_color_setting )
    {
    case COLOR_OFF:
        ENC_clear_color();
        break;
    
    case COLOR_SEC:
        ENC_set_color( time.second * 4 );
        break;
    
    case COLOR_MIN:
        ENC_set_color( time.minute * 4 + time.second / 15 );
        break;
    
    case COLOR_HR:
        if( s_settings[SETTING_12_HOUR] )
            {
            ENC_set_color( time.hour * 20 + time.minute / 3 );
            }
        else
            {
            ENC_set_color( time.hour * 10 + time.minute / 6 );
            }
        break;
    
    case COLOR_BASE:
        ENC_set_color( ( s_base + 60 ) * 2 );
        break;
    
    case COLOR_RAND:
        if( time.second % 5 == 0 )
            {
            if( update )
                {
                    ENC_set_color( rand() & 0xFF );
                }
            update = false;
            }
        else
            {
            update = true;
            }
        break;
        
    case COLOR_FADE:
        ENC_set_color( v_ms >> 5 );
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

ENC_init();
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
    //Use the time from boot to the first user knob turn to seed the RNG
    if( !s_rand_init )
        {
        s_rand_seed++;
        }

    time = CLK_time_get();
    
    check_rand_base( time );
    process_knob( time );
    format_display(s_base, time);
    set_knob_color( time );

    if(v_disp_update_flag == true)
        {
        v_disp_update_flag = false;
        display();
        }
    }
}