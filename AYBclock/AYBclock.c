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

#define TRUE 1
#define FALSE 0

//define the codes for actions to occur
#define TWCR_START      ((1<<TWINT)|(1<<TWSTA)|(1<<TWEN)) //send start condition
#define TWCR_STOP       ((1<<TWINT)|(1<<TWSTO)|(1<<TWEN)) //send stop condition
#define TWCR_RACK       ((1<<TWINT)|(1<<TWEN)|(1<<TWEA)) //receive byte and return ack to slave
#define TWCR_RNACK      ((1<<TWINT)|(1<<TWEN)) //receive byte and return nack to slave
#define TWCR_SEND       ((1<<TWINT)|(1<<TWEN)) //pokes the TWINT flag in TWCR and TWEN

#define TWI_PINS        (1<<PORTC4|1<<PORTC5)

#define CLOCK_ADDR_READ     0xDF
#define CLOCK_ADDR_WRITE    0xDE

#define DIGIT_COUNT 10

#define DISPLAY_TIMEOUT       ( 1000 * 5 )
#define QUICK_DISPLAY_TIMEOUT ( 1000 * 2 )

#define DISPLAY_RATE 50
#define DEBOUNCE_RATE 10

#define PWM_TOP 0x0010
#define PWM_TOGGLE ( PWM_TOP * .3f )
#define PWM_RED    ( PWM_TOP * .9f )
#define PWM_GREEN  ( PWM_TOP * .1f )

#define GREEN_EN_PIN    (1<<PORTC1)
#define RED_EN_PIN      (1<<PORTC2)

#define BUTTON_IN_PIN   (1<<PORTC0)

#define CLOCK_IN_PIN    (1<<PORTD7)

#define A (1<<0)
#define B (1<<1)
#define C (1<<2)
#define D (1<<3)
#define E (1<<4)
#define F (1<<5)
#define G (1<<6)
#define DP (1<<7)

typedef struct{
    int8_t  hour;
    int8_t  minute;
    int8_t  second;
    } time_type;

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

static const uint8_t numbers[] =
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
    (0),             //NONE
    };

static const uint8_t letters[] =
    {
    (A|B|C|E|F|G),  //A
    (F|G|C|D|E),    //B
    (G|E|D),        //C
    (E|G|B|C|D),    //D
    (A|F|G|E|D),    //E
    (A|F|G|E),      //F
    (A|C|D|E|F),    //G
    (B|C|E|F|G),    //H
    (A|B|C|D),      //I
    (B|C|D),        //J
    (B|C|E|F|G),    //K
    (D|E|F),        //L
    (A|C|E|G),      //M
    (C|E|G),        //N
    (A|B|C|D|E|F),  //O
    (A|B|E|F|G),    //P
    (A|B|C|F|G),    //Q
    (E|G),          //R
    (A|C|D|F|G),    //S
    (D|E|F|G),      //T
    (B|C|D|E|F),    //U
    (C|D|E),        //V
    (B|D|F),        //W
    (B|C|E|F|G),    //X
    (B|C|D|F|G),    //Y
    (A|B|D|E|G)     //Z
    };

static uint8_t convert_to_base(
    int8_t      number,
    int8_t      base,
    int8_t     *output,
    uint8_t     out_sz
    );

static uint8_t max_width(int8_t base);
static void format_display(int8_t base, time_type time);
static time_type get_time(void);
static void init_spi(void);
static uint8_t spi(uint8_t data);
static void init_timers(void);
static void display(void);
static encoder_dir encoder_read(void);
static void init_twi(void);
static uint8_t TWI_start(void);
static uint8_t TWI_write(uint8_t data);
static uint8_t TWI_read(bool ack);
static void TWI_stop(void);
static void init_clock(void);
static uint8_t bcd_to_int(uint8_t data);
static uint8_t int_to_bcd(uint8_t data);
static void format_string( const char *string );
static void set_time( time_type time );
static void process_knob( time_type time );

static volatile uint16_t s_ms;
static volatile uint16_t s_time;

static volatile uint8_t  disp_red[DIGIT_COUNT];
static volatile uint8_t  disp_green[DIGIT_COUNT];

static volatile uint8_t s_disp_update_flag;

static volatile display_state s_disp_state;

static volatile uint16_t s_disp_timer;

static int8_t            s_base;

//******************************************************************************
//                          init_twi
//initialize TWI registers
//
static void init_twi(void)
{
TWBR = 80;
}

static uint8_t TWI_start(void)
{
TWCR = TWCR_START; //send start condition
while(!(TWCR & (1 << TWINT)))
    {
    } //wait for start condition to transmit
if((TW_STATUS != TW_START) & (TW_STATUS != TW_REP_START))
    {
    return(1);
    }
return(0); //return success value
}

//******************************************************************************
static uint8_t TWI_write
    (
    uint8_t data
    )
{
TWDR = data; //send device addr, write bit set
TWCR = TWCR_SEND; //poke TWINT to send address
while(!(TWCR & (1 << TWINT)))
    {
    } //wait for LM73 address to go out
if(TW_STATUS != TW_MT_SLA_ACK)
    {
    return(1);
    }//check status reg for SLA+W ACK
return(0); //return success value
}

//******************************************************************************
static uint8_t TWI_read
    (
    bool ack
    )
{
if(ack)
    {
    TWCR = TWCR_RACK; //poke TWINT to send address
    }
else
    {
    TWCR = TWCR_RNACK; //poke TWINT to send address
    }
while(!(TWCR & (1 << TWINT)))
    {
    } //wait for LM73 address to go out
return(TWDR); //return success value
}

//******************************************************************************
static void TWI_stop(void)
{
TWCR = TWCR_STOP; //finish transaction
}

static void init_spi(void)
{
SPCR = 1 << SPE | 1 << MSTR | 1 << CPOL;
SPSR = 1 << SPI2X;
}

static uint8_t spi
    (
    uint8_t data
    )
{
SPDR = data;
while(bit_is_clear(SPSR, SPIF))
    {
    }
return(SPDR);
}

static uint8_t bcd_to_int(uint8_t data)
{
return((data & 0x0F) + ((data & 0xF0) >> 4) * 10);
}

static uint8_t int_to_bcd(uint8_t data)
{
return( ( data % 10 ) | ( ( data / 10 ) << 4 ) );
}

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

static void init_clock(void)
{
//Start Clock
TWI_start();
TWI_write(CLOCK_ADDR_WRITE);
TWI_write(0x00);
TWI_write(0x80);
TWI_stop();

//Enable 1Hz outptut
TWI_start();
TWI_write(CLOCK_ADDR_WRITE);
TWI_write(0x07);
TWI_write((1 << 6));
TWI_stop();
}

ISR(TIMER2_COMPA_vect)
{
static uint16_t count;
static uint16_t disp_count;
static uint8_t  btn_count;
static uint8_t  btn_data;

count++;
s_ms++;
btn_count++;
if(btn_count == DEBOUNCE_RATE)
    {
    btn_count = 0;
    btn_data = btn_data << 1;
    btn_data |= (PINC & BUTTON_IN_PIN);
    if(btn_data == 0xF0)
        {
        s_disp_state = (s_disp_state + 1) % DISPLAY_COUNT;
        s_disp_timer = DISPLAY_TIMEOUT;
        }
    }
if(s_disp_timer)
    {
    s_disp_timer--;
    if(!s_disp_timer)
        {
        s_disp_state = DISPLAY_TIME;
        OCR1A = PWM_TOGGLE;
        }
    }
if(disp_count++ == DISPLAY_RATE)
    {
    s_disp_update_flag = TRUE;
    disp_count = 0;
    }
if(count == 1000)
    {
    count = 0;
    s_time++;
    if(s_time == 10000)
        {
        s_time = 0;
        }
    }
}

static time_type get_time(void)
{
uint8_t     second;
uint8_t     minute;
uint8_t     hour;
time_type   time;

time.hour = 0;
time.minute = 0;

TWI_start();
TWI_write(CLOCK_ADDR_WRITE);
TWI_write(0x00);
TWI_start();
TWI_write(CLOCK_ADDR_READ);
second = TWI_read(TRUE);
minute = TWI_read(TRUE);
hour = TWI_read(FALSE);
TWI_stop();

time.second = bcd_to_int(second & 0x7f);
time.minute = bcd_to_int(minute & 0x7F);
time.hour = bcd_to_int(hour & 0x3f);

return(time);
}

static void set_time( time_type time )
{
TWI_start();
TWI_write(CLOCK_ADDR_WRITE);
TWI_write(0x01);
TWI_write(int_to_bcd(time.minute));
TWI_write(int_to_bcd(time.hour));
TWI_stop();
}

//******************************************************************************
//                          encoder_read
//reads the value from the encoders using SPI and calculates the direction
//of turning by saving the previous 4 states of the encoders, states are only
//updated if the encoders value has changes from the last check
static encoder_dir encoder_read( void )
{
static uint16_t     encoder;

uint16_t            data_in;
encoder_dir         dir;

data_in = ((PIND & (1 << PORTD2 | 1 << PORTD3)) >> 2) & 0x03;

dir = ENCODER_NONE;

if((0x03 & encoder) != (data_in & 0x03))
    {
    encoder = (encoder << 2) | data_in;
    //only look at the last 10 bits of the past states
    //0b0010110100 is counter clockwise
    if((0x03FF & encoder) == 0b0010110100)
        {
        dir = ENCODER_CW;
        }
    //0b0001111000 is clockwise
    else if((0x03FF & encoder) == 0b0001111000)
        {
        dir = ENCODER_CCW;
        }
    }
return(dir);
}

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

static void format_display
    (
    int8_t base,
    time_type time
    )
{
int8_t      sec_str[DIGIT_COUNT];
int8_t      min_str[DIGIT_COUNT];
int8_t      hr_str[DIGIT_COUNT];

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

if(s_disp_state == DISPLAY_BASE)
    {
    format_string( "BASE");

    min_len = convert_to_base(abs(base), 10, min_str, DIGIT_COUNT);
    for(i = 0; i < min_len; i++)
        {
        if(base < 0)
            {
            disp_red[i] = numbers[min_str[i]];
            }
        else
            {
            disp_green[i] = numbers[min_str[i]];
            }
        }
    if(base < 0)
        {
        disp_red[i] = G;
        }
    }
else
    {
    overlap = FALSE;
    dp_blink = ((PIND & CLOCK_IN_PIN) == CLOCK_IN_PIN);
    max_len = max_width(base);

    sec_len = convert_to_base(time.second, base, sec_str, DIGIT_COUNT);
    for(i = sec_len; i < max_len; i++)
    {
        sec_str[i] = 0;
    }
    sec_len = max_len;

    min_len = convert_to_base(time.minute, base, min_str, DIGIT_COUNT);
    for(i = min_len; i < max_len; i++)
        {
        min_str[i] = 0;
        }
    min_len = max_len;
    hr_len = convert_to_base(time.hour, base, hr_str, DIGIT_COUNT);
    if( hr_len == 0 )
        {
        hr_str[0] = 0;
        hr_len++;
        }

    // |Base| > 20 needs to use red/green for the digits, thus can't overlay
    if(base > 20 || base < -20)
        {
        dp_blink = TRUE;
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
        overlap = TRUE;
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
            dp_blink = TRUE;
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

    dp_green = DP;
    dp_red = DP;

    switch( s_disp_state )
        {
        case DISPLAY_SET_HOUR:
            if( overlap )
                {
                dp_green = 0;
                dp_start = sec_len;
                dp_end = sec_len + hr_len;
                dp_fill = TRUE;
                }
            else
                {
                dp_start = sec_len + min_len;
                dp_end = sec_len + min_len + hr_len;
                dp_fill = TRUE;
                }                
            break;

        case DISPLAY_SET_MIN:
            if( overlap )
                {
                dp_red = 0;
                }
            dp_start = sec_len;
            dp_end = sec_len + min_len;
            dp_fill = TRUE;
            break;

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
            dp_fill = FALSE;
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

PORTB |= (1 << PORTB0);

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

PORTB &= ~(1 << PORTB0);
}

static uint8_t max_width
    (
    int8_t      base
    )
{
uint8_t     width;
switch(base)
    {
    case -2:
        width = 7;
        break;

    case -3:
    case -4:
        width = 5;
        break;
            
    case 2:
        width = 6;
        break;

    case 3:
        width = 4;
        break;

    case 4:
    case 5:
    case 6:
    case 7:
        width = 3;
        break;

    case -60:
    case 60:
        width = 1;
        break;

    default:
        if( base > 0 )
            {
            width = 2;
            }
        else
            {
            width = 3;
            }
        break;
    }

return(width);
}

static uint8_t convert_to_base
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

static void process_knob( time_type time )
{
encoder_dir     dir;

dir = encoder_read();
if( dir == ENCODER_NONE )
    {
    return;
    }

switch( s_disp_state )
    {
    case DISPLAY_TIME:
        s_disp_state = DISPLAY_BASE;
        s_disp_update_flag = TRUE;
        s_disp_timer = QUICK_DISPLAY_TIMEOUT;
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
        s_disp_update_flag = TRUE;
        s_disp_timer = QUICK_DISPLAY_TIMEOUT;
        break;

    case DISPLAY_SET_HOUR:
        time.hour = ( time.hour + 24 + dir) % 24;
        s_disp_update_flag = TRUE;
        set_time( time );
        s_disp_timer = DISPLAY_TIMEOUT;
        break;

    case DISPLAY_SET_MIN:
        time.minute = ( time.minute + 60 + dir) % 60;
        s_disp_update_flag = TRUE;
        set_time( time );
        s_disp_timer = DISPLAY_TIMEOUT;
        break;

    default:
        break;
    }
}

int main(void)
{
time_type   time;

DDRB = 0xFF;
DDRC = GREEN_EN_PIN | RED_EN_PIN;
DDRD = (0 << PORTD2) | (0 << PORTD3);
PORTD = (1 << PORTD2) | (1 << PORTD3) | CLOCK_IN_PIN;
PORTC = GREEN_EN_PIN | RED_EN_PIN | TWI_PINS | BUTTON_IN_PIN;

init_timers();
init_spi();
init_twi();

s_ms = 0;
time.hour = 12;
time.minute = 59;
s_base = 10;

sei();

init_clock();

while(1)
    {
    time = get_time();
    process_knob( time );
    format_display(s_base, time);

    if(s_disp_update_flag == TRUE)
        {
        s_disp_update_flag = FALSE;
        display();
        }

    }
}