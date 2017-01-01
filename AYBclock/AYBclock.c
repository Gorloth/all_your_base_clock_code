/*
 * AYBclock.c
 *
 * Created: 2/10/2016 5:27:51 PM
 *  Author: Kirby
 */ 
#define F_CPU 16000000

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>

#define TRUE 1
#define FALSE 0

#define DIGIT_COUNT 10

#define PWM_TOP 0x0010
#define PWM_TOGGLE ( PWM_TOP * .40 )

#define A (1<<0)
#define B (1<<1)
#define C (1<<2)
#define D (1<<3)
#define E (1<<4)
#define F (1<<5)
#define G (1<<6)
#define DP (1<<7)

typedef struct{
    int8_t    hour;
    int8_t    minute;
    } time_type;

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
    (A|C||D|E|F),    //G
    (B|C|E|F|G),     //H
    (A|B|C|D),       //I
    (B|C|D),         //J
    (0),             //NONE
};

static uint8_t convert_to_base(
    int8_t      number,
    int8_t      base,
    int8_t     *output,
    uint8_t     out_sz
    );

static uint8_t max_width( int8_t base );
static void format_time( int8_t base, time_type time );
static time_type get_time( void );
static void init_spi(void);
static uint8_t spi( uint8_t data );
static void init_timers( void );
static void display( void );

static volatile uint16_t  s_ms;
static volatile uint16_t time;

static volatile uint8_t  disp_red[DIGIT_COUNT];
static volatile uint8_t  disp_green[DIGIT_COUNT];

static void init_spi(){
    SPCR = 1<<SPE | 1<<MSTR;
    SPSR = 1<<SPI2X;
}

static uint8_t spi( uint8_t data ){
    SPDR = data;
    while(bit_is_clear(SPSR,SPIF)) {}
    return( SPDR );
}

static void init_timers(){
    //PB1 output
    //      clear PB1 on compare match,  Fast PWM mode
    TCCR1A = 1<<COM1A1                   | 0<<WGM11 | 0<<WGM10;
    //       clock / 1024
    TCCR1B = 1<<CS12 | 0<<CS11 | 1<<CS10 | 1<<WGM13 | 0<<WGM12;

    //Top 
    ICR1 = PWM_TOP;

    //Toggle at
    OCR1A = PWM_TOGGLE;
    
    
    OCR2A = 250-1;
    TCCR2A = 1<<COM2A1 | 1<<WGM21 ;
    TCCR2B = 1<<CS22;
    TIMSK2 = 1<<OCIE2A;
}

ISR(TIMER2_COMPA_vect){
    static uint16_t count;
    count++;
    s_ms++;
    if( count == 1000 ){
        count = 0;
        time++;
        if( time == 10000 ){
            time = 0;
        }
    }
}

static time_type get_time( void )
{
    time_type time;
    time.hour = 0;
    time.minute = 0;
    
    return( time );
}

static void format_time( 
    int8_t base, 
    time_type time 
)
{
int8_t      min_str[DIGIT_COUNT];
int8_t      hr_str[DIGIT_COUNT];

uint8_t     i;

uint8_t     min_len;
uint8_t     min_max_len;
uint8_t     hr_len;

min_len = convert_to_base( time.minute, base, min_str, DIGIT_COUNT);
min_max_len = max_width( base );
for( i = min_len; i < min_max_len; i++ )
    {
    min_str[i] = 0;
    }
min_len = min_max_len;
hr_len = convert_to_base( time.hour, base, hr_str, DIGIT_COUNT);

for( i = 0; i < DIGIT_COUNT; i++ )
    {
    disp_green[i] = 0;
    }
    
for( i = 0; i < DIGIT_COUNT; i++ )
    {
    disp_red[i] = 0;
    }

// |Base| > 20 needs to use red/green for the digits, thus can't overlay
if( base > 20 || base < -20 )
    {
    for( i = 0; i < min_len; i++ )
        {
        if( min_str[i] >= 40 )
            {
            disp_red[i] = numbers[min_str[i]-40];
            }
        else if( min_str[i] >= 40 )
            {
            disp_green[i] = numbers[min_str[i]-20];
            disp_red[i] = numbers[min_str[i]-20];
            }
        else
            {
            disp_green[i] = numbers[min_str[i]];
            }
        }
    for( i = i; i < hr_len + min_len; i++ )
        {
        if( hr_str[i-min_len] >= 40 )
            {
            disp_red[i] = numbers[hr_str[i-min_len]-40];
            }
        else if( hr_str[i-min_len] >= 20 )
            {
            disp_green[i] = numbers[hr_str[i-min_len]-20];
            disp_red[i] = numbers[hr_str[i-min_len]-20];
            }
        else
            {
            disp_green[i] = numbers[hr_str[i-min_len]];
            }
        }
    }
// if minutes and hours are too long to fit next to each other overlay
// hours in red, minutes in green
else if( ( min_len + hr_len ) > DIGIT_COUNT )
    {
    for( i = 0; i < min_len; i++)
        {
        disp_green[i] = numbers[min_str[i]];
        }
    for( i = 0; i < hr_len; i++)
        {
        disp_red[i] = numbers[hr_str[i]];
        }
    }
else
    {
    for( i = 0; i < min_len; i++)
        {
        disp_green[i] = numbers[min_str[i]];
        disp_red[i] = numbers[min_str[i]];
        }
    for( i = 0; i < hr_len; i++)
        {
        disp_green[i+min_len] = numbers[hr_str[i]];
        disp_red[i+min_len] = numbers[hr_str[i]];
        }
    disp_green[min_len] |= DP;
    disp_red[min_len] |= DP;
    }
}

static void display( void )
{
int8_t      i;

for( i = DIGIT_COUNT - 1; i >= 0; i-- )
    {
    spi( disp_green[i] );
    }
PORTC = PORTC & ~(1<<PORTC1);
PORTC = PORTC | (1<<PORTC1);

for( i = DIGIT_COUNT - 1; i >= 0; i-- )
    {
    spi( disp_red[i] );
    }
PORTC = PORTC & ~(1<<PORTC0);
PORTC = PORTC | (1<<PORTC0);
}

static uint8_t max_width(
    int8_t      base
    )
{
    uint8_t     width;
    if( base > 0 ){
        switch( base ){
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
                
            default:
                width = 2;
                break;
        }
    }
    else{
        switch( base ){
            case -2:
                width = 7;
                break;
            
            case -3:
            case -4:
                width = 5;
                break;

            default:
                width = 3;
                break;
        }
    }
    
return( width );
}

static uint8_t convert_to_base(
    int8_t      number,
    int8_t      base,
    int8_t     *output,
    uint8_t     out_sz
)
{
uint8_t         i;

i = 0;

for(i=0; i < out_sz && number; i++ )
    {
    *output = number % base;
    number = number / base;    
    if( *output < 0 )
        {
        *output += abs(base);
        number++;
        }
    output++;
    }

return( i );
}

int main(void)
{
    time_type   time;
    int8_t      base;
    uint8_t     i;
    
    DDRB = 0xFF;
    DDRC = (1<<PORTC0)|(1<<PORTC1);
    
    PORTC = (1<<PORTC0)|(1<<PORTC1);
    
    init_timers();
    init_spi();

    s_ms = 0;
    time.hour = 12;
    time.minute = 59;
    base = 10;

    //sei();
    
    for( i = 0; i < DIGIT_COUNT; i++ )
        {
        //disp_red[i] = numbers[2];
        //disp_green[i] = numbers[7];
        }

    while(1){
        //time = get_time();
        format_time( base, time );
        display();
        while(1){
            time.minute++;
            if( time.minute == 60 )
            {
                time.minute = 0;
                time.hour = (time.hour+1) % 24;
            }
            format_time( base, time );
            display();
            _delay_ms(250);
            _delay_ms(250);
            _delay_ms(250);
            _delay_ms(250);
        }
        
    }
}