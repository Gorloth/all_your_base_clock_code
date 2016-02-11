/*
 * AYBclock.c
 *
 * Created: 2/10/2016 5:27:51 PM
 *  Author: Kirby
 */ 
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define TRUE 1
#define FALSE 0

#define A (1<<5)
#define B (1<<3)
#define C (1<<6)
#define D (1<<2)
#define E (1<<0)
#define F (1<<7)
#define G (1<<4)
#define DP (1<<1)

#define WAKE_UP_DELAY   0
#define NOT_ARMED       1
#define WAITING         2
#define START_DELAY     3
#define START           4
#define RACING          5
#define FIRST_FIN       6
#define SECOND_FIN      7

#define TILT_THRESHOLD 0xD0
#define START_DELAY_TIME    1000
#define WAKE_UP_DELAY_TIME  1000
#define ADC_THRESHOLD       0x50

#define LEFT 1
#define RIGHT 2

static const uint8_t numbers[] =
{
    0xFF - (A|B|C|D|E|F),    //0
    0xFF - (B|C),            //1
    0xFF - (A|B|G|E|D),      //2
    0xFF - (A|B|G|C|D),      //3
    0xFF - (F|G|B|C),        //4
    0xFF - (A|F|G|C|D),      //5
    0xFF - (A|F|G|C|E|D),    //6
    0xFF - (A|B|C),          //7
    0xFF - (A|B|C|D|E|F|G),  //8
    0xFF - (A|B|C|D|F|G),    //9
    0xFF - (A|B|C|E|F|G),    //A
    0xFF - (F|G|C|D|E),      //B
    0xFF - (G|E|D),          //C
    0xFF - (E|G|B|C|D),      //D
    0xFF - (A|F|G|E|D),      //E
    0xFF - (A|F|G|E),        //F
    0xFF,                    //NONE
};

static volatile uint16_t  s_ms;
static volatile uint16_t time;

void init_spi(){
    SPCR = 1<<SPE | 1<<MSTR;
    SPSR = 1<<SPI2X;
}

uint8_t spi( uint8_t data ){
    SPDR = data;
    while(bit_is_clear(SPSR,SPIF)) {} //grab encoder data
    return( SPDR );
}

void spi_write( uint8_t addr, uint8_t data ){
    PORTB &= ~0x04;
    spi(0x0A); //WRITE
    spi(addr);
    spi(data);
    PORTB |= 0x04;
}

uint8_t spi_read( uint8_t addr ){
    uint8_t data;
    PORTB &= ~0x04;
    spi(0x0B); //READ
    spi(addr);
    data = spi(0xFF);
    PORTB |= 0x04;
    return( data );
}

//Timer1 Prescaler = 0; Preload = 15999; Actual Interrupt Time = 1 ms
void init_timers(){
    //PB1 output
    TCCR1A = 1<<COM1A1 | 1<<WGM11 | 0<<WGM10;
    TCCR1B = 1<<CS11 | 1<<WGM13 | 1<<WGM12;

    //OCR1AH = 0x3E;
    //OCR1AL = 0x7F;

    ICR1H = 0x9C;
    ICR1L = 0x3F;

    OCR1AH = 0x9C;
    OCR1AL = 0x3F;

    OCR1A = 3000-1;

    OCR2A = 250-1;
    TCCR2A = 1<<COM2A1 | 1<<WGM21 ;
    TCCR2B = 1<<CS22;

    TIMSK2 = 1<<OCIE2A;
}

ISR(TIMER2_COMPA_vect){
    static uint8_t count;
    count++;
    s_ms++;
    if( count == 10 ){
        count = 0;
        time++;
        if( time == 10000 ){
            time = 0;
        }
    }
}

int main(void)
{
    init_timers();
    init_spi();

    s_ms = 0;
    time = 0;

    sei();
    
    while(1){
    }
}