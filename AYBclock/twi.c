/*
* twi.c
*/

#include <avr/io.h>
#include <util/twi.h>
#include <stdlib.h>

#include "twi.h"

//define the codes for actions to occur
#define TWCR_START      ((1<<TWINT)|(1<<TWSTA)|(1<<TWEN)) //send start condition
#define TWCR_STOP       ((1<<TWINT)|(1<<TWSTO)|(1<<TWEN)) //send stop condition
#define TWCR_RACK       ((1<<TWINT)|(1<<TWEN)|(1<<TWEA)) //receive byte and return ack to slave
#define TWCR_RNACK      ((1<<TWINT)|(1<<TWEN)) //receive byte and return nack to slave
#define TWCR_SEND       ((1<<TWINT)|(1<<TWEN)) //pokes the TWINT flag in TWCR and TWEN

#define TWI_PINS        (1<<PORTC4|1<<PORTC5)

//******************************************************************************
//                          init_twi
//initialize TWI registers
//
void TWI_init(void)
{
TWBR = 80;
PORTC = TWI_PINS;
}

uint8_t TWI_start(void)
{
TWCR = TWCR_START; //send start condition
while(!(TWCR & (1 << TWINT))){} //wait for start condition to transmit

return((TW_STATUS != TW_START) & (TW_STATUS != TW_REP_START));
}

//******************************************************************************
uint8_t TWI_write
    (
    uint8_t data
    )
{
TWDR = data; //send data, write bit set
TWCR = TWCR_SEND; //poke TWINT to send data

while(!(TWCR & (1 << TWINT))){}

return(TW_STATUS != TW_MT_SLA_ACK);//check status reg for SLA+W ACK
}

//******************************************************************************
uint8_t TWI_read
    (
    bool ack
    )
{
if(ack)
    {
    TWCR = TWCR_RACK;
    }
else
    {
    TWCR = TWCR_RNACK;
    }

while(!(TWCR & (1 << TWINT))){}

return(TWDR); //return success value
}

//******************************************************************************
void TWI_stop(void)
{
TWCR = TWCR_STOP; //finish transaction
}
