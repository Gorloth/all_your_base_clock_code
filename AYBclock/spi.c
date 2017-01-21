/*
* spi.c
*/

#include <avr/io.h>
#include <stdlib.h>

void spi_init(void)
{
SPCR = 1 << SPE | 1 << MSTR | 1 << CPOL;
SPSR = 1 << SPI2X;
}

uint8_t spi
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
