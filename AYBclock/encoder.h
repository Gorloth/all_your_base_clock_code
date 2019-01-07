/*
* clock.h
*/

#include <stdint.h>

typedef int8_t ENC_dir;
enum
    {
    ENC_CW = -1,
    ENC_CCW = 1,
    ENC_NONE = 0
    };
    
void ENC_init( void );

ENC_dir ENC_read( void );

void ENC_set_color( uint8_t hue );

void ENC_clear_color( void );