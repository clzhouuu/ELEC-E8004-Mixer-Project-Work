#ifndef BOLT_H
#define BOLT_H

#include <stdint.h>
#include "message.h"

// initialization
uint8_t bolt_init(void);

// check if BOLT has data waiting
uint8_t bolt_data_available(void);

// send a framed message to CP 
uint8_t bolt_write(uint8_t* data, uint16_t len);

// read a framed message from CP
uint8_t bolt_read(uint8_t* buf, uint8_t* len);

#endif 