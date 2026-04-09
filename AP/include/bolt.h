#ifndef BOLT_H
#define BOLT_H

#include <stdint.h>
#include "message.h"

// initialization
uint8_t bolt_init(void);

// check if BOLT has data waiting
uint8_t bolt_data_available(void);

// send a framed message to CP 
uint8_t bolt_send(uint8_t channel, const void* payload, uint8_t length);

// read a framed message from CP
uint8_t bolt_recv(uint8_t* channel_out, uint8_t* buf, uint8_t* len_out);

#endif 