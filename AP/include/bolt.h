#ifndef BOLT_H
#define BOLT_H

#include <stdint.h>

// initialization
uint8_t bolt_init(void);

// writing from APP to CP
uint8_t bolt_write(uint8_t* data, uint8_t len);

// reading from CP to APP
uint8_t bolt_read(uint8_t* buf, uint8_t* len);

// check if BOLT has data waiting
uint8_t bolt_data_available(void);

#endif 
