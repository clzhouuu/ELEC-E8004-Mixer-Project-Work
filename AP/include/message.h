
#ifndef MESSAGE_H
#define MESSAGE_H

#include <stdint.h>

// Structure of messages:
// Information  Detail                      Bytes
// Identity     node ID                     1
// Position     (x, y, angle)               4, 4, 4
// Velocity     (velocity, turning rate)    4, 4
// Time         data stamp                  4

typedef struct {
    uint8_t  robot_id; 
    float    x;      
    float    y;     
    float    ang;    
    float    v;          
    float    vang;            
    uint32_t timestamp_ms; 
} __attribute__((packed)) robot_state_t;

#define NUM_ROBOTS      3
#define MESSAGE_SIZE    sizeof(robot_state_t)

#endif 