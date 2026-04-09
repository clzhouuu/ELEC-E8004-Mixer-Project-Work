#ifndef CREATE3_H
#define CREATE3_H

#include <stdint.h>
#include "message.h"

#define CREATE3_START_A 0xCAu
#define CREATE3_START_B 0xFEu
#define CREATE3_MSG_ODOM 0x01u
#define CREATE3_MAX_PAYLOAD 64u

typedef struct __attribute__((packed)) {
    float x;
    float y;
    float theta;
    float v;
    float w;
} Create3Payload_t;

void create3_init(uint8_t robot_id);
void create3_poll(uint32_t now_ms);
uint8_t create3_get_pose(RobotPoseMsg_t *out_pose);

#endif