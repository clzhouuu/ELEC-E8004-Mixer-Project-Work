#ifndef MESSAGE_H
#define MESSAGE_H

#include <stdint.h>

// fixed point math for Q16.16 
#define FP_SCALE (65536L)
#define FP_FROM_FLOAT(x) ((int32_t)((x) * FP_SCALE))
#define FP_TO_FLOAT(x) ((float)(x)  / FP_SCALE)
#define FP_MUL(a, b) ((int32_t)(((int64_t)(a) * (b)) >> 16))
#define FP_DIV(a, b) ((int32_t)(((int64_t)(a) << 16) / (b)))

#define BOLT_MAX_PAYLOAD 32u

// Structure of messages
typedef struct {
    uint32_t timestamp_ms;
    int32_t x_fp; // x coordinate
    int32_t y_fp; // y coordinate
    int32_t theta_fp; // orientation
    int32_t v_fp; // linear velocity
    uint8_t robot_id; // 0, 1, or 2
    uint8_t status; // see POSE_STATUS_* flags
} __attribute__((packed)) RobotPoseMsg_t;

// status flags for RobotPoseMsg
#define POSE_STATUS_VALID (1u << 0)   // data is fresh
#define POSE_STATUS_STALE (1u << 1)   // no update for over 500ms
#define POSE_STATUS_INITIALISED (1u << 2)   // first message has been received

/* Velocity correction message from AP to CP
typedef struct {
    uint32_t timestamp_ms;  // time
    int32_t delta_v_fp;   // speed correction (m/s)
    int32_t delta_w_fp;   // turning correction (rad/s)
    uint8_t robot_id;      // 0, 1, or 2
    uint8_t flags;         
} __attribute__((packed)) VelCorrectionMsg_t;

#define VEL_CORR_VALID (1u << 0)            // correction is ready to use
#define VEL_CORR_STOP (1u << 1)             // emergency stop
#define VEL_CORR_PEERS_MISSING (1u << 3)   // haven't heard from robots

*/

#define NUM_ROBOTS      3
#define MESSAGE_SIZE    sizeof(RobotPoseMsg_t)

#endif 
