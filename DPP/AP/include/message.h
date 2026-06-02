#ifndef MESSAGE_H
#define MESSAGE_H

#include <stdint.h>
#include <stddef.h>

// fixed point math for Q16.16
#define FP_SCALE (65536L)
#define FP_FROM_FLOAT(x) ((int32_t)((x) * FP_SCALE))
#define FP_TO_FLOAT(x) ((float)(x) / FP_SCALE)
#define FP_MUL(a, b) ((int32_t)(((int64_t)(a) * (b)) >> 16))
#define FP_DIV(a, b) ((int32_t)(((int64_t)(a) << 16) / (b)))

// Change this to 2 or 3 to match your Mixer network.
#define NUM_ROBOTS 2

#define BOLT_MAX_PAYLOAD 64u

#define MX_PAYLOAD_ONLY 52u
#define AGGREGATE_SIZE  6u

typedef struct {
    uint8_t  robot_id;
    uint16_t seq;
    uint64_t t_ns;
    int32_t  x_fp;
    int32_t  y_fp;
    int32_t  z_fp;
    int32_t  qx_fp;
    int32_t  qy_fp;
    int32_t  qz_fp;
    int32_t  qw_fp;
    uint8_t  status;
} __attribute__((packed)) RobotPoseMsg_t;

#define POSE_STATUS_VALID       (1u << 0)
#define POSE_STATUS_STALE       (1u << 1)
#define POSE_STATUS_INITIALISED (1u << 2)

#define MESSAGE_SIZE sizeof(RobotPoseMsg_t)

enum bolt_pkt_type
{
    BOLT_SYNC = 0,
    BOLT_POSE = 1,
};

typedef struct __attribute__((packed))
{
    uint16_t round;
} sync_pkt_t;

typedef struct __attribute__((packed))
{
    uint8_t  robot_id;
    uint16_t seq;
    uint16_t cp_tx_round;
    uint64_t t_ns;
    int32_t  x_fp;
    int32_t  y_fp;
    int32_t  z_fp;
    int32_t  qx_fp;
    int32_t  qy_fp;
    int32_t  qz_fp;
    int32_t  qw_fp;
} pose_pkt_t;

typedef struct __attribute__((packed))
{
    struct __attribute__((packed))
    {
        uint8_t type;
        uint8_t pad;
    };
    union __attribute__((packed))
    {
        uint8_t     payload_start;
        sync_pkt_t  sync;
        pose_pkt_t  pose;
    };
} bolt_pkt_t;

#define BOLT_PKT_HEADER_SIZE offsetof(bolt_pkt_t, payload_start)
#define LEN_BOLT_POSE       (BOLT_PKT_HEADER_SIZE + sizeof(pose_pkt_t))
#define LEN_BOLT_SYNC       (BOLT_PKT_HEADER_SIZE + sizeof(sync_pkt_t))

#endif
