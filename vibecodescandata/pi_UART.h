#ifndef __PI_UART_H__
#define __PI_UART_H__

#include <stdint.h>

// -----------------------------------------------------------------------
// Protocol constants
// -----------------------------------------------------------------------
#define PI_UART_START_A         0xAA
#define PI_UART_START_B         0x55

// Message IDs
#define PI_UART_MSG_POSE        0x01    // Pi -> MSP432: own robot pose (PiPayload_t)
#define PI_UART_MSG_PEER_POSE   0x02    // MSP432 -> Pi: peer robot pose
#define PI_UART_MSG_SCAN_FRAG   0x03    // Pi -> MSP432: LiDAR scan fragment

#define PI_UART_MAX_PAYLOAD     64u

// -----------------------------------------------------------------------
// Fixed-point helpers  (Q16.16)
// -----------------------------------------------------------------------
#define FP_SCALE                65536
#define FP_FROM_FLOAT(x)        ((int32_t)((x) * FP_SCALE))
#define FP_TO_FLOAT(x)          ((float)(x) / FP_SCALE)

// -----------------------------------------------------------------------
// Pose status flags
// -----------------------------------------------------------------------
#define POSE_STATUS_VALID       0x01
#define POSE_STATUS_INITIALISED 0x02
#define POSE_STATUS_STALE       0x04

// -----------------------------------------------------------------------
// Number of robots in the swarm
// -----------------------------------------------------------------------
#define NUM_ROBOTS              3u

// -----------------------------------------------------------------------
// Structs
// -----------------------------------------------------------------------

// Pose payload sent from Pi to MSP432 (MSG_POSE)
typedef struct __attribute__((packed))
{
    float x;
    float y;
    float theta;
    float v;
} PiPayload_t;

// Pose stored internally on MSP432
typedef struct
{
    uint8_t  robot_id;
    int32_t  x_fp;
    int32_t  y_fp;
    int32_t  theta_fp;
    int32_t  v_fp;
    uint32_t timestamp_ms;
    uint8_t  status;
} RobotPoseMsg_t;

// Scan fragment payload sent from Pi to MSP432 (MSG_SCAN_FRAG).
// Matches what lidar_node.py sends: each fragment carries up to
// PI_SCAN_POINTS_PER_FRAG range samples (uint16, mm).
#define PI_SCAN_POINTS_PER_FRAG     8u
#define PI_SCAN_MAX_FRAGS           64u     // supports up to 512 downsampled points

typedef struct __attribute__((packed))
{
    uint8_t  robot_id;
    uint8_t  frag_id;       // 0-based
    uint8_t  total_frags;
    uint8_t  count;         // valid points in this fragment
    float    angle_start;   // radians
    float    angle_step;    // radians per point
    uint32_t timestamp_ms;
    uint16_t ranges[PI_SCAN_POINTS_PER_FRAG]; // mm, 0 = inf/invalid
} PiScanFrag_t;

// -----------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------
void    pi_uart_init(uint8_t robot_id);
void    pi_uart_poll(uint32_t now_ms);

// Returns 1 and fills out_pose if a fresh pose is available, else 0.
uint8_t pi_uart_get_pose(RobotPoseMsg_t *out_pose);

// Returns 1 and fills out_frag if a fresh scan fragment is available, else 0.
uint8_t pi_uart_get_scan_frag(PiScanFrag_t *out_frag);

// Send a peer pose back to the Pi.
void    pi_uart_send_pose(const RobotPoseMsg_t *pose);

#endif // __PI_UART_H__
