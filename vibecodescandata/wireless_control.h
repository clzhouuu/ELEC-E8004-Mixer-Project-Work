#ifndef __WIRELESS_CONTROL_H__
#define __WIRELESS_CONTROL_H__

#include "mixer/mixer.h"
#include <stddef.h>

// The Bolt durations contain besides the actual SPI data transfer
// also the code around such as memcpy, mixer_write etc.
// measured: <274us for generation size 3 (1 control, 2 data packets)
#define BOLT_WRITE_DURATION     (MX_GENERATION_SIZE * GPI_TICK_US_TO_HYBRID2(200))

// In the current state, only one msg is read from Bolt before the Mixer round.
// measured: <102us for 20B
#define BOLT_READ_DURATION      (GPI_TICK_US_TO_HYBRID2(200))

// measured: <6us
#define MIXER_ARM_DURATION      (GPI_TICK_US_TO_HYBRID2(10))

#define MIXER_INITIATOR_DELAY   (3 * MX_SLOT_LENGTH)

// After calling mixer_start, the actual first transmission
// (starting point for the deadline calculation) is a bit delayed.
// measured: ~47us
#define MIXER_FIRST_TX_DELAY    (GPI_TICK_US_TO_HYBRID2(47))

#define MIXER_DURATION          (MIXER_FIRST_TX_DELAY + (MX_ROUND_LENGTH * MX_SLOT_LENGTH))

// Safety buffer after Mixer to compensate potential inaccuracies at the end of a Mixer round.
#define MIXER_DEADLINE_BUFFER   (5 * MX_SLOT_LENGTH)

// offsets are time deltas with respect to t_ref (end of Mixer round) for certain events
#define SYNC_LINE_OFFSET(ref)       ((ref) + MIXER_DEADLINE_BUFFER \
                                           + BOLT_WRITE_DURATION)

// depends on the actual communication period p
#define MIXER_OFFSET(ref, p)        ((ref) + (p) \
                                           - MIXER_DURATION \
                                           - MIXER_INITIATOR_DELAY \
                                           + MX_SLOT_LENGTH) // TODO: unclear why

#define READ_AND_ARM_OFFSET(ref, p) (MIXER_OFFSET(ref, p) - MIXER_ARM_DURATION \
                                                          - BOLT_READ_DURATION)

//**************************************************************************************************

enum bolt_pkt_type
{
    BOLT_SYNC   = 0,    // initiator sync packet (slot 0), no payload needed
    BOLT_POSE   = 1,    // robot pose packet (slots 1-3)
    BOLT_SCAN   = 2,    // LiDAR scan fragment
};

//**************************************************************************************************

typedef struct __attribute__((packed))
{
    uint16_t round;
} sync_pkt_t;

typedef struct __attribute__((packed))
{
    uint8_t  robot_id;
    int32_t  x_fp;
    int32_t  y_fp;
    int32_t  theta_fp;
    int32_t  v_fp;
    uint32_t timestamp_ms;
} pose_pkt_t;

// LiDAR scan fragment packet.
//
// A full scan is split into fragments before being sent over BOLT.
// Each fragment carries SCAN_POINTS_PER_FRAG range samples (uint16, mm).
//
// Layout of MX_PAYLOAD_SIZE = 32 bytes:
//   robot_id      1 B
//   frag_id       1 B   (0-based fragment index)
//   total_frags   1 B   (total number of fragments for this scan)
//   count         1 B   (number of valid points in this fragment, <= SCAN_POINTS_PER_FRAG)
//   angle_start   4 B   (float32, start angle of this fragment in radians)
//   angle_step    4 B   (float32, angle increment per point in radians)
//   timestamp_ms  4 B
//   ranges        2 B * SCAN_POINTS_PER_FRAG
//
// Total with SCAN_POINTS_PER_FRAG = 8:
//   1+1+1+1+4+4+4 + 8*2 = 32 bytes  -- fits exactly in MX_PAYLOAD_SIZE
#define SCAN_POINTS_PER_FRAG    8

typedef struct __attribute__((packed))
{
    uint8_t  robot_id;
    uint8_t  frag_id;
    uint8_t  total_frags;
    uint8_t  count;           // valid points in this fragment (<= SCAN_POINTS_PER_FRAG)
    float    angle_start;     // radians, start angle of this fragment
    float    angle_step;      // radians, angle increment per point
    uint32_t timestamp_ms;
    uint16_t ranges[SCAN_POINTS_PER_FRAG]; // mm, 0 = invalid/inf
} scan_pkt_t;

ASSERT_CT_STATIC(sizeof(scan_pkt_t) == MX_PAYLOAD_SIZE,
                 scan_pkt_t_must_be_exactly_MX_PAYLOAD_SIZE_bytes);

//**************************************************************************************************

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
        scan_pkt_t  scan;
    };
} bolt_pkt_t;

#define BOLT_PKT_HEADER_SIZE    offsetof(bolt_pkt_t, payload_start)
#define LEN_BOLT_SYNC           (BOLT_PKT_HEADER_SIZE + sizeof(sync_pkt_t))
#define LEN_BOLT_POSE           (BOLT_PKT_HEADER_SIZE + sizeof(pose_pkt_t))
#define LEN_BOLT_SCAN           (BOLT_PKT_HEADER_SIZE + sizeof(scan_pkt_t))

ASSERT_CT_STATIC(sizeof(sync_pkt_t) <= MX_PAYLOAD_SIZE, sync_pkt_t_too_large_for_MX_PAYLOAD_SIZE);
ASSERT_CT_STATIC(sizeof(pose_pkt_t) <= MX_PAYLOAD_SIZE, pose_pkt_t_too_large_for_MX_PAYLOAD_SIZE);
ASSERT_CT_STATIC(sizeof(scan_pkt_t) <= MX_PAYLOAD_SIZE, scan_pkt_t_too_large_for_MX_PAYLOAD_SIZE);

#endif // __WIRELESS_CONTROL_H__
