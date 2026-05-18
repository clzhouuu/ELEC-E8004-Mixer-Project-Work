/**
 * pi_uart.h
 * =========
 * UART interface between the MSP432 Application Processor (AP) and the
 * Raspberry Pi.
 *
 * Frame format (matches the Python LidarUARTBridge):
 *
 *   SYNC(2)  ENC_LEN(2 LE)  COBS_PAYLOAD(N)  CRC16-HQXQ(2 LE)
 *   SYNC = 0xAA 0x55
 *
 * Inside the COBS-decoded payload:
 *   Byte 0      : packet type
 *   Bytes 1..N  : type-specific body
 *
 * Packet types
 *   0x01  PI_PKT_SCAN       LiDAR scan data (opaque – not parsed here)
 *   0x02  PI_PKT_POSE_TX    Own pose sent by Pi to AP  (AP reads these)
 *   0x03  PI_PKT_POSE_RX    Peer pose sent by AP to Pi (AP writes these)
 *
 * Pose body layout (21 bytes, all little-endian):
 *   uint8_t  robot_id
 *   int32_t  x_fp         Q16.16 metres
 *   int32_t  y_fp         Q16.16 metres
 *   int32_t  theta_fp     Q16.16 radians
 *   int32_t  v_fp         Q16.16 m/s
 *   uint32_t timestamp_ms
 */

#ifndef PI_UART_H
#define PI_UART_H

#include <stdint.h>
#include "message.h"   /* RobotPoseMsg_t, POSE_STATUS_* */

#ifdef __cplusplus
extern "C" {
#endif

/* ── Packet type tags ──────────────────────────────────────────────────────── */
#define PI_PKT_SCAN     0x01u
#define PI_PKT_POSE_TX  0x02u   /* Pi → AP : own pose */
#define PI_PKT_POSE_RX  0x03u   /* AP → Pi : peer pose */

/* ── Frame limits ──────────────────────────────────────────────────────────── */
/* COBS worst-case overhead: ceil(N/254) extra bytes.
 * Largest expected payload: scan header + 20000 points × 6 B ≈ 120 kB.
 * For pose-only builds a much smaller buffer is fine.                        */
#define PI_UART_MAX_ENCODED  131072u
#define PI_UART_MAX_DECODED  131072u

/* ── Public API ────────────────────────────────────────────────────────────── */

/**
 * pi_uart_init
 *
 * Configure eUSCI_A1 for the given baud rate (SMCLK = 12 MHz assumed) and
 * prepare internal state.  Call once before any other pi_uart_* function.
 *
 * @param robot_id  This robot's ID (1-based), stored and stamped on outgoing
 *                  PI_PKT_POSE_RX frames.
 */
void pi_uart_init(uint8_t robot_id);

/**
 * pi_uart_poll
 *
 * Non-blocking drain of the UART RX FIFO into the internal ring buffer and
 * frame parser.  Call as frequently as possible from the main loop.
 *
 * @param now_ms  Current system time in milliseconds (from SysTick).
 */
void pi_uart_poll(uint32_t now_ms);

/**
 * pi_uart_get_pose
 *
 * Retrieve the most-recently parsed PI_PKT_POSE_TX packet from the Pi.
 * Returns 1 and fills *pose_out if a new pose has arrived since the last
 * call; returns 0 otherwise (pose_out is unchanged).
 *
 * Thread-safety: call only from the main loop (same context as pi_uart_poll).
 */
uint8_t pi_uart_get_pose(RobotPoseMsg_t *pose_out);

/**
 * pi_uart_send_pose
 *
 * Encode pose as a PI_PKT_POSE_RX frame and transmit it to the Pi.
 * Typically called after the AP receives a peer-robot pose via BOLT so that
 * the Pi can display / use it.
 *
 * @param pose  Peer robot pose to forward.  robot_id must be set.
 */
void pi_uart_send_pose(const RobotPoseMsg_t *pose);

#ifdef __cplusplus
}
#endif

#endif /* PI_UART_H */
