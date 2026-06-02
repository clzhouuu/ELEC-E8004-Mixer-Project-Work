#ifndef PI_UART_H
#define PI_UART_H

#include <stdint.h>
#include "message.h"

// Newline JSON pose messages.
#define PI_UART_MAX_PAYLOAD 256u

void pi_uart_init(uint8_t robot_id);
void pi_uart_poll(uint32_t now_ms);

uint8_t pi_uart_get_pose(RobotPoseMsg_t *out_pose);
void    pi_uart_send_pose(const RobotPoseMsg_t *pose);

// Test helper. Leave compiled in if you want, but call only under DUMMY_* flags.
void pi_uart_test_dummy_json(void);

#endif
