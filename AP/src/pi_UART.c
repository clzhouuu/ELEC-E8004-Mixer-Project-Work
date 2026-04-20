#include <string.h>
#include <stdint.h>
#include <msp432p401r.h>
#include "MSP432P4xx/gpio.h"
#include "MSP432P4xx/uart.h"
#include "board_config.h"
#include "pi_UART.h"

// this needs to be changed for Raspberry Pi

// local varibales
static RobotPoseMsg_t g_latest_pose;
static uint8_t g_robot_id = 0;
static uint8_t g_fresh_pose = 0;

// states
typedef enum {
    PI_WAIT_START_A = 0,
    PI_WAIT_START_B,
    PI_WAIT_MSG_ID,
    PI_WAIT_LENGTH,
    PI_WAIT_PAYLOAD,
    PI_WAIT_CHECKSUM
} PiRxState_t;

// setup
static PiRxState_t g_state = PI_WAIT_START_A;
static uint8_t g_msg_id = 0;
static uint8_t g_length = 0;
static uint8_t g_payload_idx = 0;
static uint8_t g_checksum = 0;
static uint8_t g_payload[PI_UART_MAX_PAYLOAD];

// reset
static void parser_reset(void) {
    g_state = PI_WAIT_START_A;
    g_msg_id = 0;
    g_length = 0;
    g_payload_idx = 0;
    g_checksum = 0;
}

// process a message
static void process_frame(uint32_t now_ms) {
    PiPayload_t pose;

    if (g_length != sizeof(PiPayload_t)) {
        return;
    }

    memcpy(&pose, g_payload, sizeof(pose));

    g_latest_pose.timestamp_ms = now_ms;
    g_latest_pose.robot_id = g_robot_id;
    g_latest_pose.status = POSE_STATUS_VALID | POSE_STATUS_INITIALISED;
    g_latest_pose.x_fp = FP_FROM_FLOAT(pose.x);
    g_latest_pose.y_fp = FP_FROM_FLOAT(pose.y);
    g_latest_pose.theta_fp = FP_FROM_FLOAT(pose.theta);
    g_latest_pose.v_fp = FP_FROM_FLOAT(pose.v);

    g_fresh_pose = 1;
}


// byte by byte state parsing with structure START_A START_B MSG_ID LENGTH PAYLOAD CHECKSUM
static void uart_feed_byte(uint8_t byte, uint32_t now_ms) {
    switch (g_state) {
    case PI_WAIT_START_A:
        if (byte == PI_UART_START_A) {
            g_state = PI_WAIT_START_B;
        }
        break;

    case PI_WAIT_START_B:
        if (byte == PI_UART_START_B) {
            g_state = PI_WAIT_MSG_ID;
        } else {
            parser_reset();
        }
        break;

    case PI_WAIT_MSG_ID:
        g_msg_id = byte;
        g_checksum = byte;
        g_state = PI_WAIT_LENGTH;
        break;

    case PI_WAIT_LENGTH:
        if (byte == 0 || byte > PI_UART_MAX_PAYLOAD) {
            parser_reset();
            break;
        }
        g_length = byte;
        g_checksum ^= byte;
        g_payload_idx = 0;
        g_state = PI_WAIT_PAYLOAD;
        break;

    case PI_WAIT_PAYLOAD:
        g_payload[g_payload_idx++] = byte;
        g_checksum ^= byte;
        if (g_payload_idx >= g_length) {
            g_state = PI_WAIT_CHECKSUM;
        }
        break;

    case PI_WAIT_CHECKSUM:
        if (byte == g_checksum) {
            if (g_msg_id == PI_UART_MSG_POSE) {
                process_frame(now_ms);
            }
        }
        parser_reset();
        break;

    default:
        parser_reset();
        break;
    }
}

// public API
void pi_uart_init(uint8_t robot_id) {
    g_robot_id = robot_id;
    memset(&g_latest_pose, 0, sizeof(g_latest_pose));
    g_fresh_pose = 0;
    parser_reset();

    GPIO_setAsPeripheralModuleFunctionInputPin(
        UART_RX_PORT,
        UART_RX_PIN,
        GPIO_PRIMARY_MODULE_FUNCTION
    );

    GPIO_setAsPeripheralModuleFunctionOutputPin(
        UART_TX_PORT,
        UART_TX_PIN,
        GPIO_PRIMARY_MODULE_FUNCTION
    );

    const eUSCI_UART_Config uartConfig = {
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,
        6,
        8,
        0x20,
        EUSCI_A_UART_NO_PARITY,
        EUSCI_A_UART_LSB_FIRST,
        EUSCI_A_UART_ONE_STOP_BIT,
        EUSCI_A_UART_MODE,
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION // how were all these variables chosen
    };

    UART_initModule(EUSCI_A0_BASE, &uartConfig);
    UART_enableModule(EUSCI_A0_BASE);
}

// polling and processing
void pi_uart_poll(uint32_t now_ms) {
    while (UART_getInterruptStatus(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)) {
        uint8_t byte = UART_receiveData(EUSCI_A0_BASE);
        uart_feed_byte(byte, now_ms);
    }
}

// receiving pose
uint8_t pi_uart_get_pose(RobotPoseMsg_t *out_pose) {
    if (!g_fresh_pose) {
        return 0;
    }

    *out_pose = g_latest_pose;
    g_fresh_pose = 0;
    return 1;
}
