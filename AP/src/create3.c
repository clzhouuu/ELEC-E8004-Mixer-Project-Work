#include <string.h>
#include <stdint.h>
#include <../inc/msp432p401r.h>
#include "../include/driverlib/MSP432P4xx/gpio.h"
#include "../include/driverlib/MSP432P4xx/uart.h"
#include "../include/board_config.h"
#include "../include/create3.h"

// local varibales
static RobotPoseMsg_t g_latest_pose;
static uint8_t g_robot_id = 0;
static uint8_t g_fresh_pose = 0;

// states
typedef enum {
    C3_WAIT_START_A = 0,
    C3_WAIT_START_B,
    C3_WAIT_MSG_ID,
    C3_WAIT_LENGTH,
    C3_WAIT_PAYLOAD,
    C3_WAIT_CHECKSUM
} Create3RxState_t;

// setup
static Create3RxState_t g_state = C3_WAIT_START_A;
static uint8_t g_msg_id = 0;
static uint8_t g_length = 0;
static uint8_t g_payload_idx = 0;
static uint8_t g_checksum = 0;
static uint8_t g_payload[CREATE3_MAX_PAYLOAD];

// reset
static void parser_reset(void) {
    g_state = C3_WAIT_START_A;
    g_msg_id = 0;
    g_length = 0;
    g_payload_idx = 0;
    g_checksum = 0;
}

// process a message
static void process_odom_frame(uint32_t now_ms) {
    Create3Payload_t odom;

    if (g_length != sizeof(Create3Payload_t)) {
        return;
    }

    memcpy(&odom, g_payload, sizeof(odom));

    g_latest_pose.timestamp_ms = now_ms;
    g_latest_pose.robot_id = g_robot_id;
    g_latest_pose.status = POSE_STATUS_VALID | POSE_STATUS_INITIALISED;
    g_latest_pose.x_fp = FP_FROM_FLOAT(odom.x);
    g_latest_pose.y_fp = FP_FROM_FLOAT(odom.y);
    g_latest_pose.theta_fp = FP_FROM_FLOAT(odom.theta);
    g_latest_pose.v_fp = FP_FROM_FLOAT(odom.v);
    g_latest_pose.w_fp = FP_FROM_FLOAT(odom.w);

    g_fresh_pose = 1;
}


// byte by byte state parsing with structure START_A START_B MSG_ID LENGTH PAYLOAD CHECKSUM
static void create3_feed_byte(uint8_t byte, uint32_t now_ms) {
    switch (g_state) {
    case C3_WAIT_START_A:
        if (byte == CREATE3_START_A) {
            g_state = C3_WAIT_START_B;
        }
        break;

    case C3_WAIT_START_B:
        if (byte == CREATE3_START_B) {
            g_state = C3_WAIT_MSG_ID;
        } else {
            parser_reset();
        }
        break;

    case C3_WAIT_MSG_ID:
        g_msg_id = byte;
        g_checksum = byte;
        g_state = C3_WAIT_LENGTH;
        break;

    case C3_WAIT_LENGTH:
        if (byte == 0 || byte > CREATE3_MAX_PAYLOAD) {
            parser_reset();
            break;
        }
        g_length = byte;
        g_checksum ^= byte;
        g_payload_idx = 0;
        g_state = C3_WAIT_PAYLOAD;
        break;

    case C3_WAIT_PAYLOAD:
        g_payload[g_payload_idx++] = byte;
        g_checksum ^= byte;
        if (g_payload_idx >= g_length) {
            g_state = C3_WAIT_CHECKSUM;
        }
        break;

    case C3_WAIT_CHECKSUM:
        if (byte == g_checksum) {
            if (g_msg_id == CREATE3_MSG_ODOM) {
                process_odom_frame(now_ms);
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
void create3_init(uint8_t robot_id) {
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
void create3_poll(uint32_t now_ms) {
    while (UART_getInterruptStatus(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)) {
        uint8_t byte = UART_receiveData(EUSCI_A0_BASE);
        create3_feed_byte(byte, now_ms);
    }
}

// receiving pose
uint8_t create3_get_pose(RobotPoseMsg_t *out_pose) {
    if (!g_fresh_pose) {
        return 0;
    }

    *out_pose = g_latest_pose;
    g_fresh_pose = 0;
    return 1;
}