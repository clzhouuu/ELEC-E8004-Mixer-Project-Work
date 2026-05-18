/**
 * pi_uart.c
 * =========
 * UART ↔ Pi multiplexer for the MSP432 Application Processor.
 *
 * Hardware assumptions
 * --------------------
 *   Peripheral : eUSCI_A1  (change EUSCI_Ax_BASE below if needed)
 *   SMCLK      : 12 000 000 Hz  (set by CS_setDCOCenteredFrequency in main)
 *   Baud rate  : 460 800 bps  (configurable via pi_uart_init)
 *
 * The UART RX interrupt feeds a circular byte buffer.  pi_uart_poll() is
 * called from the main loop to drain that buffer and run the frame parser.
 * TX is polled (byte-by-byte spin-wait) – acceptable for 21-byte pose frames
 * at 20 Hz; for large scan frames consider DMA.
 *
 * Frame parser state machine
 * --------------------------
 *   WAIT_SYNC0 → WAIT_SYNC1 → READ_LEN_LO → READ_LEN_HI
 *   → ACCUMULATE → DISPATCH
 */

#include "pi_uart.h"
#include "message.h"

#include <msp432p401r.h>
#include <MSP432P4xx/cs.h>   /* only for SMCLK frequency constant lookup */

#include <string.h>
#include <stddef.h>

/* ── Build-time configuration ──────────────────────────────────────────────── */

#define PI_UART_BASE        EUSCI_A1_BASE   /* change if wired differently   */
#define PI_UART_SMCLK_HZ    12000000u
#define PI_UART_BAUD        460800u

/* Internal RX ring buffer – must be power-of-two for cheap masking.          */
#define RX_RING_SIZE        4096u
#define RX_RING_MASK        (RX_RING_SIZE - 1u)

/* Maximum COBS-encoded frame body we will accept (prevents heap overflow).   */
#define MAX_ENC_LEN         8192u

/* Wire constants */
#define SYNC_BYTE0  0xAAu
#define SYNC_BYTE1  0x55u

/* ── CRC-16/IBM-SDLC (CCITT) – same polynomial as Python binascii.crc_hqx ── */

static uint16_t crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFu;
    uint16_t i;
    for (i = 0; i < len; i++)
    {
        crc ^= (uint16_t)data[i] << 8;
        uint8_t b;
        for (b = 0; b < 8; b++)
        {
            if (crc & 0x8000u)
                crc = (uint16_t)((crc << 1) ^ 0x1021u);
            else
                crc <<= 1;
        }
    }
    return crc;
}

/* ── Minimal COBS decoder ──────────────────────────────────────────────────── */
/**
 * Returns decoded length on success, 0 on error.
 * out must point to a buffer of at least (in_len) bytes.
 */
static uint16_t cobs_decode(const uint8_t *in, uint16_t in_len,
                             uint8_t *out, uint16_t out_max)
{
    uint16_t out_len = 0;
    uint16_t i = 0;

    while (i < in_len)
    {
        uint8_t code = in[i++];
        if (code == 0) return 0;   /* framing error */

        uint8_t k;
        for (k = 1; k < code; k++)
        {
            if (i >= in_len || out_len >= out_max) return 0;
            out[out_len++] = in[i++];
        }
        if (code < 0xFF && out_len < out_max)
        {
            out[out_len++] = 0x00u;
        }
    }
    /* Remove the trailing zero that COBS always appends */
    if (out_len > 0) out_len--;
    return out_len;
}

/* ── Minimal COBS encoder ──────────────────────────────────────────────────── */
/**
 * Returns encoded length (excluding null terminator), 0 on error.
 * out must be at least (in_len + in_len/254 + 2) bytes.
 */
static uint16_t cobs_encode(const uint8_t *in, uint16_t in_len,
                              uint8_t *out, uint16_t out_max)
{
    /* We need in_len + overhead + 1 null terminator */
    if ((uint32_t)in_len + in_len / 254 + 3 > out_max) return 0;

    uint16_t out_idx  = 0;
    uint16_t code_idx = 0;   /* index where the next code byte goes */
    uint8_t  code     = 1;

    code_idx = out_idx++;   /* reserve space for first code byte */

    uint16_t i;
    for (i = 0; i < in_len; i++)
    {
        if (in[i] == 0x00u)
        {
            out[code_idx] = code;
            code_idx = out_idx++;
            code = 1;
        }
        else
        {
            out[out_idx++] = in[i];
            code++;
            if (code == 0xFFu)
            {
                out[code_idx] = code;
                code_idx = out_idx++;
                code = 1;
            }
        }
    }
    out[code_idx]   = code;
    out[out_idx++]  = 0x00u;   /* null terminator */
    return out_idx;            /* includes terminator */
}

/* ── RX ring buffer (filled by ISR) ──────────────────────────────────────────*/

static volatile uint8_t  rx_ring[RX_RING_SIZE];
static volatile uint16_t rx_head = 0;   /* written by ISR     */
static volatile uint16_t rx_tail = 0;   /* consumed by poll() */

/* ISR – keep short */
void EUSCIA1_IRQHandler(void)   /* adjust name to match your startup file */
{
    if (EUSCI_A1->IFG & EUSCI_A_IFG_RXIFG)
    {
        uint8_t byte = (uint8_t)EUSCI_A1->RXBUF;   /* clears flag */
        uint16_t next = (rx_head + 1u) & RX_RING_MASK;
        if (next != rx_tail)   /* drop if full */
        {
            rx_ring[rx_head] = byte;
            rx_head = next;
        }
    }
}

/* ── Frame parser state machine ───────────────────────────────────────────── */

typedef enum {
    ST_WAIT_SYNC0 = 0,
    ST_WAIT_SYNC1,
    ST_LEN_LO,
    ST_LEN_HI,
    ST_ACCUMULATE,
    ST_DISPATCH
} parser_state_t;

static parser_state_t  g_state    = ST_WAIT_SYNC0;
static uint16_t        g_enc_len  = 0;
static uint16_t        g_acc_idx  = 0;

/* Static buffers – avoids stack pressure on MSP432 */
static uint8_t  g_enc_buf[MAX_ENC_LEN];
static uint8_t  g_dec_buf[MAX_ENC_LEN];   /* COBS decoded (same max size)   */

/* ── Module-level state ───────────────────────────────────────────────────── */

static uint8_t         g_own_robot_id = 1;

/* Latched pose from Pi – protected by no-preemption (single main loop) */
static RobotPoseMsg_t  g_rx_pose;
static uint8_t         g_rx_pose_fresh = 0;

/* ── Pose binary layout helpers (little-endian) ───────────────────────────── */
/*
 *  Byte 0      uint8  robot_id
 *  Bytes 1-4   int32  x_fp   (Q16.16)
 *  Bytes 5-8   int32  y_fp
 *  Bytes 9-12  int32  theta_fp
 *  Bytes 13-16 int32  v_fp
 *  Bytes 17-20 uint32 timestamp_ms
 *  Total: 21 bytes
 */
#define POSE_BODY_SIZE  21u

static void unpack_pose_body(const uint8_t *body, RobotPoseMsg_t *out)
{
    out->robot_id = body[0];

    /* int32 little-endian read */
#define LE32S(p) ((int32_t)( (uint32_t)(p)[0]         \
                            | (uint32_t)(p)[1] << 8    \
                            | (uint32_t)(p)[2] << 16   \
                            | (uint32_t)(p)[3] << 24 ))
#define LE32U(p) ((uint32_t)( (uint32_t)(p)[0]         \
                            | (uint32_t)(p)[1] << 8    \
                            | (uint32_t)(p)[2] << 16   \
                            | (uint32_t)(p)[3] << 24 ))

    out->x_fp         = LE32S(body + 1);
    out->y_fp         = LE32S(body + 5);
    out->theta_fp     = LE32S(body + 9);
    out->v_fp         = LE32S(body + 13);
    out->timestamp_ms = LE32U(body + 17);

    out->status = POSE_STATUS_VALID | POSE_STATUS_INITIALISED;

#undef LE32S
#undef LE32U
}

static void pack_pose_body(const RobotPoseMsg_t *pose, uint8_t *body)
{
#define PUT8(p,  v)  (p)[0] = (uint8_t)(v)
#define PUT32(p, v) \
    (p)[0] = (uint8_t)((uint32_t)(v));        \
    (p)[1] = (uint8_t)((uint32_t)(v) >> 8);   \
    (p)[2] = (uint8_t)((uint32_t)(v) >> 16);  \
    (p)[3] = (uint8_t)((uint32_t)(v) >> 24)

    PUT8 (body,      pose->robot_id);
    PUT32(body +  1, pose->x_fp);
    PUT32(body +  5, pose->y_fp);
    PUT32(body +  9, pose->theta_fp);
    PUT32(body + 13, pose->v_fp);
    PUT32(body + 17, pose->timestamp_ms);

#undef PUT8
#undef PUT32
}

/* ── Frame dispatch ───────────────────────────────────────────────────────── */

static void dispatch_frame(const uint8_t *decoded, uint16_t dec_len)
{
    if (dec_len < 1) return;

    uint8_t pkt_type = decoded[0];
    const uint8_t *body = decoded + 1;
    uint16_t body_len = dec_len - 1;

    switch (pkt_type)
    {
        case PI_PKT_SCAN:
            /* Scan data from the Pi – the AP currently has no use for raw
             * scan data, so we silently discard it.  If you add a local
             * obstacle-avoidance routine, parse body here.               */
            break;

        case PI_PKT_POSE_TX:
            /* Own pose arriving from Pi → latch for main loop to pick up */
            if (body_len >= POSE_BODY_SIZE)
            {
                unpack_pose_body(body, &g_rx_pose);
                g_rx_pose_fresh = 1;
            }
            break;

        default:
            /* Unknown type – ignore */
            break;
    }
}

/* ── Parser – consume one byte at a time from ring buffer ─────────────────── */

static void parser_feed(uint8_t byte)
{
    switch (g_state)
    {
        case ST_WAIT_SYNC0:
            if (byte == SYNC_BYTE0) g_state = ST_WAIT_SYNC1;
            break;

        case ST_WAIT_SYNC1:
            g_state = (byte == SYNC_BYTE1) ? ST_LEN_LO : ST_WAIT_SYNC0;
            break;

        case ST_LEN_LO:
            g_enc_len = byte;
            g_state   = ST_LEN_HI;
            break;

        case ST_LEN_HI:
            g_enc_len |= (uint16_t)byte << 8;
            if (g_enc_len == 0 || g_enc_len > MAX_ENC_LEN)
            {
                g_state = ST_WAIT_SYNC0;   /* reject oversized frame */
            }
            else
            {
                g_acc_idx = 0;
                g_state   = ST_ACCUMULATE;
            }
            break;

        case ST_ACCUMULATE:
            g_enc_buf[g_acc_idx++] = byte;
            if (g_acc_idx >= g_enc_len + 2u)   /* +2 for CRC */
            {
                g_state = ST_DISPATCH;
                /* fall through immediately */
                goto do_dispatch;
            }
            break;

        case ST_DISPATCH:
        do_dispatch:
        {
            /* Last two bytes accumulated are the CRC */
            uint16_t crc_recv = (uint16_t)g_enc_buf[g_enc_len]
                              | (uint16_t)g_enc_buf[g_enc_len + 1] << 8;
            uint16_t crc_calc = crc16(g_enc_buf, g_enc_len);

            if (crc_calc == crc_recv)
            {
                uint16_t dec_len = cobs_decode(g_enc_buf, g_enc_len,
                                               g_dec_buf, (uint16_t)sizeof(g_dec_buf));
                if (dec_len > 0)
                    dispatch_frame(g_dec_buf, dec_len);
            }
            /* Always reset parser after dispatch attempt */
            g_state = ST_WAIT_SYNC0;
            break;
        }
    }
}

/* ── TX helpers ───────────────────────────────────────────────────────────── */

static void uart_tx_byte(uint8_t b)
{
    while (!(EUSCI_A1->IFG & EUSCI_A_IFG_TXIFG));
    EUSCI_A1->TXBUF = b;
}

static void uart_tx_bytes(const uint8_t *data, uint16_t len)
{
    uint16_t i;
    for (i = 0; i < len; i++) uart_tx_byte(data[i]);
}

/* ── Public API ───────────────────────────────────────────────────────────── */

void pi_uart_init(uint8_t robot_id)
{
    g_own_robot_id = robot_id;
    g_rx_pose_fresh = 0;
    g_state = ST_WAIT_SYNC0;

    /* Configure P3.2 (RX) and P3.3 (TX) as eUSCI_A1 module pins.
     * Adjust port/pin numbers to match your board schematic.            */
    P3->SEL0 |=  (BIT2 | BIT3);
    P3->SEL1 &= ~(BIT2 | BIT3);

    EUSCI_A1->CTLW0 = EUSCI_A_CTLW0_SWRST;   /* hold in reset */

    EUSCI_A1->CTLW0 = EUSCI_A_CTLW0_SWRST
                    | EUSCI_A_CTLW0_SSEL__SMCLK;   /* clock = SMCLK */

    /* Baud-rate registers for 460800 @ 12 MHz
     * UCBRx=1, UCBRFx=10, UCBRSx=0x00, UCOS16=1
     * (from Table 22-5 in MSP432 TRM, or slau356i online calculator)   */
    EUSCI_A1->BRW   = 1u;
    EUSCI_A1->MCTLW = (10u << 4) | EUSCI_A_MCTLW_OS16;

    EUSCI_A1->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;   /* release reset */

    /* Enable RX interrupt */
    EUSCI_A1->IE |= EUSCI_A_IE_RXIE;
    NVIC_EnableIRQ(EUSCIA1_IRQn);
}

void pi_uart_poll(uint32_t now_ms)
{
    (void)now_ms;   /* reserved for future rate-limiting / watchdog use */

    /* Drain ring buffer – disable IRQ just long enough to snapshot head */
    __disable_irq();
    uint16_t head = rx_head;
    __enable_irq();

    while (rx_tail != head)
    {
        uint8_t byte = (uint8_t)rx_ring[rx_tail];
        rx_tail = (rx_tail + 1u) & RX_RING_MASK;
        parser_feed(byte);

        /* Re-snapshot head in case ISR added more bytes */
        __disable_irq();
        head = rx_head;
        __enable_irq();
    }
}

uint8_t pi_uart_get_pose(RobotPoseMsg_t *pose_out)
{
    if (!g_rx_pose_fresh) return 0;
    *pose_out       = g_rx_pose;
    g_rx_pose_fresh = 0;
    return 1;
}

void pi_uart_send_pose(const RobotPoseMsg_t *pose)
{
    /* Build inner payload: type byte + 21-byte pose body */
    uint8_t inner[1 + POSE_BODY_SIZE];
    inner[0] = PI_PKT_POSE_RX;
    pack_pose_body(pose, inner + 1);

    /* COBS encode */
    /* Worst-case COBS output: input_len + ceil(input_len/254) + 1 terminator */
    uint8_t  enc[1 + POSE_BODY_SIZE + 4];   /* generously sized */
    uint16_t enc_len = cobs_encode(inner, (uint16_t)sizeof(inner),
                                   enc,   (uint16_t)sizeof(enc));
    if (enc_len == 0) return;   /* encode error */

    /* enc_len includes the null terminator byte; the wire format sends
     * enc_len-1 bytes as "encoded payload" (terminator is implicit).
     * Match the Python convention: ENC_LEN = bytes before null term.      */
    uint16_t wire_enc_len = enc_len - 1u;

    uint16_t crc = crc16(enc, wire_enc_len);

    /* Emit: SYNC | ENC_LEN(2 LE) | COBS_BYTES | CRC(2 LE) */
    uart_tx_byte(SYNC_BYTE0);
    uart_tx_byte(SYNC_BYTE1);
    uart_tx_byte((uint8_t)(wire_enc_len & 0xFFu));
    uart_tx_byte((uint8_t)(wire_enc_len >> 8));
    uart_tx_bytes(enc, wire_enc_len);
    uart_tx_byte((uint8_t)(crc & 0xFFu));
    uart_tx_byte((uint8_t)(crc >> 8));
}
