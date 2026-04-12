#include <msp432p401r.h>
#include <string.h>
#include "../include/bolt.h"
#include "../include/message.h"
#include "../include/create3.h"


// CHANGE THIS FOR EACH ROBOT
#define OWN_ROBOT_ID        0u

// WHAT DO WE WANT THE CLOCK SPEED TO BE? 20Hz?
#define SMCLK_HZ            12000000u

// HOW OFTEN DO WE WANT THIS TO RUN
#define CONTROL_PERIOD_MS   50u

// WHAT IS THE SAFETY RADIUS
#define SAFETY_RADIUS_M     0.5f

// system timer
static volatile uint32_t g_tick_ms = 0;

void SysTick_Handler(void) {
    g_tick_ms++;
}

static void systick_init(void) {
    SysTick->LOAD = (SMCLK_HZ / 1000) - 1;
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk
                  | SysTick_CTRL_TICKINT_Msk
                  | SysTick_CTRL_ENABLE_Msk;
}

static uint32_t get_tick(void) {
    return g_tick_ms;
}

// latest poses
static RobotPoseMsg_t g_poses[NUM_ROBOTS];

static int32_t isqrt(int32_t n) {
    if (n <= 0) return 0;
    int32_t x = n;
    int32_t y = (x + 1) / 2;
    while (y < x) {
        x = y;
        y = (x + n / x) / 2;
    }
    return x;
}

// distance calculation
static int32_t distance(RobotPoseMsg_t* a, RobotPoseMsg_t* b) {
    int32_t dx = a->x_fp - b->x_fp;
    int32_t dy = a->y_fp - b->y_fp;
    int64_t sumsq = (int64_t)dx * dx + (int64_t)dy * dy;
    return isqrt((int32_t)(sumsq >> 16));
}

// scan for new BOLT msgs
static void receive_from_bolt(void) {
    if (!bolt_data_available()) {
        return;
    }

    uint8_t buf[BOLT_MAX_PAYLOAD];
    uint8_t len;

    if (!bolt_recv(buf, &len)) {
        return;
    }

    if (len == sizeof(RobotPoseMsg_t)) {
        RobotPoseMsg_t incoming;
        memcpy(&incoming, buf, sizeof(incoming));
        if (incoming.robot_id < NUM_ROBOTS && 
            incoming.robot_id != OWN_ROBOT_ID) {
            g_poses[incoming.robot_id] = incoming;
        }

    }
}

// build and send own pose to CP
static void send_own_pose(uint32_t now_ms) {
    RobotPoseMsg_t pose;
    (void)now_ms;

    if (!create3_get_pose(&pose)) {
        return;
    }

    g_poses[OWN_ROBOT_ID] = pose;
    bolt_write((uint8_t*)&pose, sizeof(pose));
}

/* calculate correction and send to CP
static void send_correction(uint32_t now_ms) {
    VelCorrectionMsg_t correction;
    correction.timestamp_ms = now_ms;
    correction.robot_id = OWN_ROBOT_ID;
    correction.delta_v_fp = 0;
    correction.delta_w_fp = 0;
    correction.flags = VEL_CORR_VALID;

    int32_t safety_fp = FP_FROM_FLOAT(SAFETY_RADIUS_M);
    uint8_t i;

    for (i = 0; i < NUM_ROBOTS; i++) {
        if (i == OWN_ROBOT_ID) {
            continue;
        }

        if (!(g_poses[i].status & POSE_STATUS_VALID)) {
            continue;
        }

        if (g_poses[i].status & POSE_STATUS_STALE) {
            continue;
        }

        int32_t dist = distance(&g_poses[OWN_ROBOT_ID], &g_poses[i]);

        if (dist < safety_fp) {
            // MATH NOT DONE
            break;
        }
    }

    bolt_send(CH_AP_VEL_CORRECTION, &correction, sizeof(correction));
}
*/

// mark as stale data if no update for a while
static void update_stale_flags(uint32_t now_ms) {
    uint8_t i;
    for (i = 0; i < NUM_ROBOTS; i++) {
        if (i == OWN_ROBOT_ID) {
            continue;
        }
        if (!(g_poses[i].status & POSE_STATUS_VALID)) {
            continue;
        }

        uint32_t age = now_ms - g_poses[i].timestamp_ms;
        if (age > 500u) {
            g_poses[i].status |= POSE_STATUS_STALE;
        }
    }
}


int main(void) {
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;   // stop watchdog

    systick_init();
    bolt_init();
    create3_init(OWN_ROBOT_ID);


    __enable_irq();

    uint32_t last_control_ms = 0;

    while (1) {
        uint32_t now = get_tick();
        
        create3_poll(now);

        // check for updates
        receive_from_bolt();
        /* every X ms (20 Hz) send pose and correction
        if ((now - last_control_ms) >= CONTROL_PERIOD_MS) {
            last_control_ms = now;
            update_stale_flags(now);
            send_own_pose(now);
            send_correction(now);
        }
        */
    }
}