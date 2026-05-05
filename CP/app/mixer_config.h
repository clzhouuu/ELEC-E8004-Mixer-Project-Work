#ifndef __MIXER_CONFIG_H__
#define __MIXER_CONFIG_H__

// mixer configuration file
// Adapt the settings to the needs of your application.

#include "gpi/platform_spec.h"		// GPI_ARCH_IS_...
#include "gpi/tools.h"				// NUM_ELEMENTS()

// GPI_ARCH_BOARD_nRF_PCA10056
// GPI_ARCH_BOARD_TUDNES_DPP2COM

/*****************************************************************************/
/* basic settings ************************************************************/

// The array contains physical node IDs and their position in the array is the logical node ID.
// static const uint8_t nodes[]	= { 1, 2, 3 };
static const uint8_t nodes[] = { 1, 2, 3};

#define MX_NUM_NODES			NUM_ELEMENTS(nodes)
#define MX_INITIATOR_ID			0
#define MX_PAYLOAD_ONLY			20 // 16B state and 4B control input for logging
#define MX_PAYLOAD_SIZE			32 // +2 because of senderID + 2 because of rank and version + 2 prio and trigger + 1 slot_full_rank + 4 radio_on_time + 1 just to have a power of 2
#define DEFAULT_MODE			0

#if DEFAULT_MODE == 0
	// Entries in the plants array send probability values.
	static const uint8_t plants[] = {1, 2, 3};

	#define MX_ROUND_LENGTH				190 // in #slots
	#define ROUND_PERIOD				GPI_TICK_MS_TO_HYBRID2(100)
	#define AGGREGATE_SIZE				6
	#define MX_SLOT_LENGTH				GPI_TICK_US_TO_HYBRID2(400)
#endif

#define NUM_PLANTS				NUM_ELEMENTS(plants)
#define MX_GENERATION_SIZE		(NUM_ELEMENTS(plants) + 1) // + initiator


// Possible values (Gpi_Radio_Mode):
//		IEEE_802_15_4	= 1
//		BLE_1M			= 2
//		BLE_2M			= 3
//		BLE_125k		= 4
//		BLE_500k		= 5
#define MX_PHY_MODE				3
// Values mentioned in the manual (nRF52840_PS_v1.1):
// +8dBm,  +7dBm,  +6dBm,  +5dBm,  +4dBm,  +3dBm, + 2dBm,
//  0dBm,  -4dBm,  -8dBm, -12dBm, -16dBm, -20dBm, -40dBm
#define MX_TX_PWR_DBM			-8


/*****************************************************************************/
/* special settings **********************************************************/

#define MX_WEAK_ZEROS			1
#define WEAK_RELEASE_SLOT		1
#define MX_WARMSTART_RNDS		1
#define PLANT_STATE_LOGGING		1
#define SIMULATE_MESSAGES		0

// turn verbose log messages on or off
// NOTE: These additional prints might take too long when using short round intervals.
#define MX_VERBOSE_STATISTICS	1
#define MX_VERBOSE_PACKETS		0
#define MX_VERBOSE_PROFILE		0
#define WC_PROFILE_MAIN			0

#define MX_SMART_SHUTDOWN		1
// 0	no smart shutdown
// 1	no unfinished neighbor, without full-rank map(s)
// 2	no unfinished neighbor
// 3	all nodes full rank
// 4	all nodes full rank, all neighbors ACKed knowledge of this fact
// 5	all nodes full rank, all nodes ACKed knowledge of this fact
#define MX_SMART_SHUTDOWN_MODE	2


/*****************************************************************************/
/* convinience macros ********************************************************/

#define SET_COM_GPIO1() (NRF_P0->OUTSET = BV(26))
#define CLR_COM_GPIO1() (NRF_P0->OUTCLR = BV(26))
#define SET_COM_GPIO2() (NRF_P0->OUTSET = BV(28))
#define CLR_COM_GPIO2() (NRF_P0->OUTCLR = BV(28))

#endif // __MIXER_CONFIG_H__
