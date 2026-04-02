#ifndef __WIRELESS_CONTROL_H__
#define __WIRELESS_CONTROL_H__

#include "mixer/mixer.h"

// The Bolt durations contain besides the actual SPI data transfer
// also the code around such as memcpy, mixer_write etc.
// measured: <274us for generation size 3 (1 control, 2 data packets)
#define BOLT_WRITE_DURATION		(MX_GENERATION_SIZE * GPI_TICK_US_TO_HYBRID2(200))

// In the current state, only one msg is read from Bolt before the Mixer round.
// measured: <102us for 20B
#define BOLT_READ_DURATION		(GPI_TICK_US_TO_HYBRID2(200))

// measured: <6us
#define MIXER_ARM_DURATION		(GPI_TICK_US_TO_HYBRID2(10))

#define MIXER_INITIATOR_DELAY	(3 * MX_SLOT_LENGTH)

// After calling mixer_start, the actual first transmission
// (starting point for the deadline calculation) is a bit delayed.
// measured: ~47us
#define MIXER_FIRST_TX_DELAY	(GPI_TICK_US_TO_HYBRID2(47))

#define MIXER_DURATION			(MIXER_FIRST_TX_DELAY + (MX_ROUND_LENGTH * MX_SLOT_LENGTH))

// Safety buffer after Mixer to compensate potential inaccuracies at the end of a Mixer round.
#define MIXER_DEADLINE_BUFFER	(5 * MX_SLOT_LENGTH)

// offsets are time deltas with respect to t_ref (end of Mixer round) for certain events
#define SYNC_LINE_OFFSET(ref)		((ref) + MIXER_DEADLINE_BUFFER \
										   + BOLT_WRITE_DURATION)

// depends on the actual communication period p
#define MIXER_OFFSET(ref, p)		((ref) + (p) \
										   - MIXER_DURATION \
										   - MIXER_INITIATOR_DELAY \
										   + MX_SLOT_LENGTH) // TODO: unclear why

#define READ_AND_ARM_OFFSET(ref, p)	(MIXER_OFFSET(ref, p) - MIXER_ARM_DURATION \
														  - BOLT_READ_DURATION)


#define BOLT_PKT_INFO_SIZE	offsetof(bolt_pkt_t, payload_start)
#define LEN_BOLT_CONTROL	(BOLT_PKT_INFO_SIZE + sizeof(control_pkt_t))
#define LEN_BOLT_DATA		(BOLT_PKT_INFO_SIZE + sizeof(data_pkt_t))
#define LEN_BOLT_DATA_PROB	(BOLT_PKT_INFO_SIZE + sizeof(data_prob_pkt_t))
#define LEN_BOLT_PRINT		(BOLT_PKT_INFO_SIZE + sizeof(stats_pkt_t))
#define LEN_BOLT_INIT		(BOLT_PKT_INFO_SIZE + sizeof(init_pkt_t))
#define LEN_BOLT_DATA_FAKE	(BOLT_PKT_INFO_SIZE + sizeof(data_pkt_t))
#define LEN_BOLT_AGG_DATA	(BOLT_PKT_INFO_SIZE + sizeof(agg_pkt_t))

//**************************************************************************************************

// ATTENTION: Do not change this since the same AP code is used for periodic and predictive control.
enum bolt_pkt_type
{
	BOLT_CONTROL,
	BOLT_DATA,
	BOLT_DATA_PROB,
	BOLT_PRINT,
	BOLT_INIT,
	BOLT_DATA_FAKE,
	BOLT_AGG_DATA
};

//**************************************************************************************************

typedef struct __attribute__((packed)) prob_info_t_tag
{
	uint8_t currTrigger;
	uint8_t prob;
} prob_info_t;

// This structure contains information about the current mode (to inform nodes that join the
// network) and when (# of rounds) a new mode should be used.
typedef struct __attribute__((packed)) control_pkt_t_tag
{
	uint16_t seqNum;
	uint8_t  curSchedID : 4;
	uint8_t  newSchedID : 4;
	uint8_t  newSchedRnds;
} control_pkt_t;

typedef struct __attribute__((packed)) data_pkt_t_tag
{
	uint16_t senderID;
	uint8_t  payload[MX_PAYLOAD_ONLY];
} data_pkt_t;

// ATTENTION: Changing the member order in this struct requires changes in mixer_write() calls.
typedef struct __attribute__((packed)) data_prob_pkt_t_tag
{
	uint16_t senderID;
	uint8_t  payload[MX_PAYLOAD_ONLY];
	prob_info_t  prob_info;
} data_prob_pkt_t;

// NOTE: rtimer_clock_t is 8 bytes but in our use case, the values fit very well in 2 bytes
typedef struct __attribute__((packed)) stats_pkt_t_tag
{
	uint16_t radioOnCP;
	uint16_t cpuOnCP;
	uint8_t  msgsMissed;
	uint8_t  boltFails;
} stats_pkt_t;

typedef struct __attribute__((packed)) init_pkt_t_tag
{
	uint16_t 	phyNodeID;
	uint8_t		modeID;
} init_pkt_t;

typedef struct __attribute__((packed)) agg_pkt_t_tag
{
	uint8_t data[AGGREGATE_SIZE];
} agg_pkt_t;

typedef struct __attribute__((packed)) enhanced_data_pkt_t_tag
{
	uint16_t  senderID;
	uint8_t   payload[MX_PAYLOAD_ONLY];
	uint8_t   rank;
	uint8_t   version;
	uint8_t   priority;
	uint8_t   currTrigger;
	uint8_t   slot_full_rank;
	uint32_t  radio_on_time;
} enhanced_data_pkt_t;

typedef struct __attribute__((packed)) bolt_pkt_t_tag
{
	struct __attribute__((packed))
	{
		uint8_t type; // bolt_pkt_type
		uint8_t pad[1]; // pad to 16 bit
	};

	union __attribute__((packed))
	{
		uint8_t				payload_start; // just a marker (e.g. for offsetof(bolt_pkt_t, payload_start))
		data_pkt_t			data;
		data_prob_pkt_t		data_prob;
		control_pkt_t		control;
		stats_pkt_t			stats;
		init_pkt_t			init;
		agg_pkt_t			agg_data;
		enhanced_data_pkt_t	enh_data;
	};

} bolt_pkt_t;

ASSERT_CT_STATIC(sizeof(data_pkt_t) <= MX_PAYLOAD_SIZE, size_of_data_pkt_t_must_be_smaller_than_MX_PAYLOAD_SIZE);
ASSERT_CT_STATIC(sizeof(control_pkt_t) <= MX_PAYLOAD_SIZE, size_of_control_pkt_t_must_be_smaller_than_MX_PAYLOAD_SIZE);
ASSERT_CT_STATIC(sizeof(enhanced_data_pkt_t) <= MX_PAYLOAD_SIZE, size_of_enhanced_data_pkt_t_must_be_smaller_than_MX_PAYLOAD_SIZE);

//**************************************************************************************************

typedef struct prob_sentinel_tag
{
	uint8_t head;
	uint8_t tail;
	uint8_t num_nodes;
} prob_sentinel;



#endif // __WIRELESS_CONTROL_H__