// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2013 - 2024 Intel Corporation

#ifndef IPU_MSG_TERM_H_INCLUDED__
#define IPU_MSG_TERM_H_INCLUDED__

/**
 * @addtogroup ipu_msg_abi
 * @{
 */

#include "ipu_msg_header.h"
#include "ia_gofo_common_abi.h"

#pragma pack(push, 1)

/**
 * Enumeration of terminal types
 * @todo As terminals are described by their attributes in the master
 * graph, it is unclear if this is at all necessary
 */
enum ipu_msg_term_type {
	/** Type zero is always padding */
	IPU_MSG_TERM_TYPE_PAD = 0,
	/** Generic terminal - and the base type for all others */
	IPU_MSG_TERM_TYPE_BASE,
	/** Number of terminal types */
	IPU_MSG_TERM_TYPE_N,
};

/**
 *  Terminal events that FW will reflect to client SW.

 *  If an event is not directly available in hardware, firmware may
 *  internally and silently work-around this at its discretion.
 */
/** No event notification request  */
#define IPU_MSG_TERM_EVENT_TYPE_NONE		0U
/** Progress event. */
#define IPU_MSG_TERM_EVENT_TYPE_PROGRESS	1U
/** Number of terminal event types   */
#define IPU_MSG_TERM_EVENT_TYPE_N			(IPU_MSG_TERM_EVENT_TYPE_PROGRESS + 1U)

/**
 * Base terminal structure
 *
 * Note the lack of an options field.
 * This was omitted to save space as most terminals probably won't need this.
 * This may be added as a subclass extension or alternatively,
 * node options could be used instead at the price
 * of specifying the terminal ID in the option.
 */
struct ipu_msg_term {
	/** Type here is one of ipu_msg_term_type. */
	struct ia_gofo_tlv_header tlv_header;
	/**
	 *  ID of terminal in the static node description.
	 *  @note Only connect terminals support events.
	*/
	uint8_t term_id;
	/**
	 *  Request terminal event.  See IPU_MSG_TERM_EVENT_TYPE_BIT macros.
	 *
	 *  FW will send an event notification message to client SW when the event occurs.
	 *  When multiple event types are supported, they may be requested together by
	 *  bitwise OR'ing their bitmapped values here.
	 */
	uint8_t event_req_bm;
	/** Reserved for alignment padding.  Set to zero. */
	uint8_t reserved[2];
	/**
	 *  Size of payload associated with this terminal without any compression that may
	 *  optionally be applied, if supported.
	 */
	uint32_t payload_size;
	/** Terminal options */
	struct ia_gofo_tlv_list term_options;
};

/**
 * Enumeration of known terminal options
 * Terminal meta-data can be attached to the terminal through these options.
 */
enum ipu_msg_term_option_types {
	IPU_MSG_TERM_OPTION_TYPES_PADDING = 0,
	/** Number of terminal options */
	IPU_MSG_TERM_OPTION_TYPES_N
};

/**
 *  Terminal event notification message.  Sent when an requested event occurs.
 *
 *  See ipu_msg_term::event_req_bm
 */
struct ipu_msg_term_event {
	/**
	 *  ABI message header.  Type will be IPU_MSG_TYPE_TERM_EVENT.
	 *  user_token will be taken from the task wherein the event occurred.
	 */
	struct ia_gofo_msg_header header;
	/**  Identifier of the graph containing the node context */
	uint8_t graph_id;
	/**  Identifier of the frame where the event occurred */
	uint8_t frame_id;
	/** Node context identifier as specified in the graph open message */
	uint8_t node_ctx_id;
	/**
	 *  Index of node context configuration profile (variant) for
	 *  the node task where the event occurred
	 */
	uint8_t profile_idx;
	/**  Identifier of the fragment where the event occurred */
	uint8_t frag_id;
	/** ID of terminal in the static node description. */
	uint8_t term_id;
	/** Terminal event who's occurrence is now being notified.  See ipu_msg_term_event_type.  */
	uint8_t event_type;
	/** Reserved for alignment padding.  Set to zero. */
	uint8_t reserved[1];

	/**
	 * Event Timestamp.
	 *
	 * Should be set as close as possible to the event that triggered this notification message. Units
	 * are at the discretion of the sender, and there is no guarantee of time source sync or a
	 * proportional relationship with the client SW clock or even the stability of the time source.
	 *
	 * With all these caveats, this timestamp is only debug aid at best and should not be relied on for
	 * operational use.  The client SW can use the reception time of the message as the next best thing,
	 * if necessary.
	 */
	uint64_t event_ts;
};

#pragma pack(pop)

/** @} */

/** Compile time checks only - this function is not to be called! */
static inline void ipu_msg_term_test_func(void)
{
	CHECK_ALIGN32(struct ipu_msg_term);
	CHECK_ALIGN64(struct ipu_msg_term_event);
}

#endif

