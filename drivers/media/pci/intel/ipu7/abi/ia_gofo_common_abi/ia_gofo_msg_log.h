// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2013 - 2024 Intel Corporation

#ifndef IA_GOFO_MSG_LOG_H_INCLUDED__
#define IA_GOFO_MSG_LOG_H_INCLUDED__

/**
 *  @addtogroup ia_gofo_msg_abi
 *  @{
 */
#include "ia_gofo_msg_header.h"

#pragma pack(push, 1)

/** Maximum number of additional parameters per log message */
#define IA_GOFO_MSG_LOG_MAX_PARAMS (4U)

/** Reserved format ID range minimum for documented "official" (non-debug) messages */
#define IA_GOFO_MSG_LOG_DOC_FMT_ID_MIN (0U)

/** Reserved format ID range maximum for documented "official" (non-debug) messages */
#define IA_GOFO_MSG_LOG_DOC_FMT_ID_MAX (4095U)

/** Reserved format ID for invalid state */
#define IA_GOFO_MSG_LOG_FMT_ID_INVALID (0xFFFFFFFU)

/**
 *  @brief Log message information structure.  Contains the details of a log message
 *  with the exception of the message timestamp.
 *
 *  The timestamp field is not included to allow an *option* for reuse internally in
 *  firmware with those channels where hardware prepends the timestamp automatically.
 *
 *  @see ia_gofo_msg_log_info_ts
 */
struct ia_gofo_msg_log_info {
	/**
	 * Running counter of log messages.
	 * Helps detect lost messages
	 */
	uint16_t log_counter;
	/**
	* Indicates the types of the parameters.
	* 64 or 32 bits parameters.
	*/
	uint8_t msg_parameter_types;
	/**
	 * messages which can't be printed (due to limition like printing from irqs)
	 * will be queued in inner FW OS queue and printed later.
	 * mark those messages as out of order.
	 * Note: the log can be sorted by ts to get order log.
	 * ** Messages which printed before the channel (for example syscom)
	 * init will be queued until the channel will be ready
	 * ** Messages which can't be printed due to the channel limitions
	 * (for example syscom can't print from irq) will be queued and printed from idle thread
	 */
	uint8_t is_out_of_order;
	/**
	 * Unique identifier of log message.
	 * If the ID is between the range of
	 *
	 * IA_GOFO_MSG_LOG_DOC_FMT_ID_MIN =< fmt_id =< IA_GOFO_MSG_LOG_DOC_FMT_ID_MAX,
	 * it is a documented "official" (i.e. non-debug) log messages and the
	 * fmt_id is then a key to a documented string table defined per src_id.
	 * otherwise, the message is a "debug" message and the ID needs to be
	 * resolved against a dictionary generated at build time.
	 * Also see IA_GOFO_MSG_LOG_FMT_ID_INVALID
	 */
	uint32_t fmt_id;
	/**
	 * Log message parameters.
	 * Can be combined with a string lookup
	 * based on the fmt_id for readable output
	 */
	uint32_t params[IA_GOFO_MSG_LOG_MAX_PARAMS];
};

/**
 *  @brief Log message information structure.  Contains the details of a log message
 *  including the log timestamp.
 *
 *  This structure contains the payload information for IA_GOFO log messages, but without
 *  any message header.
 *
 *  This structure is separate from the containing message (See ia_gofo_msg_log) to allow
 *  its reuse with other log channels, for example a hardware logging unit, that have a
 *  different message header (or none at all) than the ia_gofo_msg_header used in
 *  ia_gofo_msg_log.
 */
struct ia_gofo_msg_log_info_ts {
	/**
	 * Message Timestamp.  Should be set as close as possible to whatever
	 * triggered the log message.
	 * Units are at the discretion of the sender, but the time source must be
	 * monotonously increasing and no two messages in-flight at the same time
	 * may have the same timestamp.
	 * @todo Optimally, the time source will be synchronized
	 * with other debug sources, such as the hardware trace unit.
	 */
	uint64_t msg_ts;

	/** Log message information */
	struct ia_gofo_msg_log_info log_info;
};

/**
 * Log reporting message, sent from IPU to host
 *
 * @note The log message severity and source ID's are part of the log meta-data referenced by the
 * fmt_id and not expressed explicitly in this log message.
 */
struct ia_gofo_msg_log {
	/**
	 * Common ABI message header.
	 * Type will be IA_GOFO_MSG_TYPE_DEV_LOG
	 */
	struct ia_gofo_msg_header header;

	/** Log message information including a timestamp */
	struct ia_gofo_msg_log_info_ts log_info_ts;
};

#pragma pack(pop)

/** @} */

/** Compile time checks only - this function is not to be called! */
static inline void ia_gofo_msg_log_test_func(void)
{
	CHECK_ALIGN64(struct ia_gofo_msg_log);
	CHECK_ALIGN64(struct ia_gofo_msg_log_info);
	CHECK_ALIGN64(struct ia_gofo_msg_log_info_ts);
}

#endif
