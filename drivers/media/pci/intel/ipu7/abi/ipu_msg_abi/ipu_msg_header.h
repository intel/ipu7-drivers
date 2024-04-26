// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2013 - 2024 Intel Corporation

#ifndef IPU_MSG_HEADER_H_INCLUDED__
#define IPU_MSG_HEADER_H_INCLUDED__

#include "ia_gofo_msg_header.h"
/**
 *  @addtogroup ipu_msg_abi
 *  @{
 */

#pragma pack(push, 1)

/**  Message type enumeration for the message headers */
enum ipu_msg_type {
	/**  Type 0 is padding, which is irrelevant here. */
	IPU_MSG_TYPE_RESERVED = IA_GOFO_MSG_TYPE_RESERVED,

	/** See ia_gofo_msg_indirect */
	IPU_MSG_TYPE_INDIRECT = IA_GOFO_MSG_TYPE_INDIRECT,

	/** See ia_gofo_msg_log */
	IPU_MSG_TYPE_DEV_LOG = IA_GOFO_MSG_TYPE_LOG,

	/** See ia_gofo_msg_general_err */
	IPU_MSG_TYPE_GENERAL_ERR = IA_GOFO_MSG_TYPE_GENERAL_ERR,

	IPU_MSG_TYPE_DEV_OPEN = 4,
	IPU_MSG_TYPE_DEV_OPEN_ACK = 5,
	IPU_MSG_TYPE_GRAPH_OPEN = 6,
	IPU_MSG_TYPE_GRAPH_OPEN_ACK = 7,
	IPU_MSG_TYPE_TASK_REQ = 8,
	IPU_MSG_TYPE_TASK_DONE = 9,
	IPU_MSG_TYPE_GRAPH_CLOSE = 10,
	IPU_MSG_TYPE_GRAPH_CLOSE_ACK = 11,
	IPU_MSG_TYPE_DEV_CLOSE = 12,
	IPU_MSG_TYPE_DEV_CLOSE_ACK = 13,
	IPU_MSG_TYPE_TERM_EVENT = 14,
	IPU_MSG_TYPE_N,
};

#pragma pack(pop)
/**  @} */
#endif
