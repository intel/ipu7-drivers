// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2013 - 2024 Intel Corporation

#ifndef IA_GOFO_MSG_INDIRECT_H_INCLUDED__
#define IA_GOFO_MSG_INDIRECT_H_INCLUDED__

/**
 * @addtogroup ia_gofo_msg_abi
 * @{
 */

#include "ia_gofo_common_abi.h"
#include "ia_gofo_msg_header.h"
#include "ia_gofo_msg_err_codes.h"

#pragma pack(push, 1)

/**
 * Short generic message to redirect message parsing somewhere else where the real
 * message is located.
 */
struct ia_gofo_msg_indirect {
	/**
	 *  Common ABI message header.  Type will be IA_GOFO_MSG_TYPE_INDIRECT
	 *  Exceptionally for this message type, user_data is reserved and must be set to zero.
	 *  The user_data of the referenced message may be used as usual.
	 */
	struct ia_gofo_msg_header header;

	/**  Copy of the TLV part of the message header that this one refers to. */
	struct ia_gofo_tlv_header ref_header;

	/**
	 *  Pointer to the actual message header that this message refers to.
	 *  Header must be of type TLV_HEADER_TYPE_32BIT.
	 *  and alignment must be to TLV_MSG_ALIGNMENT
	 */
	ia_gofo_addr_t ref_msg_ptr;
};

#pragma pack(pop)

/** @} */

/** Compile time checks only - this function is not to be called! */
static inline void ia_gofo_msg_indirect_test_func(void)
{
	CHECK_ALIGN64(struct ia_gofo_msg_indirect);
}

#endif

