// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2013 - 2024 Intel Corporation

#ifndef IA_GOFO_MSG_HEADER_H_INCLUDED__
#define IA_GOFO_MSG_HEADER_H_INCLUDED__

/**
 *  @addtogroup ia_gofo_msg_abi
 *  @{
 */

#include "ia_gofo_tlv.h"
#include "ia_gofo_msg_err.h"

#pragma pack(push, 1)

/**  Type 0 is padding, which is irrelevant here */
#define IA_GOFO_MSG_TYPE_RESERVED 0
/** See ia_gofo_msg_indirect */
#define IA_GOFO_MSG_TYPE_INDIRECT 1
/** See ia_gofo_msg_log */
#define IA_GOFO_MSG_TYPE_LOG 2
/**
 *  This type is for returning general errors,
 *  such as header errors and SYSCOM  failures
 */
#define IA_GOFO_MSG_TYPE_GENERAL_ERR 3

/** Generic header for all message types */
struct ia_gofo_msg_header {
	/**  All messages are top level TLV "items" */
	struct ia_gofo_tlv_header tlv_header;
	/**  Additional information.  Option type ID's are scoped to the message type */
	struct ia_gofo_tlv_list msg_options;
	/**
	 *  Set by SW for commands,
	 *  and same will be returned by FW in that command's response(s)
	 */
	uint64_t user_token;
};

/** Generic header for all the ack message types */
struct ia_gofo_msg_header_ack {
	/**  Common ABI message header.  Type depends on the kind of response message. */
	struct ia_gofo_msg_header header;
	/**
	 *  Generic error structure.  err_header.error_code == 0 means ACK,
	 *  all others are an error
	 */
	struct ia_gofo_msg_err err;

};

/**
 *  Generic error message with its own message type.
 *  Dedicated for reporting general message errors, such as invalid message type in the header.
 */
struct ia_gofo_msg_general_err {
	/**  Common ack header */
	struct ia_gofo_msg_header_ack header;

};

#pragma pack(pop)

/**  @} */

/**  Compile time checks only - this function is not to be called! */
static inline void ia_gofo_msg_header_test_func(void)
{
	CHECK_ALIGN64(struct ia_gofo_msg_header);
	CHECK_ALIGN64(struct ia_gofo_msg_header_ack);
	CHECK_ALIGN64(struct ia_gofo_msg_general_err);
}

#endif
