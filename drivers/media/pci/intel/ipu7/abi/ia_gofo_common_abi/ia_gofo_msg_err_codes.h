// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2013 - 2024 Intel Corporation

#ifndef IA_GOFO_MSG_ERR_CODES_H_INCLUDED__
#define IA_GOFO_MSG_ERR_CODES_H_INCLUDED__

#include "ia_gofo_msg_err.h"

/**
 * @file
 * @brief This file defines error groups and codes that are are common across
 * multiple domains.
 *
 * Error codes related to a specific message may be found in that message's
 * structure header.
 *
 * @addtogroup ia_gofo_msg_abi
 * @{
 */

/** Reserved value.  Used only with IA_GOFO_MSG_ERR_OK & IA_GOFO_MSG_ERR_UNSPECIFED */
#define IA_GOFO_MSG_ERR_GROUP_RESERVED IA_GOFO_MSG_ERR_GROUP_UNSPECIFIED
/** Generic message errors that aren't specific to another group or IP */
#define IA_GOFO_MSG_ERR_GROUP_GENERAL 1

/** Error detail enumeration for error group IA_GOFO_MSG_ERR_GROUP_GENERAL */
enum ia_gofo_msg_err_general {
	/** No error */
	IA_GOFO_MSG_ERR_GENERAL_OK = IA_GOFO_MSG_ERR_OK,
	/**
	 *  Message was smaller than expected for its type.  err_detail[0] is the size, in bytes,
	 *  received. err_detail[1] is the minimum required
	 */
	IA_GOFO_MSG_ERR_GENERAL_MSG_TOO_SMALL = 1,
	/**
	 *  Message was larger than can be supported.  err_detail[0] is the size, in bytes,
	 *  received. err_detail[1] is the maximum supported
	 */
	IA_GOFO_MSG_ERR_GENERAL_MSG_TOO_LARGE = 2,
	/**
	 *  State machine error.  Messge received doesn't fit with device state.
	 *  err_detail[0] will be the actual device state.
	 */
	IA_GOFO_MSG_ERR_GENERAL_DEVICE_STATE = 3,
	/**
	 *  An item in the message is not aligned.  err_detail[0] is the offending item's offset
	 *  into the message.  err_detail[1] is the required alignment.
	 */
	IA_GOFO_MSG_ERR_GENERAL_ALIGNMENT = 4,
	/**
	 *  The referenced address in an indirect message is invalid.  err_detail[0] is the
	 *  offending address.
	 */
	IA_GOFO_MSG_ERR_GENERAL_INDIRECT_REF_PTR_INVALID = 5,
	/**
	 *  The message type in the header is invalid.  err_detail[0] is the received type.
	 */
	IA_GOFO_MSG_ERR_GENERAL_INVALID_MSG_TYPE = 6,
	/**
	 *  The header is NULL. May happen because of an error in SYSCOM,
	 *  e.g. NULL queue parameter or reading fail.
	 *  No err_detail because there was no data to parse.
	 */
	IA_GOFO_MSG_ERR_GENERAL_SYSCOM_FAIL = 7,
	/** Size of enumeration */
	IA_GOFO_MSG_ERR_GENERAL_N
};

/** @} */

#endif
