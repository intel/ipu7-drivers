// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2013 - 2024 Intel Corporation

#ifndef INSYS_MSG_ERR_CODES_H_INCLUDED__
#define INSYS_MSG_ERR_CODES_H_INCLUDED__

/**
 * @addtogroup insys_msg_abi
 * @{
 */

#include "ia_gofo_msg_err_codes.h"

/**
 *  Error groups define broad categories of errors in ack messages.
 *  This enumeration defines the error groups for INSYS.
 */
enum insys_msg_err_groups {
	/** Reserved value.  Used only with IA_GOFO_MSG_ERR_OK & IA_GOFO_MSG_ERR_UNSPECIFED */
	INSYS_MSG_ERR_GROUP_RESERVED = IA_GOFO_MSG_ERR_GROUP_RESERVED,
	/**
	 *  Generic message general errors that aren't specific to another group.  See ia_gofo_msg_err_general for
	 *  individual errors.
	 */
	INSYS_MSG_ERR_GROUP_GENERAL = IA_GOFO_MSG_ERR_GROUP_GENERAL,
	/** Stream open/start/flush/abort/stop messages errors.  See insys_msg_err_stream for individual errors. */
	INSYS_MSG_ERR_GROUP_STREAM = 2,
	/** Capture messages errors.  See insys_msg_err_capture for individual errors. */
	INSYS_MSG_ERR_GROUP_CAPTURE = 3,

	/**
	 *  Number of items in this enumeration.
	 */
	INSYS_MSG_ERR_GROUP_N,
};

/** @} */

#endif
