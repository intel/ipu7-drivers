// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2013 - 2024 Intel Corporation

#ifndef IPU_MSG_ERR_CODES_H_INCLUDED__
#define IPU_MSG_ERR_CODES_H_INCLUDED__

/**
 * @addtogroup ipu_msg_abi
 * @{
 */

#include "ia_gofo_msg_err_codes.h"

/**
 *  Error groups define broad categories of errors in ack messages
 *  @todo log messages too?
 *  These are the basic groups common to all applications based on
 *  this messaging protocol.
 *  Some groups ID are reserved for application specific extensions.
 *  See IPU_MSG_ERR_GROUP_APP_EXT_START
 */
enum ipu_msg_err_groups {
	/** Reserved value.  Used only with IPU_MSG_ERR_OK & IPU_MSG_ERR_UNSPECIFED */
	IPU_MSG_ERR_GROUP_RESERVED = IA_GOFO_MSG_ERR_GROUP_RESERVED,
	/** Generic message general errors that aren't specific to another group */
	IPU_MSG_ERR_GROUP_GENERAL = IA_GOFO_MSG_ERR_GROUP_GENERAL,
	/** Device open/close errors */
	IPU_MSG_ERR_GROUP_DEVICE = 2,
	/** Graph open/close errors */
	IPU_MSG_ERR_GROUP_GRAPH = 3,
	/** Task request errors */
	IPU_MSG_ERR_GROUP_TASK = 4,

	/**
	 *  Number of items in this enumeration.
	 *  Cannot be larger than IPU_MSG_ERR_GROUP_APP_EXT_START
	 */
	IPU_MSG_ERR_GROUP_N,
};

/** @} */

#endif
