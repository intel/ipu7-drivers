// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2013 - 2024 Intel Corporation

#ifndef IA_GOFO_MSG_ERR_H_INCLUDED__
#define IA_GOFO_MSG_ERR_H_INCLUDED__

/**
 *  @addtogroup ia_gofo_msg_abi
 *  @{
 */

#include <linux/types.h>
#include "ia_gofo_align_utils.h"

/** Number of elements in ia_gofo_msg_err::err_detail */
#define IA_GOFO_MSG_ERR_MAX_DETAILS (4U)

/** All uses of ia_gofo_msg_err must define err_code such that value 0 is not an error */
#define IA_GOFO_MSG_ERR_OK (0U)

/**
 *  This value is reserved, but should never be sent on the ABI
 *  Used as a placeholder for defensive programming techniques
 *  that set error "NOT_OK" state as default and only set "OK" when that
 *  can be verified.
 *
 *  If received in an ABI message, it is likely a bug in the programming.
 */
#define IA_GOFO_MSG_ERR_UNSPECIFED (0xFFFFFFFFU)

/**
 * All uses of ia_gofo_msg_err must define err_code such that this value is reserved.
 * Companion value to IA_GOFO_MSG_ERR_UNSPECIFED
 */
#define IA_GOFO_MSG_ERR_GROUP_UNSPECIFIED (0U)

/** Quick test for error code */
#define IA_GOFO_MSG_ERR_IS_OK(err) (IA_GOFO_MSG_ERR_OK == (err).err_code)

#pragma pack(push, 1)

/**
 * IPU messaging error structure, used to return errors or generic results in
 * ACK/DONE responses.
 *
 * This structure is generic and can be used in different contexts where the
 * meaning of the rest of the fields is dependent on err_group.
 */
struct ia_gofo_msg_err {
	/** Grouping of the error code.  @note Groups are domain specific. */
	uint32_t err_group;
	/**
	 *  Main error code.
	 *  Numbering depends on err_group, BUT zero must always mean OK (NO ERROR)
	 *  See IA_GOFO_MSG_ERR_OK
	 */
	uint32_t err_code;
	/** Error parameters.  Meaning depends on err_group and err_code */
	uint32_t err_detail[IA_GOFO_MSG_ERR_MAX_DETAILS];
};

#pragma pack(pop)

/**
 * Application extension starting group ID
 *
 * Applications may define their own group ID's starting at this value.
 */
#define IA_GOFO_MSG_ERR_GROUP_APP_EXT_START (16U)

/**
 *  Maximum group ID that will be on the ABI.  Defining this allows applications the option
 *  to use higher values internally with a shared error infrastructure.
 */
#define IA_GOFO_MSG_ERR_GROUP_MAX (31U)

/** See IA_GOFO_MSG_ERR_GROUP_MAX */
#define IA_GOFO_MSG_ERR_GROUP_INTERNAL_START (IA_GOFO_MSG_ERR_GROUP_MAX + 1U)

/** @} */

/** Compile time checks only - this function is not to be called! */
static inline void ia_gofo_msg_err_test_func(void)
{
	CHECK_ALIGN64(struct ia_gofo_msg_err);
}

#endif
