// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2013 - 2024 Intel Corporation

#ifndef IA_GOFO_TLV_H_INCLUDED__
#define IA_GOFO_TLV_H_INCLUDED__

/**
 *  @addtogroup ia_gofo_common_abi
 *  @{
 */

/**
 *  @file
 *  Defines the Type Length Value (TLV) structures and some helper macros
 *  for them.  TLV's are used as a generic header in messages AND
 *  in their component parts.
 *  @see ia_gofo_tlv_header
 */

#include <linux/types.h>
#include "ia_gofo_common_abi.h"
#include "ia_gofo_align_utils.h"

/**  Reserved TLV types for all parsing contexts */
#define TLV_TYPE_PADDING (0U)

#pragma pack(push, 1)

/** Applies the sizeof operator to a member field */
#define IA_GOFO_ABI_SIZEOF_FIELD(struct_type, field) (sizeof(((struct_type *)0)->field))

/**
 *  Number of bits in a byte.  Bytes here assumed to be octects.
 *  This macro is for clarity to make expressions more expressive
 */
#define IA_GOFO_ABI_BITS_PER_BYTE (8U)

/**
 *  Header for type-length-value fields
 *  Placement must be aligned to TLV_MSG_ALIGNMENT
 *  Total length must be a multiple of TLV_ITEM_ALIGNMENT bytes
 *  (and not a multiple of the the stricter TLV_MSG_ALIGNMENT).
 */
struct ia_gofo_tlv_header {
	/**  Type of this field.  Subject to parsing context. */
	uint16_t tlv_type;
	/**
	 *  Size of this field in TLV_ITEM_ALIGNMENT elements, such that the base of this
	 *  structure + (tlv_len32 * TLV_ITEM_ALIGNMENT) will be the next tlv field in a list
	 *
	 *  See TLV_MAX_LEN and TLV_ITEM_ALIGNMENT
	 */
	uint16_t tlv_len32;
};

/**
 * Maximum number of *bytes* a TLV item can contain
 * Note: there is (uint32_t) cast for (1U) as WA for the KW checker
 * MISRA.SHIFT.RANGE.201 which interprets 1U's type as unsigned char rather
 * than unsigned int.
 */
#define TLV_MAX_LEN \
	((((uint32_t)(1U)) << (IA_GOFO_ABI_SIZEOF_FIELD(struct ia_gofo_tlv_header, tlv_len32) * IA_GOFO_ABI_BITS_PER_BYTE)) \
	* TLV_ITEM_ALIGNMENT)

/**
 * Root of a list of TLV fields
 * See TLV_LIST_ALIGNMENT
 */
struct ia_gofo_tlv_list {
	/** Number of elements in the TLV list */
	uint16_t num_elems;
	/**
	 *  Offset from the base of the *this* root structure (not the surrounding
	 *  structure), in bytes, pointing to the first element in the TLV list
	 *  @note Must be aligned to TLV_LIST_ALIGNMENT
	 */
	uint16_t head_offset;
};

/**
 *  Minimum alignment for all TLV items, both for starting address and size.
 *  But see also TLV_MSG_ALIGNMENT
 */
#define TLV_ITEM_ALIGNMENT ((uint32_t)sizeof(uint32_t))

/**
 *  Minimum alignment for all top level TLV items (messages), both for starting address and size
 *  This is needed because those items are allowed to have 64 bit integers within and we need to
 *  remove the possibility of misaligning the fields of an othewise correctly built structure.
 *
 */
#define TLV_MSG_ALIGNMENT ((uint32_t)sizeof(uint64_t))

/**
 *  Alignment required for both start and total size of a TLV list
 *
 *  All TLV lists head_offset must conform to alignment AND
 *  all TLV lists must be padded with zero's after the end of the last item of the list
 *  to this same alignment to ensure that any following TLV items are also
 *  aligned without having to think about it too much.
 *
 *  The alignment value is chosen to be general (strict) enough for all TLV requirements.
 *  This is a potentially a bit wasteful, but prevents issues
 *  that could arise.  Assuming here that we don't have that many lists in a single message
 *  and/or that the items they contain result in a more or less aligned list size anyway.
 */
#define TLV_LIST_ALIGNMENT TLV_ITEM_ALIGNMENT

#pragma pack(pop)

/**  Compile time checks only - this function is not to be called! */
static inline void ia_gofo_msg_tlv_test_func(void)
{
	CHECK_ALIGN16(struct ia_gofo_tlv_header);
	CHECK_ALIGN32(struct ia_gofo_tlv_list);
}

/**  @} */

#endif
