// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2013 - 2024 Intel Corporation

#ifndef IA_GOFO_TLV_UTILS_H_INCLUDED__
#define IA_GOFO_TLV_UTILS_H_INCLUDED__

#include <stddef.h>
#include <linux/types.h>
#include <stdbool.h>
#include "ia_gofo_abi_platform.h"
#include "ia_gofo_tlv.h"

#define TLV_MASK_BITS(field, bits_mask) ((field) & (bits_mask))

static inline void ia_gofo_tlv_header_set_size(struct ia_gofo_tlv_header *tlv_header,
					uint32_t tlv_len_bytes)
{
	IA_GOFO_ASSERT(IA_GOFO_MODULO((uintptr_t)tlv_header, TLV_ITEM_ALIGNMENT) == 0U);
	IA_GOFO_ASSERT(IA_GOFO_MODULO(tlv_len_bytes, TLV_ITEM_ALIGNMENT) == 0U);
	IA_GOFO_ASSERT(tlv_len_bytes <= TLV_MAX_LEN);

	tlv_header->tlv_len32 = (uint16_t)TLV_MASK_BITS((tlv_len_bytes / TLV_ITEM_ALIGNMENT), 0xFFFFU);
}

static inline void ia_gofo_tlv_header_init(struct ia_gofo_tlv_header *tlv_header,
					uint32_t tlv_type, uint32_t tlv_len_bytes)
{
	IA_GOFO_ASSERT(IA_GOFO_MODULO((uintptr_t)tlv_header, TLV_ITEM_ALIGNMENT) == 0U);
	IA_GOFO_ASSERT(IA_GOFO_MODULO(tlv_len_bytes, TLV_ITEM_ALIGNMENT) == 0U);

	tlv_header->tlv_type = TLV_MASK_BITS((uint16_t)tlv_type, 0x3FU);
	ia_gofo_tlv_header_set_size(tlv_header, tlv_len_bytes);
}

static inline uint32_t ia_gofo_tlv_header_get_size(const struct ia_gofo_tlv_header *tlv_header)
{
	uint32_t tlv_len_bytes = tlv_header->tlv_len32 * TLV_ITEM_ALIGNMENT;

	return tlv_len_bytes;
}

static inline uint32_t ia_gofo_tlv_header_get_type(const struct ia_gofo_tlv_header *tlv_header)
{
	uint32_t tlv_len_bytes = tlv_header->tlv_type;

	return tlv_len_bytes;
}

static inline void ia_gofo_tlv_header_parse(const struct ia_gofo_tlv_header *tlv_header,
					uint32_t *tlv_type, uint32_t *tlv_len_bytes)
{
	*tlv_type = ia_gofo_tlv_header_get_type(tlv_header);
	*tlv_len_bytes = ia_gofo_tlv_header_get_size(tlv_header);
}

/**
 * Initialize a list of TLV fields where the offset value is explicitly set
 * Note that head_offset is relative to tlv_list.  @see ia_gofo_msg_tlv_list_init_by_ptr()
 */
static inline void ia_gofo_tlv_list_init(struct ia_gofo_tlv_list *tlv_list, uint16_t head_offset)
{
	IA_GOFO_ASSERT(IA_GOFO_MODULO(head_offset, TLV_LIST_ALIGNMENT) == 0U);
	tlv_list->num_elems = 0U;
	tlv_list->head_offset = head_offset;
}

/**
 * Initialize a list of TLV fields where the offset value set based on its memory location
 * As tlv_list->head_offset is relative to tlv_list, this form may be more convenient
 */
static inline void ia_gofo_tlv_list_init_by_ptr(struct ia_gofo_tlv_list *tlv_list, const void *head_offset_ptr)
{
	IA_GOFO_ASSERT(IA_GOFO_MODULO(((uintptr_t)head_offset_ptr), TLV_LIST_ALIGNMENT) == 0U);
	tlv_list->num_elems = 0U;
	tlv_list->head_offset = TLV_MASK_BITS((uint16_t)((uintptr_t)head_offset_ptr -
							(uintptr_t)tlv_list), 0xFFFFU);
}

static inline const struct ia_gofo_tlv_header *ia_gofo_tlv_list_get_ptr(
	const struct ia_gofo_tlv_list *tlv_list, uint16_t *num_elems, uint16_t *head_offset)
{
	const struct ia_gofo_tlv_header *head_offset_ptr;

	*num_elems = tlv_list->num_elems;
	*head_offset = tlv_list->head_offset;
	const uint8_t *tmp_arr = (const uint8_t *)tlv_list;

	head_offset_ptr = (const struct ia_gofo_tlv_header *)&tmp_arr[tlv_list->head_offset];

	return head_offset_ptr;
}

/**
 * Returns the size of a tlv object, taking the various tlv header variants into account
 * Will return zero if the tlv header is not legal
 */
static inline uint32_t ia_gofo_tlv_get_size(const struct ia_gofo_tlv_header *tlv_object)
{
	uint32_t size_bytes = tlv_object->tlv_len32 * TLV_ITEM_ALIGNMENT;

	return size_bytes;
}

/** Returns a pointer to the memory just after the tlv object or NULL on error */
static inline const struct ia_gofo_tlv_header *ia_gofo_tlv_get_next(const struct ia_gofo_tlv_header *tlv_object)
{
	const uint8_t *tmp_arr = (const uint8_t *)tlv_object;
	const struct ia_gofo_tlv_header *retval = (const struct ia_gofo_tlv_header *)&tmp_arr[ia_gofo_tlv_header_get_size(tlv_object)];

	return retval;
}

/** Copy a TLV object just after the current one */
struct ia_gofo_tlv_header *ia_gofo_tlv_append(const struct ia_gofo_tlv_header *tlv_object_base,
					const struct ia_gofo_tlv_header *tlv_object_append_src);

/** Increment the count of objects in the list */
static inline void ia_gofo_tlv_list_increment(struct ia_gofo_tlv_list *list_root)
{
	list_root->num_elems++;
}

/** Returns a pointer to the memmory just AFTER the tlv list */
static inline const struct ia_gofo_tlv_header *ia_gofo_tlv_list_get_end(const struct ia_gofo_tlv_list *list_root)
{
	/* Assuming here that list root has been initialized */
	IA_GOFO_ASSERT(list_root->head_offset != 0U);
	const uint8_t *tmp_arr = (const uint8_t *)list_root;

	const struct ia_gofo_tlv_header *append_ptr = (const struct ia_gofo_tlv_header *)&tmp_arr[list_root->head_offset];

	uint16_t i;

	for (i = 0U; i < list_root->num_elems; i++) {
		append_ptr = ia_gofo_tlv_get_next(append_ptr);
	}

	return append_ptr;
}

/**
 * Append an UINITIALIZED tlv object to a list
 * This function updates the link-list meta data the list.
 * The content of the returned object pointer will then need to be populated.
 * Not doing so will result in undefined behavior.
 * @return Pointer to the appended object or NULL on failure
 */
static inline const struct ia_gofo_tlv_header *ia_gofo_tlv_list_append(struct ia_gofo_tlv_list *list_root)
{
	const struct ia_gofo_tlv_header *append_ptr = ia_gofo_tlv_list_get_end(list_root);

	ia_gofo_tlv_list_increment(list_root);

	return append_ptr;
}

/**
 * Append a *copy* of a tlv object to a list
 * @return Pointer to the appended object or NULL on failure
 */
static inline const struct ia_gofo_tlv_header *ia_gofo_tlv_list_append_copy(
	struct ia_gofo_tlv_list *list_root, const struct ia_gofo_tlv_header *tlv_object)
{
	const struct ia_gofo_tlv_header *append_ptr = ia_gofo_tlv_list_append(list_root);

	IA_GOFO_ASSERT(append_ptr);

	append_ptr = ia_gofo_tlv_append(append_ptr, tlv_object);
	ia_gofo_tlv_list_increment(list_root);

	return append_ptr;
}

#endif
