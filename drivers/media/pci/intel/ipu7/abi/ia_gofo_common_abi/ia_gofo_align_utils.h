// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2013 - 2024 Intel Corporation

#ifndef IA_GOFO_ALIGN_UTILS_H_INCLUDED__
#define IA_GOFO_ALIGN_UTILS_H_INCLUDED__

#include <linux/types.h>

/**
 * Generic macro to trap size alignment violations at compile time
 * A violation will result in a compilation error.
 * @param struct_type The structure type to check
 * @param alignment Byte alignment to enforce
 */
#define CHECK_SIZE_ALIGNMENT(struct_type, alignment) do { \
	const uint8_t arr[((sizeof(struct_type) % (alignment)) == 0U) ? 1 : -1]; (void)arr; \
	} while (false)

/** Macro to trap 16 bit size alignment violations at compile time */
#define CHECK_ALIGN16(struct_type) CHECK_SIZE_ALIGNMENT(struct_type, sizeof(uint16_t))

/** Macro to trap 32 bit size alignment violations at compile time */
#define CHECK_ALIGN32(struct_type) CHECK_SIZE_ALIGNMENT(struct_type, sizeof(uint32_t))

/** Macro to trap 64 bit size alignment violations at compile time */
#define CHECK_ALIGN64(struct_type) CHECK_SIZE_ALIGNMENT(struct_type, sizeof(uint64_t))

/**
 * Cache line size in bytes
 * @todo Should probably get cache line size from per-platform header
 */
#define IA_GOFO_CL_SIZE (64U)

/** Macro to trap cache line alignment violations at compile time */
#define CHECK_ALIGN_CL(struct_type) CHECK_SIZE_ALIGNMENT(struct_type, IA_GOFO_CL_SIZE)

/**
 * Memory page size in bytes
 * @todo Should probably get page size from per-platform header
 */
#define IA_GOFO_PAGE_SIZE (4096U)
#define IA_GOFO_SRAM_PAGE_SIZE (256U)

/** Macro to trap memory page violations at compile time */
#define CHECK_ALIGN_PAGE(struct_type) CHECK_SIZE_ALIGNMENT(struct_type, IA_GOFO_PAGE_SIZE)

/** Compile time test of two constant values */
#define CHECK_EQUAL(lval, rval) do { const uint8_t arr[((lval) == (rval)) ? 1 : -1]; (void)arr; } while (false)

/** Compile time test of two constant values */
#define CHECK_SMALLER_OR_EQUAL(lval, rval) do { const uint8_t arr[((lval) <= (rval)) ? 1 : -1]; (void)arr; } while (false)

/** Test the size (in bytes) of a structure */
#define CHECK_SIZE(struct_type, size) CHECK_EQUAL(sizeof(struct_type), size)

#endif

