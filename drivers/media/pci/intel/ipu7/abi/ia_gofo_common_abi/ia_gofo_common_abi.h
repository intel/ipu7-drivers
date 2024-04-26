// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2013 - 2024 Intel Corporation

#ifndef IA_GOFO_COMMON_ABI_H_INCLUDED__
#define IA_GOFO_COMMON_ABI_H_INCLUDED__

#include <linux/types.h>
#include "ia_gofo_align_utils.h"

/**
 * @defgroup ia_gofo_common_abi Common Elements for IPU ABI's
 * The most basic and common ABI definitions used by a variety of ABI modules
 * @{
 */

#pragma pack(push, 1)

/**
 * @file
 * This file contains the most basic and common ABI definitions
 * used by Host and IPU firmware(s) for communication
 */

#ifndef IA_GOFO_PTR_AS_NATIVE

/**
 * IPU address space pointer type.  Note that it's size is likely different than
 * the host's own pointers.
 */
typedef uint32_t ia_gofo_addr_t;

#else

/**
 * IPU address space pointer type - defined to be the same as the host - useful
 * for simulated unit test or if somehow firmware and host code run on the same
 * processor.
 */
typedef uintptr_t ia_gofo_addr_t;

#endif

/** IPU NULL address */
#define IA_GOFO_ADDR_NULL (0U)

/**
 * Host pointer (at integer)
 * @todo This could change if we work with small "host"
 */
typedef uint64_t host_addr_t;

/**
 * Version structure for semantic versioning
 * See semver.org
 * In the IPU protocols,
 * the version is not valid if both major and minor components are BOTH zero.
 * That is, any variant of the pattern 0.0.x.x is not valid
 * @note Order of fields is optimized to allow comparison of the entire structure contents
 * as a little endien unsigned integer.
 * @see IA_GOFO_MSG_VERSION_INIT
 */
struct ia_gofo_version_s {
	/** Patch number.  Not sure if we'll actually use this... */
	uint8_t patch;
	/** Subminor version number.  Change means a non-breaking bug fix. */
	uint8_t subminor;
	/** Minor version number. Change means non-breaking change (still backwards compatible) */
	uint8_t minor;
	/** Major version number.  Change means a compatibility break */
	uint8_t major;
};

/**
 * Convenience macro to initialize the fields of a version structure
 * This macro is recommended for static initialization as initialization by order is
 * not intuitive due to the ordering of the fields.
 */
#define IA_GOFO_MSG_VERSION_INIT(major_val, minor_val, subminor_val, patch_val) \
	{.major = (major_val), .minor = (minor_val), .subminor = (subminor_val), .patch = (patch_val)}

/** Convenience macro to determine if a version value is valid */
#define IA_GOFO_MSG_VERSION_IS_VALID(version_struct) \
	(!((version_struct).major == 0U) && ((version_struct).minor == 0U))

/**
 * Maximum nuber of versions in the version list
 * @see ia_gofo_msg_version_list
 */
#define IA_GOFO_MSG_VERSION_LIST_MAX_ENTRIES (3U)

/** Maximum nuber of versions  in the version list */
#define IA_GOFO_MSG_RESERVED_SIZE (3U)

/** List of versions */
struct ia_gofo_msg_version_list {
	/** Number of elements in versions field */
	uint8_t num_versions;
	/** Reserved for alignment.  Set to zero. */
	uint8_t reserved[IA_GOFO_MSG_RESERVED_SIZE];
	/** Version array. */
	struct ia_gofo_version_s versions[IA_GOFO_MSG_VERSION_LIST_MAX_ENTRIES];
};

#pragma pack(pop)

/** @} */

/** Compile time checks only - this function is not to be called! */
static inline void ia_gofo_common_abi_test_func(void)
{
	CHECK_ALIGN32(struct ia_gofo_version_s);
	CHECK_ALIGN32(struct ia_gofo_msg_version_list);
}

#endif
