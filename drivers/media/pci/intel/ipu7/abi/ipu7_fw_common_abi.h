/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2020 - 2024 Intel Corporation
 */

#ifndef IPU7_FW_COMMOM_ABI_H
#define IPU7_FW_COMMOM_ABI_H

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

#ifndef IA_GOFO_MODULO
/**
 *  Simple modulo.  Required due to way some assert macro's work where the expression
 *  is string-ified and then the percent sign could be erroneously interpreted as part of a
 *  format string.
 */
#define IA_GOFO_MODULO(dividend, divisor) ((dividend) % (divisor))
#endif

#ifndef IA_GOFO_ASSERT
/** Empty definition to allow headers to compile if NO_IA_GOFO_ABI_PLATFORM_DEPS is defined */
#define IA_GOFO_ASSERT(expr)
#endif

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

#pragma pack(push, 1)

/** Data movement mechanisms between producer and consumer */
enum ia_gofo_msg_link_streaming_mode {
	/**
	 * Static Offline.
	 * This value is required to be set when FW is responsible to manage the data flow
	 * accros a link without streaming hardware support.
	 *
	 * If SW manages the timing of this data flow by holding back task requests, it
	 * should not define as a link on this interface.
	 */
	IA_GOFO_MSG_LINK_STREAMING_MODE_SOFF = 0,
	/** Dynamic Offline */
	IA_GOFO_MSG_LINK_STREAMING_MODE_DOFF = 1,
	/** Buffer chasing large memory */
	IA_GOFO_MSG_LINK_STREAMING_MODE_BCLM = 2,
	/** Buffer chasing small memory with fix starting point */
	IA_GOFO_MSG_LINK_STREAMING_MODE_BCSM_FIX = 3,
	/** Number of items in this enumeration */
	IA_GOFO_MSG_LINK_STREAMING_MODE_N
};

/**
 * SOC PBKs.
 * Enumeration of all HW PBKs instances which are external to specifics IPs.
 * Those PBK used for cross IPs connection.
 */
enum ia_gofo_soc_pbk_instance_id {
	IA_GOFO_SOC_PBK_ID0 = 0,
	IA_GOFO_SOC_PBK_ID1 = 1,
	IA_GOFO_SOC_PBK_ID_N
};

/** Maxinum number of PBK slots that may be allocated per link  */
#define IA_GOFO_MSG_LINK_PBK_MAX_SLOTS (2U)

#pragma pack(pop)

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

#pragma pack(push, 1)

/** Maximum number of additional parameters per log message */
#define IA_GOFO_MSG_LOG_MAX_PARAMS (4U)

/** Reserved format ID range minimum for documented "official" (non-debug) messages */
#define IA_GOFO_MSG_LOG_DOC_FMT_ID_MIN (0U)

/** Reserved format ID range maximum for documented "official" (non-debug) messages */
#define IA_GOFO_MSG_LOG_DOC_FMT_ID_MAX (4095U)

/** Reserved format ID for invalid state */
#define IA_GOFO_MSG_LOG_FMT_ID_INVALID (0xFFFFFFFU)

/**
 *  @brief Log message information structure.  Contains the details of a log message
 *  with the exception of the message timestamp.
 *
 *  The timestamp field is not included to allow an *option* for reuse internally in
 *  firmware with those channels where hardware prepends the timestamp automatically.
 *
 *  @see ia_gofo_msg_log_info_ts
 */
struct ia_gofo_msg_log_info {
	/**
	 * Running counter of log messages.
	 * Helps detect lost messages
	 */
	uint16_t log_counter;
	/**
	* Indicates the types of the parameters.
	* 64 or 32 bits parameters.
	*/
	uint8_t msg_parameter_types;
	/**
	 * messages which can't be printed (due to limition like printing from irqs)
	 * will be queued in inner FW OS queue and printed later.
	 * mark those messages as out of order.
	 * Note: the log can be sorted by ts to get order log.
	 * ** Messages which printed before the channel (for example syscom)
	 * init will be queued until the channel will be ready
	 * ** Messages which can't be printed due to the channel limitions
	 * (for example syscom can't print from irq) will be queued and printed from idle thread
	 */
	uint8_t is_out_of_order;
	/**
	 * Unique identifier of log message.
	 * If the ID is between the range of
	 *
	 * IA_GOFO_MSG_LOG_DOC_FMT_ID_MIN =< fmt_id =< IA_GOFO_MSG_LOG_DOC_FMT_ID_MAX,
	 * it is a documented "official" (i.e. non-debug) log messages and the
	 * fmt_id is then a key to a documented string table defined per src_id.
	 * otherwise, the message is a "debug" message and the ID needs to be
	 * resolved against a dictionary generated at build time.
	 * Also see IA_GOFO_MSG_LOG_FMT_ID_INVALID
	 */
	uint32_t fmt_id;
	/**
	 * Log message parameters.
	 * Can be combined with a string lookup
	 * based on the fmt_id for readable output
	 */
	uint32_t params[IA_GOFO_MSG_LOG_MAX_PARAMS];
};

/**
 *  @brief Log message information structure.  Contains the details of a log message
 *  including the log timestamp.
 *
 *  This structure contains the payload information for IA_GOFO log messages, but without
 *  any message header.
 *
 *  This structure is separate from the containing message (See ia_gofo_msg_log) to allow
 *  its reuse with other log channels, for example a hardware logging unit, that have a
 *  different message header (or none at all) than the ia_gofo_msg_header used in
 *  ia_gofo_msg_log.
 */
struct ia_gofo_msg_log_info_ts {
	/**
	 * Message Timestamp.  Should be set as close as possible to whatever
	 * triggered the log message.
	 * Units are at the discretion of the sender, but the time source must be
	 * monotonously increasing and no two messages in-flight at the same time
	 * may have the same timestamp.
	 * @todo Optimally, the time source will be synchronized
	 * with other debug sources, such as the hardware trace unit.
	 */
	uint64_t msg_ts;

	/** Log message information */
	struct ia_gofo_msg_log_info log_info;
};

/**
 * Log reporting message, sent from IPU to host
 *
 * @note The log message severity and source ID's are part of the log meta-data referenced by the
 * fmt_id and not expressed explicitly in this log message.
 */
struct ia_gofo_msg_log {
	/**
	 * Common ABI message header.
	 * Type will be IA_GOFO_MSG_TYPE_DEV_LOG
	 */
	struct ia_gofo_msg_header header;

	/** Log message information including a timestamp */
	struct ia_gofo_msg_log_info_ts log_info_ts;
};

#pragma pack(pop)

/** @} */

/** Compile time checks only - this function is not to be called! */
static inline void ia_gofo_msg_log_test_func(void)
{
	CHECK_ALIGN64(struct ia_gofo_msg_log);
	CHECK_ALIGN64(struct ia_gofo_msg_log_info);
	CHECK_ALIGN64(struct ia_gofo_msg_log_info_ts);
}

/**
 * Output queue for acknowledge/done messages
 * This queue is opened by firmware at startup
 */
#define IA_GOFO_MSG_ABI_OUT_ACK_QUEUE_ID (0U)

/**
 * Output queue for log messages, including error messages
 * This queue is opened by firmware at startup
 */
#define IA_GOFO_MSG_ABI_OUT_LOG_QUEUE_ID (1U)

/**
 * Input queue for device level messages
 * This queue is opened by firmware at startup
 */
#define IA_GOFO_MSG_ABI_IN_DEV_QUEUE_ID (2U)

#endif