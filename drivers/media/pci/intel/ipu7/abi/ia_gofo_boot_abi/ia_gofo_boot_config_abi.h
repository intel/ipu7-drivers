// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2013 - 2024 Intel Corporation

#ifndef IA_GOFO_BOOT_CONFIG_ABI_H_INCLUDED__
#define IA_GOFO_BOOT_CONFIG_ABI_H_INCLUDED__

/*
 * Watchdog handler, written in ASM, uses this file to get the value to be stored in a boot status register.
 * Rest of the file will be ignored by assembler to prevent it compiling with C syntax resulting in many errors.
 */
#ifndef __ASSEMBLER__

#include "ia_gofo_common_abi.h"
#include "ia_gofo_syscom_abi.h"
#include "ia_gofo_align_utils.h"

#pragma pack(push, 1)

/**
 * @addtogroup ia_gofo_boot_abi
 * @{
 *
 */

/** Number of FW Boot_Param Registers per application */
#define IA_GOFO_BUTTRESS_FW_BOOT_PARAMS_MAX_REG_IDX_PER_APP ((uint32_t)IA_GOFO_FW_BOOT_ID_MAX)

/** Offset to IS sets of boot parameter registers */
#define IA_GOFO_BUTTRESS_FW_BOOT_PARAMS_IS_OFFSET (0U)

/** Offset to PS sets of boot parameter registers */
#define IA_GOFO_BUTTRESS_FW_BOOT_PARAMS_PS_OFFSET ((IA_GOFO_BUTTRESS_FW_BOOT_PARAMS_IS_OFFSET) + \
	(uint32_t)(IA_GOFO_BUTTRESS_FW_BOOT_PARAMS_MAX_REG_IDX_PER_APP))

/**
 * Offset to primary set of boot parameter registers from subsystem register base.
 * See IA_GOFO_BUTTRESS_FW_BOOT_PARAMS_IS_OFFSET and IA_GOFO_BUTTRESS_FW_BOOT_PARAMS_PS_OFFSET
 */
#define IA_GOFO_BUTTRESS_FW_BOOT_PARAMS_PRIMARY_OFFSET (0U)

/**
 * Offset to secondary set of boot parameter registers from subsystem register base.
 *
 * New in IPU7, the secondary registers are in their own memory
 * page to allow access controls to be defined separately for
 * each security context.  Primary context registers are now
 * directly accesable to the untrusted SW and only the secondary
 * conext regiseters are limited to the trusted SW. Previously,
 * all FW_BOOT_PARAMS registers were accessible only to the
 * trusted SW.
 *
 * The value here is in units of register indexes from IA_GOFO_BUTTRESS_FW_BOOT_PARAMS_PRIMARY_OFFSET.
 *
 * See IA_GOFO_BUTTRESS_FW_BOOT_PARAMS_IS_OFFSET and IA_GOFO_BUTTRESS_FW_BOOT_PARAMS_PS_OFFSET
 */
#define IA_GOFO_CCG_IPU_BUTTRESS_FW_BOOT_PARAMS_SECONDARY_OFFSET (0x3000U / 4U)

/**
 * Offset to Stand-alone SoC (HKR) secondary set of boot parameter registers from subsystem register base.
 * The value here is in units of register indexs from IA_GOFO_BUTTRESS_FW_BOOT_PARAMS_PRIMARY_OFFSET.
 */
#define IA_GOFO_HKR_IPU_BUTTRESS_FW_BOOT_PARAMS_SECONDARY_OFFSET (IA_GOFO_BUTTRESS_FW_BOOT_PARAMS_MAX_REG_IDX_PER_APP * 2U)
/**
 * Offset to Stand-alone SoC (HKR) secondary set of boot parameter registers from subsystem register base.
 * The value here is in units of register indexs from IA_GOFO_BUTTRESS_FW_BOOT_PARAMS_PRIMARY_OFFSET.
 */
#define IA_GOFO_HKR_HIF_BUTTRESS_FW_BOOT_PARAMS_SECONDARY_OFFSET (IA_GOFO_BUTTRESS_FW_BOOT_PARAMS_MAX_REG_IDX_PER_APP)

/** Number in Intel SoC (CCG) of FW Boot Param Registers */
#define IA_GOFO_CCG_IPU_BUTTRESS_FW_BOOT_PARAMS_MAX_REG_IDX (IA_GOFO_BUTTRESS_FW_BOOT_PARAMS_MAX_REG_IDX_PER_APP * 4U)

/** Number in Stand-alone SoC (HKR) of FW Boot Param Registers */
#define IA_GOFO_HKR_IPU_BUTTRESS_FW_BOOT_PARAMS_MAX_REG_IDX (IA_GOFO_BUTTRESS_FW_BOOT_PARAMS_MAX_REG_IDX_PER_APP * 4U)

/** Size of reserved words in boot structure */
#define IA_GOFO_BOOT_RESERVED_SIZE (58U)

/** Size of reserved words in secondary boot structure */
#define IA_GOFO_BOOT_SECONDARY_RESERVED_SIZE (IA_GOFO_BOOT_RESERVED_SIZE)

/** Size of reserved fields in secondary boot structure in bytes.
 * Those fields are reserved for future use and to allow unity with the Primary boot configuration structure.
 */
#define IA_GOFO_BOOT_SECONDARY_RESERVED_FIELDS (sizeof(ia_gofo_addr_t) + sizeof(ia_gofo_addr_t) + sizeof(uint32_t))

/**
 * Boot parameter register index (offset) in register units (4 bytes each)
 * Offset is relative to a base per subsystem and boot context
 */
enum ia_gofo_buttress_reg_id {
	/** pass boot (mostly syscom) configuration to SPC */
	IA_GOFO_FW_BOOT_CONFIG_ID = 0,

	/** syscom state - initialized by host before uC kick, modified by firmware afterwards */
	IA_GOFO_FW_BOOT_STATE_ID = 1,

	/**
	 * Reserved for some future use.  Secondary context only,
	 * occupying the same index as STATE, which is
	 * used only in the primary context as there is only one
	 * system state and one owner at the host.
	 *
	 * Host should initialize to zero in the secondary context, but
	 * only if there is a secondary context. When there is no
	 * secondary context, this register must NOT be written or
	 * read. It may not even be implemented physically in hardware
	 * that does not support security via dual contexts.
	 *
	 * See IA_GOFO_FW_BOOT_STATE_ID
	 */
	IA_GOFO_FW_BOOT_RESERVED1_ID = IA_GOFO_FW_BOOT_STATE_ID,

	/**
	 * Location of indices for syscom queues.  Set by firmware, read by host
	 * This is the base of a contiguous array of syscom index pairs
	 * @note Not valid until IA_GOFO_FW_BOOT_STATE is set to IA_GOFO_FW_BOOT_STATE_READY or
	 * at least IA_GOFO_FW_BOOT_STATE_QUEUE_INIT_DONE
	 */
	IA_GOFO_FW_BOOT_SYSCOM_QUEUE_INDICES_BASE_ID = 2,

#ifndef REDUCED_FW_BOOT_PARAMS
	/**
	 * syscom untrusted addr min - defines range of shared buffer
	 * addresses valid for the primary (untrusted) device context.
	 * Initialized by host before uC kick and read by firmware.
	 *
	 * Usage in the firmware is:
	 *
	 * bool is_addr_untrusted = addr >= untrusted_addr_min;
	 *
	 * where *is_addr_untrusted* must evaluate to true for all
	 * buffers (start and end of each one) passed to the primary
	 * context and false for all buffers passed to the secondary
	 * context.
	 *
	 * Alignment constraint: the value of this register must be
	 * aligned to 4KB pages.
	 *
	 * This register usage has meaning only in the secondary
	 * (trusted) context parameter set, despite it being directly
	 * relevant to the primary (untrusted) context because only in
	 * the secondary context do we trust the value as it comes from
	 * the trusted host entity.  The corresponding register in the
	 * primary context may be assigned a different use in the
	 * future.
	 *
	 * When there is no secondary context, this register must NOT be
	 * written or read.  It may not even be implemented physically
	 * in hardware that does not support security via dual contexts.
	 */
	IA_GOFO_FW_BOOT_UNTRUSTED_ADDR_MIN_ID = 3,

	/**
	 * Reserved for some future use. Primary context only,
	 * occupying the same index as UNTRUSTED_ADDR_MIN, which is
	 * used only in the secondary context.
	 *
	 * Host should initialize to zero (in the primary context).
	 *
	 * See IA_GOFO_FW_BOOT_UNTRUSTED_ADDR_MIN_ID
	 */
	IA_GOFO_FW_BOOT_RESERVED0_ID = IA_GOFO_FW_BOOT_UNTRUSTED_ADDR_MIN_ID,

	/**
	 * Messaging version as declared by firmware.
	 * This is the highest version determined to be compatible with host's capabilities.
	 * If no compatible version can be found, firmware will set its highest supported version.
	 * @note Not valid until IA_GOFO_FW_BOOT_STATE is set to IA_GOFO_FW_BOOT_STATE_READY or
	 * at least IA_GOFO_FW_BOOT_STATE_QUEUE_INIT_DONE
	 */
	IA_GOFO_FW_BOOT_MESSAGING_VERSION_ID = 4,
#else
	/*
	 * Funtional safety SoC products may have fewer FW_BOOT registers,
	 * but on the other hand, they do not need the UNTRUSTED_ADDR_MIN
	 * register.  To get their boot context to fit into the
	 * available registers remove the RESERVED0/UNTRUSTED_ADDR_MIN
	 * assignment from the definition.
	 */

	IA_GOFO_FW_BOOT_MESSAGING_VERSION_ID = 3,
#endif

	/** Number of register ID's */
	IA_GOFO_FW_BOOT_ID_MAX
};

#ifdef REDUCED_FW_BOOT_PARAMS
#define IA_GOFO_FW_BOOT_UNTRUSTED_ADDR_MIN_ID (0xFFU)
#endif

/**
 * Evaluates to true if boot state is critical
 * @see enum ia_gofo_boot_state
 */
#define IA_GOFO_FW_BOOT_STATE_IS_CRITICAL(boot_state) (0xDEAD0000U == ((boot_state) & 0xFFFF0000U))

/**
 * Very minimalistic boot parameters containing the minimum required
 * to setup messaging.  Any further configuration is handled in the
 * messages.
 */
struct ia_gofo_boot_config {
	/** Length of this structure in bytes, including the variable number of queue configs */
	uint32_t length;
	/** Semantic version number of this configuration structure */
	struct ia_gofo_version_s config_version;
	/**
	 * Semantic messaging protocol versions supported by host.
	 * Host will thus declare the protocol versions it
	 * supports, in order of preference
	 */
	struct ia_gofo_msg_version_list client_version_support;
	/**
	 * Package directory location in the IPU address space.
	 * Used only in unsecured boot.  Secure boot location is hard coded to IMR.
	 */
	ia_gofo_addr_t pkg_dir;
	/**
	 * Configuration specific to a subsystem.
	 * Optional. Currently used for debug purposes.
	 */
	ia_gofo_addr_t subsys_config;
	/**
	 * frequency in MHz.
	 */
	uint32_t uc_tile_frequency_mhz;
	/**
	 * 16-bit Checksum value
	 * Used only in INSYS Stand-alone SoC (HKR)
	 * Calculation: 1's complement 16bit checksum, calculated on all the boot configuration struct
	 * (including the syscom metadata). The algorithm is explained in RFC-1071 - https://www.rfc-editor.org/rfc/rfc1071
	 */
	uint16_t checksum;
	/** Padding bytes used for alignment */
	uint8_t padding[2];
	/** Reserved for future use.  Set to zero. */
	uint32_t reserved[IA_GOFO_BOOT_RESERVED_SIZE];
	/** Configuration of syscom queues */
	struct syscom_config_s syscom_context_config[1];
};

/**
 * Very minimalistic boot parameters containing the minimum required
 * to setup messaging on the secondary syscom channel.
 * Any further configuration is handled in the messages.
 */
struct ia_gofo_secondary_boot_config {
	/** Length of this structure in bytes, including the variable number of queue configs */
	uint32_t length;
	/** Semantic version number of this configuration structure */
	struct ia_gofo_version_s config_version;
	/**
	 * Semantic messaging protocol versions supported by host.
	 * Host will thus declare the protocol versions it
	 * supports, in order of preference
	 */
	struct ia_gofo_msg_version_list client_version_support;
	/** Reserved fields for future use and to keep this secondary strucutre aligned with primary structure */
	uint8_t reserved1[IA_GOFO_BOOT_SECONDARY_RESERVED_FIELDS];
	/**
	 * 16-bit Checksum value
	 * Used only in INSYS Stand-alone SoC (HKR)
	 * Calculation: 1's complement 16bit checksum, calculated on all the boot configuration struct
	 * (including the syscom metadata). The algorithm is explained in RFC-1071 - https://www.rfc-editor.org/rfc/rfc1071
	 */
	uint16_t checksum;
	/** Padding bytes used for alignment */
	uint8_t padding[2];
	/** Reserved for future use.  Set to zero. */
	uint32_t reserved2[IA_GOFO_BOOT_SECONDARY_RESERVED_SIZE];
	/** Configuration of secondary syscom queues */
	struct syscom_config_s syscom_context_config[1];
};

/**
 * Calculate allocation size of boot configuration structure as a parameter of the number
 * of queues.
 */
#define FW_BOOT_CONFIG_ALLOC_SIZE(num_queues) \
	((sizeof(struct ia_gofo_boot_config) + \
	(sizeof(struct syscom_queue_params_config) * num_queues)))

#pragma pack(pop)

/** @} */

/** Compile time checks only - this function is not to be called! */
static inline void ia_gofo_boot_config_abi_test_func(void)
{
	CHECK_ALIGN32(struct ia_gofo_boot_config);
	CHECK_ALIGN32(struct ia_gofo_secondary_boot_config);
}

#endif /* __ASSEMBLER__ */

#endif
