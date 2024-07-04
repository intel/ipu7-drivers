/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2020--2024 Intel Corporation
 */

#ifndef IPU7_FW_BOOT_ABI_H
#define IPU7_FW_BOOT_ABI_H

#include "ipu7_fw_common_abi.h"
#include "ipu7_fw_syscom_abi.h"

/**
 * Logger severity levels
 */
/** critical severity level */
#define IA_GOFO_FWLOG_SEVERITY_CRIT		(0U)
/** error severity level */
#define IA_GOFO_FWLOG_SEVERITY_ERROR	(1U)
/** warning severity level */
#define IA_GOFO_FWLOG_SEVERITY_WARNING	(2U)
/** info severity level */
#define IA_GOFO_FWLOG_SEVERITY_INFO		(3U)
/** debug severity level */
#define IA_GOFO_FWLOG_SEVERITY_DEBUG	(4U)
/** verbose severity level */
#define IA_GOFO_FWLOG_SEVERITY_VERBOSE	(5U)

/**
 * Max number of logger sources used by FW. Set to 64 (instead of 32) for extensibility.
 */
#define IA_GOFO_FWLOG_MAX_LOGGER_SOURCES (64U)

/** Logger boot-time channels enablement/disablement configuration defines */
#define LOGGER_CONFIG_CHANNEL_ENABLE_HWPRINTF_BITMASK  (1U << 0U)
#define LOGGER_CONFIG_CHANNEL_ENABLE_SYSCOM_BITMASK    (1U << 1U)
#define LOGGER_CONFIG_CHANNEL_ENABLE_ALL_BITMASK (          \
			LOGGER_CONFIG_CHANNEL_ENABLE_HWPRINTF_BITMASK | \
			LOGGER_CONFIG_CHANNEL_ENABLE_SYSCOM_BITMASK     \
		)

/** Logger boot time configuration */
struct ia_gofo_logger_config {
	/**
	 * Set to "true" to use the source severity as defined in this struct.
	 * Set to "false" to ignore the source severity and use defaults.
	 */
	uint8_t use_source_severity;
	/**
	 * Set severity for each source.
	 * Based on IA_GOFO_FWLOG_SEVERITY_<level>
	 */
	uint8_t source_severity[IA_GOFO_FWLOG_MAX_LOGGER_SOURCES];

	/**
	 * Set to "true" to use the channels enable bitmask as defined in this struct.
	 * Set to "false" to ignore the channels enable bitmask and use defaults (per compilation).
	 */
	uint8_t use_channels_enable_bitmask;
	/**
	 * Selects which logger channels to enable
	 * Note: Channel enablement will be dictated based on the compiled channels
	 * Note: buttress channel is not configurable and can not be disabled
	 *
	 * bitmask: See LOGGER_CONFIG_CHANNEL defines
	 */
	uint8_t channels_enable_bitmask;
	/**
	 * padding to align
	 */
	uint8_t padding[1];
	/**
	 * HW-PRINTF's buffer base addr in ddr.
	 * Note: in secure mode, this buffer should be in the untrusted memory range.
	 */
	ia_gofo_addr_t hw_printf_buffer_base_addr;
	/**
	 * HW-PRINTF's buffer size in ddr.
	 * Cache line aligned.
	 */
	uint32_t hw_printf_buffer_size_bytes;
};

/** Compile time checks only - this function is not to be called! */
static inline void ia_gofo_logger_config_abi_test_func(void)
{
	CHECK_ALIGN32(struct ia_gofo_logger_config);
}

/*
 * Watchdog handler, written in ASM, uses this file to get the value to be stored in a boot status register.
 * Rest of the file will be ignored by assembler to prevent it compiling with C syntax resulting in many errors.
 */
#ifndef __ASSEMBLER__

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

/**
 * The following boot status values defines used in ASM code (can't use enum)
 * this values are also added to the boot status enum
 */
/** Boot Status register value upon WDT Timeout */
#define IA_GOFO_WDT_TIMEOUT_ERR 0xDEAD0401U
/** Boot status register value upon Memory errors: Double memory error - occur while other error. */
#define IA_GOFO_MEM_FATAL_DME_ERR 0xDEAD0801U
/** Boot status register value upon Memory errors: unrecoverable memory error - in DMEM */
#define IA_GOFO_MEM_UNCORRECTABLE_LOCAL_ERR 0xDEAD0802U
/** Boot status register value upon Memory errors: error - in dirty dcache data */
#define IA_GOFO_MEM_UNCORRECTABLE_DIRTY_ERR 0xDEAD0803U
/** Boot status register value upon Memory errors: error - in dcache tag */
#define IA_GOFO_MEM_UNCORRECTABLE_DTAG_ERR 0xDEAD0804U
/** Boot status register value upon Memory errors: uncorrectable memory error - in icache and dcache (not dirty) */
#define IA_GOFO_MEM_UNCORRECTABLE_CACHE_ERR 0xDEAD0805U
/** Boot status register value upon double exception */
#define IA_GOFO_DOUBLE_EXCEPTION_ERR 0xDEAD0806U
/** Boot Status register value upon DMEM BIST failure.*/
#define IA_GOFO_BIST_DMEM_FAULT_DETECTION_ERR 0xDEAD1000U
/** Boot Status register value upon DATA INTEGRITY BIST failure.*/
#define IA_GOFO_BIST_DATA_INTEGRITY_FAILURE 0xDEAD1010U

#ifndef __ASSEMBLER__
/** Boot status values for the register at IA_GOFO_FW_BOOT_STATE_ID */
enum ia_gofo_boot_state {
	/**
	 * @note This value is only used for FuSa systems.
	 *
	 * SW sets this value to sync between safe (FuSa) SW and non safe feature SW.
	 * It should not be set or encountered by FW.
	 *
	 * This state is set when the secondary boot params have been configured
	 * by the safety SW and therefore the functional SW can kick FW.
	 * If this value is not set, the functional SW must wait for the safety SW
	 * to set it before kicking FW.
	 *
	 * Before kicking FW, the functional SW should change the boot state to
	 * IA_GOFO_FW_BOOT_STATE_UNINIT.
	 */
	IA_GOFO_FW_BOOT_STATE_SECONDARY_BOOT_CONFIG_READY = 0x57A7B000U,

	/**
	 * Set by SW before uC firmware kick
	 * FW can either busy-wait poll until it sees this value
	 * at buttress_registers_s::boot_state or wait for an interrupt
	 * and then check for this value.
	 */
	IA_GOFO_FW_BOOT_STATE_UNINIT = 0x57A7E000U,

	/**
	 * FW sets this value to buttress_registers_s::boot_state at early boot
	 * (ASAP) - confirms firmware program start
	 * @note Informational/Debug only
	 */
	IA_GOFO_FW_BOOT_STATE_STARTING_0 = 0x57A7D000U,

	/**
	 * FW sets this value to buttress_registers_s::boot_state at early boot
	 * after DMEM global/static variable initialization is done.
	 * @note Informational/Debug only
	 */
	IA_GOFO_FW_BOOT_STATE_MEM_INIT_DONE = 0x57A7D100U,

	/**
	 * FW sets this value to buttress_registers_s::boot_state at the start
	 * of late boot when boot parameters (content of
	 * buttress_registers_s::p_boot_params) are being processed and used
	 * to direct the rest of firmware boot
	 * @note Informational/Debug only
	 */
	IA_GOFO_FW_BOOT_STATE_BOOT_CONFIG_START = 0x57A7D200U,

	/**
	 * FW sets this value to buttress_registers_s::boot_state during
	 * late boot when the log output queue has been initialized and logging
	 * is now active.  At this point, host may pop log messages from the log
	 * output queue.  If the firmware boot is stuck (i.e. never reaches
	 * IA_GOFO_FW_BOOT_STATE_READY), this queue may have important information.
	 * @note Informational/Debug only - only meaningful if a log queue is
	 * defined at the higher messaging layer
	 */
	IA_GOFO_FW_BOOT_STATE_QUEUE_INIT_DONE = 0x57A7D300U,

	/**
	 * FW sets this value to buttress_registers_s::boot_state when the
	 * boot process is complete, including readiness to receive and process
	 * syscom command messages.
	 * SW can either busy-wait poll on this value, or wait for an interrupt and
	 * then check for this value.
	 *
	 * The boot_state register must be polled and checked for this value:
	 * - At any boot or communication timeout
	 * - Before every syscom queue scan start, no matter if that scan is interrupt driven or
	 *   polling-based.
	 */
	IA_GOFO_FW_BOOT_STATE_READY = 0x57A7E100U,

	/**
	 * FW sets this value to buttress_registers_s::boot_state at any time where a critical
	 * (== unrecoverable) error is detected which
	 * is not covered by another error state defined in this enumeration
	 * @note While this value is defined as part of the boot state enumeration, it may be
	 * driven by firmware at any time
	 * that firmware is active and not just during the boot process.
	 */
	IA_GOFO_FW_BOOT_STATE_CRIT_UNSPECIFIED = 0xDEAD0001U,

	/**
	 * FW sets this value to buttress_registers_s::boot_state
	 * when the boot configuration structure pointer
	 * at register IA_GOFO_FW_BOOT_CONFIG_ID is not valid. Note that firmware has limited
	 * capability to detect this, but some cases (e.g. NULL,
	 * IPU HW address space, alignment errors) are possible.
	 */
	IA_GOFO_FW_BOOT_STATE_CRIT_CFG_PTR = 0xDEAD0101U,

	/**
	 * FW sets this value to buttress_registers_s::boot_state
	 * when the boot configuration structure version specified
	 * in is not supported by the firmware
	 */
	IA_GOFO_FW_BOOT_STATE_CRIT_CFG_VERSION = 0xDEAD0201U,

	/**
	 * FW sets this value to buttress_registers_s::boot_state when the messaging protocol
	 * version is not supported by the firmware
	 */
	IA_GOFO_FW_BOOT_STATE_CRIT_MSG_VERSION = 0xDEAD0301U,

	/**
	 * FW sets this value to buttress_registers_s::boot_state when HW WDT timeout happens
	 */
	IA_GOFO_FW_BOOT_STATE_CRIT_WDT_TIMEOUT = IA_GOFO_WDT_TIMEOUT_ERR,

	/**
	 * FW sets this value to buttress_registers_s::boot_state at early boot
	 * to indicate that data section unpacking process failed.
	 * @note Informational/Debug only
	 */
	IA_GOFO_FW_BOOT_STATE_WRONG_DATA_SECTION_UNPACKING = 0xDEAD0501U,

	/**
	 * FW sets this value to buttress_registers_s::boot_state at early boot
	 * to indicate that rodata section unpacking process failed.
	 * @note Informational/Debug only
	 */
	IA_GOFO_FW_BOOT_STATE_WRONG_RO_DATA_SECTION_UNPACKING = 0xDEAD0601U,

	/**
	 * FW sets this value to buttress_registers_s::boot_state at
	 * boot to indicate that the value of the boot parameter
	 * register at index IA_GOFO_FW_BOOT_UNTRUSTED_ADDR_MIN_ID is
	 * not valid.
	 *
	 * See IA_GOFO_FW_BOOT_UNTRUSTED_ADDR_MIN_ID for the definition
	 * of valid values.
	 */
	IA_GOFO_FW_BOOT_STATE_INVALID_UNTRUSTED_ADDR_MIN = 0xDEAD0701U,

	/**
	 * FW sets this value to buttress_registers_s::boot_state when memory error happends
	 */
	IA_GOFO_FW_BOOT_STATE_CRIT_MEM_FATAL_DME = IA_GOFO_MEM_FATAL_DME_ERR,

	/**
	 * FW sets this value to buttress_registers_s::boot_state when memory error happends
	 */
	IA_GOFO_FW_BOOT_STATE_CRIT_MEM_UNCORRECTABLE_LOCAL = IA_GOFO_MEM_UNCORRECTABLE_LOCAL_ERR,

	/**
	 * FW sets this value to buttress_registers_s::boot_state when memory error happends
	 */
	IA_GOFO_FW_BOOT_STATE_CRIT_MEM_UNCORRECTABLE_DIRTY = IA_GOFO_MEM_UNCORRECTABLE_DIRTY_ERR,

	/**
	 * FW sets this value to buttress_registers_s::boot_state when memory error happends
	 */
	IA_GOFO_FW_BOOT_STATE_CRIT_MEM_UNCORRECTABLE_DTAG = IA_GOFO_MEM_UNCORRECTABLE_DTAG_ERR,

	/**
	 * FW sets this value to buttress_registers_s::boot_state when memory error happends
	 */
	IA_GOFO_FW_BOOT_STATE_CRIT_MEM_UNCORRECTABLE_CACHE = IA_GOFO_MEM_UNCORRECTABLE_CACHE_ERR,

	/**
	 * FW sets this value to buttress_registers_s::boot_state when double exception happends
	 */
	IA_GOFO_FW_BOOT_STATE_CRIT_DOUBLE_EXCEPTION = IA_GOFO_DOUBLE_EXCEPTION_ERR,

	/**
	 * HOST sets this value to buttress_registers_s::boot_state when it wants to signal FW to
	 * Begin an orderly shutdown.  This value is execptional as it is the only one that HOST
	 * may write when firmware is active and is not really a state, but rather a command.
	 *
	 * IA_GOFO_FW_BOOT_STATE_SHUTDOWN_CMD may only be written by the host AFTER host ensures that
	 * the FW state is IA_GOFO_FW_BOOT_STATE_READY.
	 */
	IA_GOFO_FW_BOOT_STATE_SHUTDOWN_CMD = 0x57A7F001U,

	/**
	 * FW sets this value to buttress_registers_s::boot_state when it has begun a shutdown
	 * sequence.
	 * @note Informational/Debug only
	 */
	IA_GOFO_FW_BOOT_STATE_SHUTDOWN_START = 0x57A7E200U,

	/**
	 * FW sets this value to buttress_registers_s::boot_state when it completes its shutdown
	 * sequence. At this point, FW is halted.
	 */
	IA_GOFO_FW_BOOT_STATE_INACTIVE = 0x57A7E300U,

	/**
	 * FW sets this value to buttress_registers_s::boot_state when it discovers hw timeout
	 * on cb cmd execution. At this point, FW is halted.
	 */
	IA_GOFO_FW_BOOT_HW_CMD_ACK_TIMEOUT = 0x57A7E400U
};
#endif /* __ASSEMBLER__ */

#endif