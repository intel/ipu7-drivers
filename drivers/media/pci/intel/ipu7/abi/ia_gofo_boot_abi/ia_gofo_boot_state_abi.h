// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2013 - 2024 Intel Corporation

#ifndef IA_GOFO_BOOT_STATE_ABI_H_INCLUDED__
#define IA_GOFO_BOOT_STATE_ABI_H_INCLUDED__

/**
 *  @addtogroup ia_gofo_boot_abi
 *  @{
 */

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

/** @} */

#endif /* IA_GOFO_BOOT_STATE_ABI_H_INCLUDED__ */
