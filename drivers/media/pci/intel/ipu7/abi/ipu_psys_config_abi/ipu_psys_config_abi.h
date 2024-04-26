// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2013 - 2024 Intel Corporation

#ifndef IPU_PSYS_CONFIG_ABI_H_INCLUDED__
#define IPU_PSYS_CONFIG_ABI_H_INCLUDED__

#include <linux/types.h>
#include "ia_gofo_logger_config_struct_abi.h"

#include "ipu_config_abi.h"

/**
 * PSYS specific config.
 * @see ia_gofo_boot_config.subsys_config
 */
struct ipu_psys_config {
	/**
	 * Enable HW agnostic debug manifest.
	 * (Formerly ddr_dmem_ddr)
	 */
	uint32_t use_debug_manifest;
	/**
	 * timeout val in millisecond.
	 */
	uint32_t timeout_val_ms;
	/**
	 * Enables compression support in FW.
	 * For HKR-1, enabling compression support means:
	 * 1. Pipelining is disabled for all CBs.
	 * 2. BCSM is not supported for any graphs.
	 * Note that enabling this bit does not mean that compression is active, just that it can be activated
	 * (using the compression TLV message in graph open).
	 * By default compression is not enabled in FW and therefore TLV compression messages would trigger an error.
	 */
	uint32_t compression_support_enabled;

	/** @see ia_gofo_boot_config.subsys_config */
	struct ia_gofo_logger_config logger_config;

	/** HW watchdog configuration */
	struct ipu_wdt_abi wdt_config;
};

#endif
