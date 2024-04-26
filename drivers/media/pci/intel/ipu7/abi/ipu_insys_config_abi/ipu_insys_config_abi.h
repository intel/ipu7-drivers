// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2013 - 2024 Intel Corporation

#ifndef IPU_INSYS_CONFIG_ABI_H_INCLUDED__
#define IPU_INSYS_CONFIG_ABI_H_INCLUDED__

#include "ipu_insys_types.h"
#include "ia_gofo_logger_config_struct_abi.h"

#include "ipu_config_abi.h"

/**
 * INSYS specific config.
 */
struct ipu_insys_config {
	/**
	 * timeout val in millisecond.
	 */
	uint32_t timeout_val_ms;
	/**  @see ia_gofo_boot_config.subsys_config */
	struct ia_gofo_logger_config logger_config;

	/** HW watchdog configuration */
	struct ipu_wdt_abi wdt_config;
};

#endif
