/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2021 - 2024 Intel Corporation
 */

#ifndef IPU7_FW_INSYS_CONFIG_ABI_H
#define IPU7_FW_INSYS_CONFIG_ABI_H

#include "ipu7_fw_config_abi.h"
#include "ipu7_fw_boot_abi.h"
#include "ipu7_fw_isys_abi.h"

/**
 * INSYS specific config.
 */
struct ipu7_insys_config {
	/**
	 * timeout val in millisecond.
	 */
	uint32_t timeout_val_ms;
	/**  @see ia_gofo_boot_config.subsys_config */
	struct ia_gofo_logger_config logger_config;

	/** HW watchdog configuration */
	struct ipu7_wdt_abi wdt_config;
};

#endif
