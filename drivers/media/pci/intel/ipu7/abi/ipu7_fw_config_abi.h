/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2021 - 2024 Intel Corporation
 */

#ifndef IPU7_FW_CONFIG_ABI_H
#define IPU7_FW_CONFIG_ABI_H

#include <linux/types.h>

#define IPU_CONFIG_ABI_WDT_TIMER_DISABLED 0U
#define IPU_CONFIG_ABI_CMD_TIMER_DISABLED 0U

/** HW watchdog configuration */
struct ipu7_wdt_abi {
	/**
	 * HW WDT timer#1 timeout value, provided in uSec.
	 * If value of zero used WDT will be disabled.
	 */
	uint32_t wdt_timer1_us;

	/**
	 * HW WDT timer#2 timeout value, provided in uSec.
	 * If value of zero used WDT will be disabled.
	 */
	uint32_t wdt_timer2_us;
};

#endif
