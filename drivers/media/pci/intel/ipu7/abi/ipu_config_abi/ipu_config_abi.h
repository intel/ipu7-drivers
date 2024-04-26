// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2013 - 2024 Intel Corporation

#ifndef IPU_CONFIG_ABI_H_INCLUDED__
#define IPU_CONFIG_ABI_H_INCLUDED__

#include <linux/types.h>

#define IPU_CONFIG_ABI_WDT_TIMER_DISABLED 0U
#define IPU_CONFIG_ABI_CMD_TIMER_DISABLED 0U

/** HW watchdog configuration */
struct ipu_wdt_abi {
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
