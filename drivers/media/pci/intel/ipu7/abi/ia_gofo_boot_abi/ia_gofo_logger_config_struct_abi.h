// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2013 - 2024 Intel Corporation

#ifndef IA_GOFO_LOGGER_CONFIG_STRUCT_ABI_H_INCLUDED__
#define IA_GOFO_LOGGER_CONFIG_STRUCT_ABI_H_INCLUDED__

#include "ia_gofo_logger_severity_abi.h"
#include "ia_gofo_align_utils.h"
#include "ia_gofo_common_abi.h"

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

#endif
