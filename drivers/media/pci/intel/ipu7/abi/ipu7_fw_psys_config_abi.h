/* INTEL CONFIDENTIAL
 *
 * Copyright (C) 2020 - 2023 Intel Corporation.
 * All Rights Reserved.
 *
 * The source code contained or described herein and all documents
 * related to the source code ("Material") are owned by Intel Corporation
 * or licensors. Title to the Material remains with Intel
 * Corporation or its licensors. The Material contains trade
 * secrets and proprietary and confidential information of Intel or its
 * licensors. The Material is protected by worldwide copyright
 * and trade secret laws and treaty provisions. No part of the Material may
 * be used, copied, reproduced, modified, published, uploaded, posted,
 * transmitted, distributed, or disclosed in any way without Intel's prior
 * express written permission.
 *
 * No License under any patent, copyright, trade secret or other intellectual
 * property right is granted to or conferred upon you by disclosure or
 * delivery of the Materials, either expressly, by implication, inducement,
 * estoppel or otherwise. Any license under such intellectual property rights
 * must be express and approved by Intel in writing.
 */

#ifndef IPU7_PSYS_CONFIG_ABI_H_INCLUDED__
#define IPU7_PSYS_CONFIG_ABI_H_INCLUDED__

#include <linux/types.h>

#include "ipu7_fw_boot_abi.h"
#include "ipu7_fw_config_abi.h"

/**
 * PSYS specific config.
 * @see ia_gofo_boot_config.subsys_config
 */
struct ipu7_psys_config {
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
	struct ipu7_wdt_abi wdt_config;
};

#endif
