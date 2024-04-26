/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2022 - 2024 Intel Corporation */

#include "ia_gofo_boot_config_abi.h"

#define FW_QUEUE_CONFIG_SIZE(num_queues) \
	(sizeof(struct syscom_queue_config) * (num_queues))

int ipu_boot_init_boot_config(struct ipu_bus_device *adev,
			      struct syscom_queue_config *qconfigs,
			      int num_queues, u32 uc_freq,
			      dma_addr_t subsys_config);
void ipu_boot_release_boot_config(struct ipu_bus_device *adev);
int ipu_boot_start_fw(const struct ipu_bus_device *adev);
int ipu_boot_stop_fw(const struct ipu_bus_device *adev);
u32 ipu_boot_get_boot_state(const struct ipu_bus_device *adev);
