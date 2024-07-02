/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2013 - 2024 Intel Corporation
 */

#ifndef IPU7_DMA_H
#define IPU7_DMA_H

#include <linux/dma-map-ops.h>
#include <linux/iova.h>

struct ipu7_mmu_info;

struct ipu7_dma_mapping {
	struct ipu7_mmu_info *mmu_info;
	struct iova_domain iovad;
};

extern const struct dma_map_ops ipu7_dma_ops;

#endif /* IPU7_DMA_H */
