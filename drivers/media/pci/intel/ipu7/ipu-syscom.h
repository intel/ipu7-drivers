/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2013 - 2024 Intel Corporation */

#ifndef IPU_SYSCOM_H
#define IPU_SYSCOM_H

#include "ia_gofo_syscom_abi.h"

struct syscom_queue_config {
	void *token_array_mem;
	u32 queue_size;
	u16 token_size_in_bytes;
	u16 max_capacity;
};

struct ipu_syscom_context {
	u16 num_input_queues;
	u16 num_output_queues;
	struct syscom_queue_config *queue_configs;
	void __iomem *queue_indices;
	dma_addr_t queue_mem_dma_addr;
	void *queue_mem;
	u32 queue_mem_size;
};

void *ipu_syscom_get_token(struct ipu_syscom_context *ctx, int q);
void ipu_syscom_put_token(struct ipu_syscom_context *ctx, int q);
struct syscom_queue_params_config *
ipu_syscom_get_queue_config(struct syscom_config_s *config);

#endif
