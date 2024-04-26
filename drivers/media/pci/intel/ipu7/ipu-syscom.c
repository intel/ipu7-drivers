// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2013 - 2024 Intel Corporation

#include "ipu.h"
#include "ipu-syscom.h"

#include "ia_gofo_syscom_queue_abi.h"

static void __iomem *ipu_syscom_get_indices(struct ipu_syscom_context *ctx,
					    u32 q)
{
	return ctx->queue_indices + (q * sizeof(struct syscom_queue_indices_s));
}

void *ipu_syscom_get_token(struct ipu_syscom_context *ctx, int q)
{
	struct syscom_queue_config *queue_params = &ctx->queue_configs[q];
	void __iomem *queue_indices = ipu_syscom_get_indices(ctx, q);
	u32 write_index = readl(queue_indices +
				offsetof(struct syscom_queue_indices_s,
					 write_index));
	u32 read_index = readl(queue_indices +
			       offsetof(struct syscom_queue_indices_s,
					read_index));
	void *token = NULL;

	if (q < ctx->num_output_queues) {
		/* Output queue */
		bool empty = (write_index == read_index);

		if (!empty)
			token = queue_params->token_array_mem +
				read_index *
				queue_params->token_size_in_bytes;
	} else {
		/* Input queue */
		bool full = (read_index == ((write_index + 1U) %
			     (u32)queue_params->max_capacity));

		if (!full)
			token = queue_params->token_array_mem +
				write_index * queue_params->token_size_in_bytes;
	}
	return token;
}
EXPORT_SYMBOL_GPL(ipu_syscom_get_token);

void ipu_syscom_put_token(struct ipu_syscom_context *ctx, int q)
{
	struct syscom_queue_config *queue_params = &ctx->queue_configs[q];
	void __iomem *queue_indices = ipu_syscom_get_indices(ctx, q);
	u32 offset, index;

	if (q < ctx->num_output_queues)
		/* Output queue */
		offset = offsetof(struct syscom_queue_indices_s, read_index);

	else
		/* Input queue */
		offset = offsetof(struct syscom_queue_indices_s, write_index);

	index = readl(queue_indices + offset);
	writel((index + 1U) % queue_params->max_capacity,
	       queue_indices + offset);
}
EXPORT_SYMBOL_GPL(ipu_syscom_put_token);

struct syscom_queue_params_config *
ipu_syscom_get_queue_config(struct syscom_config_s *config)
{
	return (struct syscom_queue_params_config *)(&config[1]);
}
EXPORT_SYMBOL_GPL(ipu_syscom_get_queue_config);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Intel IPU FW syscom library");
