/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2020 - 2024 Intel Corporation
 */

#ifndef IPU7_FW_SYSCOM_ABI_H
#define IPU7_FW_SYSCOM_ABI_H

#include <linux/types.h>
#include "ipu7_fw_common_abi.h"

/**
 * @file
 * This file describes binary data structures and registers
 * that are shared between host and firmware and are required
 * for syscom queue setup and operation.
 *
 */

/*
 * Tool not yet available
 * @mscfile simplified_syscom.signalling "SW-FW Syscom & Boot Interface Sequence
 * (Place holder - MSCGEN tool integration not yet available"
 */

/*
 * Tool not yet available
 * @mscfile simplified_syscom.signalling "SW-FW Syscom & Boot Interface Sequence
 * (Place holder - MSCGEN tool integration not yet available"
 */

#pragma pack(push, 1)

/**
 * Syscom queue size (max_capacity) cannot be smaller than this value.
 */
#define SYSCOM_QUEUE_MIN_CAPACITY 2U

/**
 * Basic queue parameters defining the backing shared memory
 * that will hold the queue array, and that array's geometry
 */
struct syscom_queue_params_config {
	/** Actual message memory. MUST be cache line (64 byte) aligned. */
	ia_gofo_addr_t token_array_mem;

	/**
	 * Size of buffer at token_array_mem. Must be a multiple of 4 bytes.
	 * If token size is greater than 64 bytes and thus a candidate for
	 * DMA transfers, it must be a multiple of 64 bytes to conform to
	 * uC iDMA constraints on the starting address.
	 * @todo TBD on threshold for using DMA.
	 */
	uint16_t token_size_in_bytes;

	/**
	 * Max capacity of the queue. This is the number of elements in
	 * the underlying allocated array.
	 * Note: max_capacity must be at least SYSCOM_QUEUE_MIN_CAPACITY!
	 */
	uint16_t max_capacity;
};

/**
 * The syscom config structure that is passed to FW at boot.
 * This is the initial syscom config structure that is initialized by
 * the host and passed to FW as part of its boot process. This defines the
 * initial number of queues.
 */
struct syscom_config_s {
	/**
	 * Maximum number of input queues that host has allocated.
	 * Currently must be exactly 2.  One for acks and one for logs.
	 */
	uint16_t max_output_queues;

	/**
	 * Maximum number of input queues that host has allocated,
	 * including the two reserved queues for non-task related messaging
	 * @todo My preference is to move this to the device_open messages
	 */
	uint16_t max_input_queues;

	/*
	 * Queue config for all queues required at bootstrapping ONLY with
	 * output queues are first, followed by input queues
	 * This array MUST be of size (max_output_queues + max_input_queues)
	 * Were we to use C99 flexible arrays, it would look like this:
	 * syscom_queue_params_config queue_configs[];
	 * See QueueParamsConfigGetQueueConfigs
	 */
};

/**
 * Calculate allocation size of Syscom configuration structure as
 * a parameter of the number of queues.
 */
#define SYSCOM_CONFIG_ALLOC_SIZE(num_queues) \
	((sizeof(struct syscom_config_s) + \
	(sizeof(struct syscom_queue_params_config) * num_queues)))

#pragma pack(pop)

static inline struct syscom_queue_params_config *syscom_config_get_queue_configs
(struct syscom_config_s *config)
{
	return (struct syscom_queue_params_config *)(&config[1]);
}

static inline const struct syscom_queue_params_config *syscom_config_get_queue_configs_const(const struct syscom_config_s *config)
{
	return (const struct syscom_queue_params_config *)(&config[1]);
}

/** Compile time checks only - this function is not to be called! */
static inline void syscom_abi_test_func(void)
{
	CHECK_ALIGN32(struct syscom_queue_params_config);
	CHECK_ALIGN32(struct syscom_config_s);
}

#pragma pack(push, 1)

/**
 * Queue indices structure to reside in DMEM.
 * This is the shared
 * part of syscom that both host and firmware read/write.
 */
struct syscom_queue_indices_s {
	/**
	 * Index to token in the front of queue
	 * This will be the next token to dequeue. Only consumer can
	 * write to this variable.
	 *
	 * Note: For now, only lowest byte is in use --> max value is 127.
	 * Defining at 32 bit because
	 * that is the smallest atomic read/write that
	 * both host and FW can support together
	 */
	uint32_t read_index;

	/**
	 * Index to the token in the rear of queue
	 * This was the last token queued. The next token will go after this.
	 * Only producer can write to this variable.
	 *
	 * Note: For now, only lowest byte is in use --> max value is 127.
	 * Defining at 32 bit because
	 * that is the smallest atomic read/write that
	 * both host and FW can support together
	 */
	uint32_t write_index;
};

#pragma pack(pop)

/** Compile time checks only - this function is not to be called! */
static inline void syscom_queue_abi_test_func(void)
{
	CHECK_ALIGN64(struct syscom_queue_indices_s);
}

#endif
