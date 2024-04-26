// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2013 - 2024 Intel Corporation

#ifndef SYSCOM_QUEUE_ABI_H_INCLUDED__
#define SYSCOM_QUEUE_ABI_H_INCLUDED__

#include "ia_gofo_syscom_abi.h"

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
