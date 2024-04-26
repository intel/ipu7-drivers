// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2013 - 2024 Intel Corporation

#ifndef IA_GOFO_MSG_Q_DEFS_H_INCLUDED__
#define IA_GOFO_MSG_Q_DEFS_H_INCLUDED__

/**
 * Output queue for acknowledge/done messages
 * This queue is opened by firmware at startup
 */
#define IA_GOFO_MSG_ABI_OUT_ACK_QUEUE_ID (0U)

/**
 * Output queue for log messages, including error messages
 * This queue is opened by firmware at startup
 */
#define IA_GOFO_MSG_ABI_OUT_LOG_QUEUE_ID (1U)

/**
 * Input queue for device level messages
 * This queue is opened by firmware at startup
 */
#define IA_GOFO_MSG_ABI_IN_DEV_QUEUE_ID (2U)

#endif

