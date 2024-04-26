// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2013 - 2024 Intel Corporation

#ifndef FWPS_MSG_Q_DEFS_H_INCLUDED__
#define FWPS_MSG_Q_DEFS_H_INCLUDED__

/**
 * @addtogroup ipu_msg_abi
 * @{
 */

#include "ia_gofo_msg_q_defs.h"

/** Maximum PSYS syscom INPUT queues */
#define FWPS_MSG_ABI_MAX_INPUT_QUEUES (60U)
/** Maximum PSYS syscom OUTPUT queues */
#define FWPS_MSG_ABI_MAX_OUTPUT_QUEUES (2U)

/** Maximum number of all queues, input and output together */
#define FWPS_MSG_ABI_MAX_QUEUES (FWPS_MSG_ABI_MAX_OUTPUT_QUEUES + FWPS_MSG_ABI_MAX_INPUT_QUEUES)

/**
 * Output queue for acknowledge/done messages
 * This queue is opened by firmware at startup
 */
#define FWPS_MSG_ABI_OUT_ACK_QUEUE_ID	(IA_GOFO_MSG_ABI_OUT_ACK_QUEUE_ID)

/**
 * Output queue for log messages, including error messages
 * This queue is opened by firmware at startup
 */
#define FWPS_MSG_ABI_OUT_LOG_QUEUE_ID	(IA_GOFO_MSG_ABI_OUT_LOG_QUEUE_ID)

#if (FWPS_MSG_ABI_OUT_LOG_QUEUE_ID >= FWPS_MSG_ABI_MAX_OUTPUT_QUEUES)
#error "Maximum output queues configuration is too small to fit ACK and LOG queues"
#endif

/**
 * Input queue for device level messages
 * This queue is opened by firmware at startup
 */
#define FWPS_MSG_ABI_IN_DEV_QUEUE_ID	(IA_GOFO_MSG_ABI_IN_DEV_QUEUE_ID)

/** Reserved input queue for possible future use */
#define FWPS_MSG_ABI_IN_RESERVED_QUEUE_ID	(3U)

/**
 * First task input queue.  Other input task queues may follow
 * up to the maximum number of queues.
 */
#define FWPS_MSG_ABI_IN_FIRST_TASK_QUEUE_ID	(FWPS_MSG_ABI_IN_RESERVED_QUEUE_ID + 1U)

#if (FWPS_MSG_ABI_IN_FIRST_TASK_QUEUE_ID >= FWPS_MSG_ABI_MAX_INPUT_QUEUES)
#error "Maximum queues configuration is too small to fit minimum number of useful queues"
#endif

/** Last task input queue */
#define FWPS_MSG_ABI_IN_LAST_TASK_QUEUE_ID	(FWPS_MSG_ABI_MAX_QUEUES - 1U)

/** Maximum number of task queues for sending ordered tasks to a node context */
#define FWPS_MSG_ABI_IN_MAX_TASK_QUEUES \
	(FWPS_MSG_ABI_IN_LAST_TASK_QUEUE_ID - FWPS_MSG_ABI_IN_FIRST_TASK_QUEUE_ID + 1U)

/** First output queue ID */
#define FWPS_MSG_ABI_OUT_FIRST_QUEUE_ID (FWPS_MSG_ABI_OUT_ACK_QUEUE_ID)

/** Last output queue ID */
#define FWPS_MSG_ABI_OUT_LAST_QUEUE_ID (FWPS_MSG_ABI_MAX_OUTPUT_QUEUES - 1U)

/** First input queue ID */
#define FWPS_MSG_ABI_IN_FIRST_QUEUE_ID (FWPS_MSG_ABI_IN_DEV_QUEUE_ID)

/** Last input queue ID */
#define FWPS_MSG_ABI_IN_LAST_QUEUE_ID (FWPS_MSG_ABI_IN_LAST_TASK_QUEUE_ID)

/** Evaluates to true if the queue_id identifies an input (SW-->FW) queue */
#define FWPS_MSG_ABI_IS_OUTPUT_QUEUE(queue_id) (\
	(queue_id >= FWPS_MSG_ABI_OUT_FIRST_QUEUE_ID) && (\
	queue_id <= FWPS_MSG_ABI_OUT_LAST_QUEUE_ID))

/** Evaluates to true if the queue_id identifies an output (FW-->SW) queue */
#define FWPS_MSG_ABI_IS_INPUT_QUEUE(queue_id) (!FWPS_MSG_ABI_IS_OUTPUT_QUEUE(queue_id))

/**
 * Maximum size for all host-fw
 * Guessing as of now
 */
#define FWPS_MSG_HOST2FW_MAX_SIZE       (2U * 1024U)

/**
 * Maximum size for all fw-host messages
 * Guessing as of now
 */
#define FWPS_MSG_FW2HOST_MAX_SIZE       (256U)

/** @} */

#endif
