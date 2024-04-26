// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2013 - 2024 Intel Corporation

#ifndef IPU_MSG_STATE_H_INCLUDED__
#define IPU_MSG_STATE_H_INCLUDED__

/**
 * @addtogroup ipu_msg_abi
 * @{
 */

/**
 * @file
 * State definitions for various context objects
 * While these are not explicitly in the messages, they are
 * defined in the messaging state machine.
 *
 * For example, a sending graph_open message will change
 * the state of a graph context in the host to "OPEN_WAIT".
 * Successful handling of the graph_open message in firmware
 * results in ! changing the firmware graph context state to
 * "OPEN" and receipt of the graph_open_ack response will change
 * the graph state to "OPEN" in the host.
 */

/**
 * States of a runtime context
 * @todo Need to add power states
 */
enum ipu_msg_dev_state {
	/** Starting state - device open message not yet sent */
	IPU_MSG_DEV_STATE_CLOSED = 0,
	/** Device open message sent, waiting for ack */
	IPU_MSG_DEV_STATE_OPEN_WAIT = 1,
	/** Only in this state can graph's be opened */
	IPU_MSG_DEV_STATE_OPEN = 2,
	/** Close message sent, waiting for ack */
	IPU_MSG_DEV_STATE_CLOSE_WAIT = 3,
	IPU_MSG_DEV_STATE_N
};

/**
 * States of a graph context
 * @note The state of the componenet node contexts are tied
 * to the graph context state - so the nodes contexts don't have a state of
 * their own.
 */
enum ipu_msg_graph_state {
	/** Starting state - device open message not yet sent */
	IPU_MSG_GRAPH_STATE_CLOSED = 0,
	/** Graph open message sent, waiting for ack */
	IPU_MSG_GRAPH_STATE_OPEN_WAIT = 1,
	/** Only in this state can node messages be sent */
	IPU_MSG_GRAPH_STATE_OPEN = 2,
	/** Close message sent, waiting for ack */
	IPU_MSG_GRAPH_STATE_CLOSE_WAIT = 3,
	IPU_MSG_GRAPH_STATE_N
};

/** States of a task */
enum ipu_msg_task_state {
	/** Task is complete */
	IPU_MSG_TASK_STATE_DONE = 0,
	/** Task has been sent, but is not yet complete. */
	IPU_MSG_TASK_STATE_WAIT_DONE = 1,
	IPU_MSG_TASK_STATE_N
};

/** @} */

#endif

