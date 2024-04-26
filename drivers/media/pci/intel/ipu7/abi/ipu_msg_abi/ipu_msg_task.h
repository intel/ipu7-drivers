// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2013 - 2024 Intel Corporation

#ifndef IPU_MSG_TASK_H_INCLUDED__
#define IPU_MSG_TASK_H_INCLUDED__

/**
 * @addtogroup ipu_msg_abi
 * @{
 */

#include "ipu_msg_node.h"
#include "ipu_msg_link.h"
#include "ia_gofo_common_abi.h"
#include "ipu_msg_err_codes.h"

#pragma pack(push, 1)

/**
 *  Request a processing task.
 *
 *  Only may be sent once the graph has been opened successfully, which
 *  also creates the node contexts and their associated task queues.
 */
struct ipu_msg_task {
	/** Message header */
	struct ia_gofo_msg_header header;
	/** Graph instance identifier as specified in the graph open message */
	uint8_t graph_id;
	/**
	 *  Index of node context configuration profile (variant) as
	 *  set in the graph_open message per node
	 */
	uint8_t profile_idx;
	/**  Node context identifier as specified in the graph open message */
	uint8_t node_ctx_id;
	/**  Frame identifier - cyclical */
	uint8_t frame_id;
	/**
	 *  Fragment identifier (zero-based index) within a frame.
	 *  Must be set to zero for unfragmented frames.
	 */
	uint8_t frag_id;
	/**
	 *  Request a "done" message when this task is complete.
	 *  1 --> send done, 0 --> don't send.  @see ipu_msg_task_done
	 */
	uint8_t req_done_msg;
	/**
	 *  Request an interrupt when a "done" message is enqueued.  Ignored if req_done_msg is 0.
	 */
	uint8_t req_done_irq;
	/**  Reserved for alignment.  Set to zero. */
	uint8_t reserved[1];

	/**
	 * Bitmap of *load* terminals (by terminal ID) who's payload has NOT changed since
	 * the last task message *for the same node context* AND
	 * there is thus potential to skip the loading of those payloads in hardware.
	 * Any terminal marked here must be enabled in the profile
	 * inicated by profile_idx both in this task and the previous task on this node context.
	 *
	 * The payload buffer pointer for terminals marked here MUST still be supplied in
	 * term_buffers and the payload content MUST be valid as firmware cannot guarantee
	 * that the load will be skipped.  Examples:
	 * - The hardware state was lost in a power saving flow between tasks.
	 * - A task from a different node context was scheduled since the last task on this
	 *   task's node context.
	 */
	ipu_msg_teb_t payload_reuse_bm;

	/**
	 * Pointers to buffers that will be used by the enabled terminals,
	 * in the IPU address space.
	 * Each entry corresponds to a terminal where the array index is the terminal ID
	 * Terminals not enabled in the profile TEB are not required to be set,
	 * but for diagnostics purposes zero'ing is preferred.
	 */
	ia_gofo_addr_t term_buffers[IPU_MSG_MAX_NODE_TERMS];
};

/**
 *  Signals completion of a processing node
 *
 *  Roughly parallel to an acknowledge message, except that:
 *  - Is deferred in time
 *  - Is optional
 */
struct ipu_msg_task_done {
	/**
	 *  ABI ACK message header.  Includes the common message header too.
	 *  Type will be IPU_MSG_TYPE_GRAPH_TASK_DONE
	 */
	struct ia_gofo_msg_header_ack header;
	/**  Identifier of the graph containing the node context */
	uint8_t graph_id;
	/**  Identifier of the frame that has completed */
	uint8_t frame_id;
	/** Node context identifier as specified in the graph open message */
	uint8_t node_ctx_id;
	/**
	 *  Index of node context configuration profile (variant) for
	 *  the node task that has completed
	 */
	uint8_t profile_idx;
	/**  Identifier of the fragment in the frame that has completed */
	uint8_t frag_id;
	/**  Reserved for alignment.  Set to zero. */
	uint8_t reserved[3];
};

/**
 *  Error detail enumeration for error group IPU_MSG_ERR_GROUP_TASK used exclusively
 *  in the task done messages
 */
enum ipu_msg_err_task {
	/**  No error */
	IPU_MSG_ERR_TASK_OK = IA_GOFO_MSG_ERR_OK,
	/**
	 * Invalid graph ID.
	 * err_detail[0] is the offending graph_id
	 */
	IPU_MSG_ERR_TASK_GRAPH_ID = 1,
	/**
	 * Invalid node ctx ID.
	 * err_detail[0] is the offending node_ctx_id
	 */
	IPU_MSG_ERR_TASK_NODE_CTX_ID = 2,
	/**
	 *  Invalid profile index.
	 *  err_detail[0] is the offending profile_idx.
	 *  err_detail[1] is the node_ctx_id
	 */
	IPU_MSG_ERR_TASK_PROFILE_IDX = 3,
	/** Task object memory allocation failure */
	IPU_MSG_ERR_TASK_CTX_MEMORY_TASK = 4,
	/**
	 *  Invalid terminal payload ptr.
	 *  Received NULL where a non-NULL value is required or other illegal value .
	 *  err_detail[0] is the offending offending pointer value.
	 *  err_detail[1] is the terminal id.
	 */
	IPU_MSG_ERR_TASK_TERM_PAYLOAD_PTR = 5,
	/**
	 *  Unexpected frame ID in a task message.
	 *  err_detail[0] is the offending frame_id and
	 *  err_detail[1] is the expected frame_id,
	 */
	IPU_MSG_ERR_TASK_FRAME_ID = 6,
	/**
	 *  Unexpected fragment ID in a task message.
	 *  err_detail[0] is the offending frag_id and
	 *  err_detail[1] is the expected frag_id,
	 */
	IPU_MSG_ERR_TASK_FRAG_ID = 7,
	/**
	 *  Error on task execution - external error.
	 *  err_detail[0] terminal id
	 *  err_detail[1] device id
	 */
	IPU_MSG_ERR_TASK_EXEC_EXT = 8,
	/**
	 *  Error on task execution - SBX error.
	 *  err_detail[0] terminal id
	 *  err_detail[1] device id
	 */
	IPU_MSG_ERR_TASK_EXEC_SBX = 9,
	/**
	 *  Error on task execution - internal error.
	 *  err_detail[0] terminal id
	 *  err_detail[1] device id
	 */
	IPU_MSG_ERR_TASK_EXEC_INT = 10,
	/**
	 *  Error on task execution - unknown error.
	 *  err_detail[0] terminal id
	 *  err_detail[1] device id
	 */
	IPU_MSG_ERR_TASK_EXEC_UNKNOWN = 11,
	/** Size of enumeration */
	IPU_MSG_ERR_TASK_N
};

#pragma pack(pop)

/**  @} */

/**  Compile time checks only - this function is not to be called! */
static inline void ipu_msg_task_test_func(void)
{
	CHECK_ALIGN64(struct ipu_msg_task);
	CHECK_ALIGN64(struct ipu_msg_task_done);
	CHECK_ALIGN32(struct ipu_msg_link_cmprs_option);
}

#endif
