// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2013 - 2024 Intel Corporation
#ifndef IPU_MSG_GRAPH_H_INCLUDED__
#define IPU_MSG_GRAPH_H_INCLUDED__

/**
 * @addtogroup ipu_msg_abi
 * @{
 */

#include "ipu_msg_term.h"
#include "ipu_msg_node.h"
#include "ipu_msg_link.h"
#include "ipu_msg_task.h"
#include "ipu_msg_err_codes.h"

#pragma pack(push, 1)

/**
 * Graph ID placeholder until the allocated ID becomes known
 * in a graph_open ACK response
 */
#define IPU_MSG_GRAPH_ID_UNKNOWN (0xFFU)

/**
 *  Graph messages map for messages Graph Open and Close
 *  @{
 */
#define IPU_MSG_GRAPH_SEND_MSG_ENABLED 1U
#define IPU_MSG_GRAPH_SEND_MSG_DISABLED 0U

#define IPU_MSG_GRAPH_OPEN_SEND_RESP (1U << 0U)
#define IPU_MSG_GRAPH_OPEN_SEND_IRQ (1U << 1U)

#define IPU_MSG_GRAPH_CLOSE_SEND_RESP (1U << 0U)
#define IPU_MSG_GRAPH_CLOSE_SEND_IRQ (1U << 1U)
/** @} */

/**
 * Open a graph instance.  This is roughly parallel to a frame stream, but it
 * defines the node topology (actually, the topology superset) of the processing
 * graph.
 * No node tasks may be queued until this message is positively acknowledged.
 */
struct ipu_msg_graph_open {
	/** Common ABI message header.  Type will be IPU_MSG_TYPE_GRAPH_OPEN */
	struct ia_gofo_msg_header header;
	/**
	 * List of *enabled* node objects that are active in the graph.
	 * Must be of type ipu_msg_node_t or a derivative.
	 */
	struct ia_gofo_tlv_list nodes;
	/**
	 * List of *enabled* link objects.  Must be a of type ipu_msg_link or a derivative.
	 * Links not specified here are assumed to be to/from
	 * the host and with full buffer atomicity.
	 */
	struct ia_gofo_tlv_list links;
	/**
	 * Identifier of graph to be opened.  Must be 0 <= graph_id < max_graphs,
	 * where max_graphs was declared in the device_open message.  See ipu_msg_dev_open.
	 * Can be set to IPU_MSG_GRAPH_ID_UNKNOWN, in which case firmware will decide.
	 */
	uint8_t graph_id;
	/**
	 * Graph messages map, configurable send response message and interrupt trigger for that message
	 *
	 * See IPU_MSG_GRAPH_OPEN_SEND for more information about graph messages.
	 */
	uint8_t graph_msg_map;
	/** Reserved for alignment AND future close instructions like */
	uint8_t reserved[6];
};

/** Enumeration of fgraph open ACK option types for use in ipu_msg_graph_open_ack */
enum ipu_msg_graph_ack_option_types {
	/** All TLV headers must define value zero as padding */
	IPU_MSG_GRAPH_ACK_OPTION_TYPES_PADDING = 0,
	/**
	 * Queue info set by FW for each managed node.
	 * See ipu_msg_graph_open_ack_node_info_t
	 * @note Slated for decommisioning once SW-FW task interface is retired.
	 */
	IPU_MSG_GRAPH_ACK_TASK_Q_INFO,
	/** Number of items in this enumeration */
	IPU_MSG_GRAPH_ACK_OPTION_TYPES_N
};

/**
 * Structure for graph_ack option IPU_MSG_GRAPH_ACK_TASK_Q_INFO
 * which contains information set by FW, but required by SW
 * Only managed nodes have task queues and thus this option
 */
struct ipu_msg_graph_open_ack_task_q_info {
	/** Option header */
	struct ia_gofo_tlv_header header;
	/**
	 *  Zero-based index of this node in the fgraph.
	 *  Must be numbered in the same order as the node appears
	 *  in the fgraph's node-list.
	 */
	uint8_t node_ctx_id;
	/** Identifier of the queue that will order task requests for this node */
	uint8_t q_id;

	/** Reserved for padding - set to zero */
	uint8_t reserved[2];
};

/**
 * Acknowledges a graph open message.  Sent only after the graph has actually been opened
 * in firmware or when the request has been rejected.
 *
 * Extension option types are defined in ipu_msg_graph_ack_option_types
 */
struct ipu_msg_graph_open_ack {
	/**
	 * ABI ACK message header.  Includes the common message header too.
	 * Type will be IPU_MSG_TYPE_GRAPH_OPEN_ACK
	 */
	struct ia_gofo_msg_header_ack header;
	/** Graph ID as specified in the graph open message */
	uint8_t graph_id;
	/** Reserved for alignment.  Set to zero. */
	uint8_t reserved[7];
};

/**
 * Close a graph instance.
 * No node tasks may be queued after this message is sent.
 *
 * Close must fail if incomplete tasks are still pending inside IPU
 * @todo Add wait_pending_tasks and force_close capablities
 */
struct ipu_msg_graph_close {
	/** Common ABI message header.  Type will be IPU_MSG_TYPE_GRAPH_CLOSE */
	struct ia_gofo_msg_header header;
	/** Graph to be closed as specified in the graph open message */
	uint8_t graph_id;
	/**
	 * Graph messages map, configurable send response message and interrupt trigger for that message
	 *
	 * See IPU_MSG_GRAPH_CLOSE_SEND for more information about graph messages.
	 */
	uint8_t graph_msg_map;
	/**
	 * Reserved for alignment AND future close instructions like
	 * wait_pending_tasks and force_close.  Set to zero.
	 */
	uint8_t reserved[6];
};

/**
 * Acknowledges a graph close message.  Sent only after the graph has actually been closed
 * in firmware or when the request has been rejected.
 */
struct ipu_msg_graph_close_ack {
	/**
	 * ABI ACK message header.  Includes the common message header too.
	 * Type will be IPU_MSG_TYPE_GRAPH_CLOSE_ACK
	 */
	struct ia_gofo_msg_header_ack header;
	/** Graph ID as specified in the graph close message */
	uint8_t graph_id;
	/** Reserved for alignment.  Set to zero. */
	uint8_t reserved[7];
};

/**
 *  Error detail enumeration for error group IPU_MSG_ERR_GROUP_GRAPH used exclusively
 *  in the graph open ack and graph close ack messages
 */
enum ipu_msg_err_graph {
	/** No error */
	IPU_MSG_ERR_GRAPH_OK = IA_GOFO_MSG_ERR_OK,
	/**
	 *  State machine error.  Messge received doesn't fit with graph state.
	 *  err_detail[0] will be the actual graph state.
	 */
	IPU_MSG_ERR_GRAPH_GRAPH_STATE = 1,
	/**
	 *  Couldn't allocate graph ID (none left) - only when
	 *  graph_id==IPU_MSG_GRAPH_ID_UNKNOWN
	 *  in the graph_open message.
	 *  err_detail[0] is the max_graphs of the device context.
	 */
	IPU_MSG_ERR_GRAPH_MAX_GRAPHS = 2,
	/**
	 * Invalid graph ID.
	 * err_detail[0] is the offending graph_id
	 */
	IPU_MSG_ERR_GRAPH_GRAPH_ID = 3,
	/**
	 * Invalid node ctx ID.
	 * err_detail[0] is the offending node_ctx_id
	 */
	IPU_MSG_ERR_GRAPH_NODE_CTX_ID = 4,
	/**
	 * Invalid node rsrc ID.
	 * err_detail[0] is the offending node_rsrc_id
	 */
	IPU_MSG_ERR_GRAPH_NODE_RSRC_ID = 5,
	/**
	 *  Invalid profile index.
	 *  err_detail[0] is the offending profile_idx.
	 *  err_detail[1] is the node_ctx_id
	 *
	 *  NOTE: Profile idx is not currently passed in graph open. This error is therefore reserved.
	 */
	IPU_MSG_ERR_GRAPH_PROFILE_IDX = 6,
	/**
	 *  Invalid terminal ID.
	 *  err_detail[0] is the offending terminal id.
	 *  err_detail[1] is the node_ctx_id
	 */
	IPU_MSG_ERR_GRAPH_TERM_ID = 7,
	/**
	 * Invalid terminal payload size.
	 * err_detail[0] is the offending payload size.
	 *
	 * Note: FW doesn't know the required size of the payload since it's not aware of the
	 * frame resolution. Therefore the payload size is checked only for a non zero value.
	 */
	IPU_MSG_ERR_GRAPH_TERM_PAYLOAD_SIZE = 8,
	/**
	 *  Invalid node ctx ID in a link endpoint.
	 *  err_detail[0] is the offending node_ctx_id.
	 *  err_detail[1] is the term_id of the node.
	 */
	IPU_MSG_ERR_GRAPH_LINK_NODE_CTX_ID = 9,
	/**
	 *  Invalid terminal ID in a link endpoint.
	 *  err_detail[0] is the offending term_id.
	 *  err_detail[1] is the node_ctx_id of the terminal.
	 */
	IPU_MSG_ERR_GRAPH_LINK_TERM_ID = 10,
	/**
	 *  Profile supplied is unsupported AND silently ignoring it would be a problem.
	 *  err_detail[0] is the offending tlv type
	 */
	IPU_MSG_ERR_GRAPH_PROFILE_TYPE = 11,
	/**
	 *  Illegal number of fragments.
	 *  err_detail[0] is the offending number of fragments
	 *  err_detail[1] is the node ctx ID
	 */
	IPU_MSG_ERR_GRAPH_NUM_FRAGS = 12,
	/**
	 *  Message type doesn't belong on the received queue.
	 *  err_detail[0] is the q_id used.
	 *  err_detail[1] is the q_id that should have been used or IPU_MSG_Q_ID_UNKNOWN
	 *  if a single proper q_id cannot be determined.
	 */
	IPU_MSG_ERR_GRAPH_QUEUE_ID_USAGE = 13,
	/**
	 *  Error on opening queue for node context tasks.
	 *  err_detail[0] is the q_id that could not be opened.
	 *  err_detail[1] is the node_ctx_id it was supposed to serve.
	 */
	IPU_MSG_ERR_GRAPH_QUEUE_OPEN = 14,
	/**
	 *  Error on closing node context task queue.
	 *  err_detail[0] is the q_id that could not be closed.
	 *  err_detail[1] is the node_ctx_id it serves.
	 */
	IPU_MSG_ERR_GRAPH_QUEUE_CLOSE = 15,
	/**
	 *  Sent message has incorrect queue id associated with it.
	 *  err_detail[0] is the node_ctx_id the message is for,
	 *  err_detail[1] is q_id is used.
	*/
	IPU_MSG_ERR_GRAPH_QUEUE_ID_TASK_REQ_MISMATCH = 16,
	/**
	 * FGraph context object memory allocation failure.
	 *  err_detail[0] is graph_id
	 */
	IPU_MSG_ERR_GRAPH_CTX_MEMORY_FGRAPH = 17,
	/**
	 * Node context object memory allocation failure.
	 * err_detail[0] is node_ctx_id
	 */
	IPU_MSG_ERR_GRAPH_CTX_MEMORY_NODE = 18,
	/**
	 * Node context profile memory allocation failure.
	 * err_detail[0] is node_ctx_id
	 */
	IPU_MSG_ERR_GRAPH_CTX_MEMORY_NODE_PROFILE = 19,
	/**
	 *  Terminal context object memory allocation failure.
	 *  err_detail[0] is node_ctx_id.
	 *  err_detail[1] is term_id
	 */
	IPU_MSG_ERR_GRAPH_CTX_MEMORY_TERM = 20,
	/**
	 *  Link context object memory allocation failure.
	 *  err_detail[0] is dest node_ctx_id.
	 *  err_detail[1] is dest term_id.
	 *  @todo can we add source node and term?
	 */
	IPU_MSG_ERR_GRAPH_CTX_MEMORY_LINK = 21,
	/**
	 *  Error configuring graph messages map. It is error to configure for the same message type to send irq but
	 *  do not send response message.
	 *  err_detail[0] value of the msg_map.
	 */
	IPU_MSG_ERR_GRAPH_CTX_MSG_MAP = 22,
	/**
	 *  foreign key is invalid.
	 *  err_detail[0] is foreign key value.
	 *  err_detail[1] is maximum foreign key value.
	 */
	IPU_MSG_ERR_GRAPH_CTX_FOREIGN_KEY = 23,
	/**
	 *  streaming mode is invalid.
	 *  err_detail[0] is link_ctx_id.
	 *  err_detail[1] is streaming mode value.
	 */
	IPU_MSG_ERR_GRAPH_CTX_STREAMING_MODE = 24,
	/**
	 *  pbk acquire failure.
	 *  err_detail[0] is soc_pbk_id.
	 *  err_detail[1] is slot_id.
	 */
	IPU_MSG_ERR_GRAPH_CTX_PBK_RSRC = 25,
	/**
	 *  Invalid event type.
	 *  err_detail[0] is the term_id.
	 *  err_detail[1] is the offending event type.
	*/
	IPU_MSG_ERR_GRAPH_UNSUPPORTED_EVENT_TYPE = 26,
	/**
	 *  Invalid num of events.
	 *  err_detail[0] is the term_id.
	 *  err_detail[1] is the is maximum events.
	*/
	IPU_MSG_ERR_GRAPH_TOO_MANY_EVENTS = 27,
	/**
	 *  Compression option memory allocation failure.
	 *  err_detail[0] is link_ctx_id of the link that contains the compression option.
	 */
	IPU_MSG_ERR_GRAPH_CTX_MEMORY_CMPRS = 28,
	/**
	 *  Alignment interval is not a valid value.
	 *  err_detail[0] is link_ctx_id of the link that contains the compression option.
	 *  err_detail[1] is the offending value.
	 */
	IPU_MSG_ERR_GRAPH_CTX_CMPRS_ALIGN_INTERVAL = 29,
	/**
	 *  Unrecognized plane type.
	 *  err_detail[0] is link_ctx_id of the link that contains the compression option.
	 *  err_detail[1] is the offending value.
	 */
	IPU_MSG_ERR_GRAPH_CTX_CMPRS_PLANE_ID = 30,
	/**
	 *  Invalid compression mode for the link source endpoint.  The type of compression (or any
	 *  compression at all) is not supported by the data source endpoints of the link.
	 *  err_detail[0] is link_ctx_id of the link that contains the compression option.
	 *  err_detail[1] is the plane index.
	 */
	IPU_MSG_ERR_GRAPH_CTX_CMPRS_UNSUPPORTED_MODE = 31,
	/**
	 *  Unrecognized bit depth or a bit depth unsupported by compression on this link.
	 *  err_detail[0] is link_ctx_id of the link that contains the compression option.
	 *  err_detail[1] is the offending value.
	 */
	IPU_MSG_ERR_GRAPH_CTX_CMPRS_BIT_DEPTH = 32,
	/**
	 *  Stride is not aligned to a valid multiple. Stride must be a multiple of tile size.
	 *  err_detail[0] is link_ctx_id of the link that contains the compression option.
	 *  err_detail[1] is the offending value.
	 */
	IPU_MSG_ERR_GRAPH_CTX_CMPRS_STRIDE_ALIGNMENT = 33,
	/**
	 *  A sub-buffer base address (== frame base address + subbuffer offset) is not properly aligned.
	 *  err_detail[0] is link_ctx_id of the link that contains the compression option.
	 *  err_detail[1] is the offending value.
	 */
	IPU_MSG_ERR_GRAPH_CTX_CMPRS_SUB_BUFFER_ALIGNMENT = 34,
	/**
	 *  One of pixel sub-buffers or tile status sub-buffers is mis-ordered.
	 *  Assuming the following order: y_pixel_buffer, uv_pixel_buffer, y_ts_buffer, uv_ts_buffer.
	 *  err_detail[0] is link_ctx_id of the link that contains the compression option.
	 *  err_detail[1] is the first detected out-of-order sub-buffer with 0 for pixel, 1 for tile status.
	 */
	IPU_MSG_ERR_GRAPH_CTX_CMPRS_LAYOUT_ORDER = 35,
	/**
	 *  One of pixel sub-buffers or tile status sub-buffers overlaps another.
	 *  err_detail[0] is link_ctx_id of the link that contains the compression option.
	 *  err_detail[1] is the plane index.
	 */
	IPU_MSG_ERR_GRAPH_CTX_CMPRS_LAYOUT_OVERLAP = 36,
	/**
	 *  Compressed buffer size is too small.
	 *  err_detail[0] is the required compression buffer size as calculated by FW.
	 *  err_detail[1] is link_ctx_id of the link that contains the compression option.
	 *
	 *
	 *  @note FW calculates the compression buffer size based on the values
	 *		supplied in the compression option, not based on the frame resolution.
	 *		If the values supplied in the compression option are incorrect, the calculated
	 *		buffer size returned here would be incorrect as well.
	 *
	 *  @note This buffer size error is specific to the compressed buffer size and says nothing about the
	 *        nominal size of the uncompressed buffer declared in the terminal.
	 */
	IPU_MSG_ERR_GRAPH_CTX_CMPRS_BUFFER_TOO_SMALL = 37,
	/**
	 * Delayed processing link creation is invalid.
	 * can be caused by one of the following violations:
	 * delayed processing link must be a self link
	 * delayed processing link cannot be SOFF
	 * err_detail[0] is link_ctx_id of the delayed processing link.
	 */
	IPU_MSG_ERR_GRAPH_CTX_DELAYED_LINK = 38,

	/** Size of enumeration */
	IPU_MSG_ERR_GRAPH_N
};

#pragma pack(pop)

/** Compile time checks only - this function is not to be called! */
static inline void ipu_msg_graph_test_func(void)
{
	CHECK_ALIGN64(struct ipu_msg_graph_open);
	CHECK_ALIGN64(struct ipu_msg_graph_open_ack);
	CHECK_ALIGN64(struct ipu_msg_graph_close);
	CHECK_ALIGN64(struct ipu_msg_graph_close_ack);
	CHECK_ALIGN32(struct ipu_msg_graph_open_ack_task_q_info);
}

/** @} */

#endif

