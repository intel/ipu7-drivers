/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2020 - 2024 Intel Corporation
 */

#ifndef IPU7_FW_MSG_ABI_H
#define IPU7_FW_MSG_ABI_H

#include "ipu7_fw_common_abi.h"

#pragma pack(push, 1)

/**  Message type enumeration for the message headers */
enum ipu7_msg_type {
	/**  Type 0 is padding, which is irrelevant here. */
	IPU_MSG_TYPE_RESERVED = IA_GOFO_MSG_TYPE_RESERVED,

	/** See ia_gofo_msg_indirect */
	IPU_MSG_TYPE_INDIRECT = IA_GOFO_MSG_TYPE_INDIRECT,

	/** See ia_gofo_msg_log */
	IPU_MSG_TYPE_DEV_LOG = IA_GOFO_MSG_TYPE_LOG,

	/** See ia_gofo_msg_general_err */
	IPU_MSG_TYPE_GENERAL_ERR = IA_GOFO_MSG_TYPE_GENERAL_ERR,

	IPU_MSG_TYPE_DEV_OPEN = 4,
	IPU_MSG_TYPE_DEV_OPEN_ACK = 5,
	IPU_MSG_TYPE_GRAPH_OPEN = 6,
	IPU_MSG_TYPE_GRAPH_OPEN_ACK = 7,
	IPU_MSG_TYPE_TASK_REQ = 8,
	IPU_MSG_TYPE_TASK_DONE = 9,
	IPU_MSG_TYPE_GRAPH_CLOSE = 10,
	IPU_MSG_TYPE_GRAPH_CLOSE_ACK = 11,
	IPU_MSG_TYPE_DEV_CLOSE = 12,
	IPU_MSG_TYPE_DEV_CLOSE_ACK = 13,
	IPU_MSG_TYPE_TERM_EVENT = 14,
	IPU_MSG_TYPE_N,
};

#pragma pack(pop)

#pragma pack(push, 1)

/** Maximum terminals per node */
#define IPU_MSG_MAX_NODE_TERMS (64U)

/** Maximum image fragments */
#define IPU_MSG_MAX_FRAGS (7U)

/** Enumeration of different kinds of nodes. */
enum ipu7_msg_node_type {
	/** Type zero is always padding */
	IPU_MSG_NODE_TYPE_PAD = 0,
	/** Basic generic node.  May be extended with options. */
	IPU_MSG_NODE_TYPE_BASE,
	IPU_MSG_NODE_TYPE_N
};

/**
 * Maximum number of devices possible in a CB
 * @todo This number copied from CB HAS "MAX_NUM_OF_DEVICE".  Should get from geneated headers.
 * @todo The actual number can be much smaller, so INTERNAL data structures might benefit from
 * lower memory use.  Consider differentiating ABI definition from internal usage.
 */
#define IPU_MSG_NODE_MAX_DEVICES (128U)

/** Number of 32 bit integers in a DEB */
#define DEB_NUM_UINT32 (IPU_MSG_NODE_MAX_DEVICES/(sizeof(uint32_t) * 8U))

/** Type for Terminal Enable Bitmaps (TEB) */
typedef uint32_t ipu7_msg_teb_t[2];

/** Type for Device Enable Bitmap (DEB) bitmaps */
typedef uint32_t ipu7_msg_deb_t[DEB_NUM_UINT32];

/**
 * Maximum number of routing bits within a CB
 * @todo This number copied from CB HAS "MAX_RBM_SIZE".  Should get from geneated headers.
 */
#define IPU_MSG_NODE_MAX_ROUTE_ENABLES (128U)

/** Number of 32 bit integers in a REB */
#define RBM_NUM_UINT32 (IPU_MSG_NODE_MAX_ROUTE_ENABLES/(sizeof(uint32_t) * 8U))

/** Type for Routing BitMap (RBM) */
typedef uint32_t ipu7_msg_rbm_t[RBM_NUM_UINT32];

/** Type for Route Enable Bitmap (REB). Tied to RBM definition. */
typedef ipu7_msg_rbm_t ipu7_msg_reb_t;

/** TLV types for node profiles.  All types must extend IPU_MSG_NODE_PROFILE_TYPE_BASE */
enum ipu7_msg_node_profile_type {
	/** Type zero is always padding */
	IPU_MSG_NODE_PROFILE_TYPE_PAD = 0,
	/** Profile for a basic generic node - see ipu7_msg_node_profile */
	IPU_MSG_NODE_PROFILE_TYPE_BASE,
	/** Profile for a CB node.- see ipu7_msg_cb_profile */
	IPU_MSG_NODE_PROFILE_TYPE_CB,
	IPU_MSG_NODE_PROFILE_TYPE_N
};

/**
 * Node profile for generic nodes.  Represents one of possibly several ways to configure the
 * terminals of a node within the same node context.
 */
struct ipu7_msg_node_profile {
	/** Type here is one of ipu7_msg_node_profile_type */
	struct ia_gofo_tlv_header tlv_header;
	/** Terminal Enable Bitmap - must be a subset of that declared during graph open. */
	ipu7_msg_teb_t teb;
};

/** Node profile for CB nodes.  Includes additional bitmaps that CB nodes require */
struct ipu7_msg_cb_profile {
	/** Generic node profile.  CB extensions follow. */
	struct ipu7_msg_node_profile profile_base;
	/** Internal CB Device Enable Bitmask */
	ipu7_msg_deb_t deb;
	/** Internal CB Routing BitMask */
	ipu7_msg_rbm_t rbm;
	/** Internal CB Routing element Enable Bitmask */
	ipu7_msg_reb_t reb;
};

/** Maximum number of profiles allowed in ipu7_msg_node::profile_list */
#define IPU_MSG_NODE_MAX_PROFILES (2U)

/** Default profile index.  Must always exist as there must always be at least one profile. */
#define IPU_MSG_NODE_DEF_PROFILE_IDX (0U)

/**
 *  Special node resource ID to identify a generic external node.  Required when there is a link
 *  to/from IPU and that node.
 *
 *  Usage of this ID assumes that all external IP's have the same abstracted streaming link
 *  interface with IPU, such that IPU need not know or care who they really are.  Thus we need not
 *  define specific ID's for each external node and maintain it separately per project.
 *
 *  If an external node has a interface requiring specific behavior from IPU, then a specific
 *  resource ID must be defined higher in the stack, just like IPU nodes.
 *
 *  Generic EXT_IP node instances (see ipu7_msg_node) should use a single node profile (see
 *  ipu7_msg_node::profiles_list) with TLV type IPU_MSG_NODE_PROFILE_TYPE_BASE and structure
 *  ipu7_msg_node_profile.  ipu7_msg_node_profile::teb should be set to all-one's, as if all terminals
 *  are enabled.
 *
 *  Links to/from EXT_IP nodes should specify the node node_ctx_id of the EXT_IP instance as for all
 *  links in their endpoints.  The endpoint terminal ID should be set to
 *  IPU_MSG_LINK_TERM_ID_DONT_CARE.
 */
#define IPU_MSG_NODE_RSRC_ID_EXT_IP (0xFFU)

/**
 * External IP Nodes have all their TEB bits set.
 * @{
 */
#define IPU_MSG_NODE_DONT_CARE_TEB_HI (0xFFFFFFFFU)
#define IPU_MSG_NODE_DONT_CARE_TEB_LO (0xFFFFFFFFU)
/** @} */

/**
 * Node resource ID of INSYS, required when there is a link from INSYS to PSYS.
 */
#define IPU_MSG_NODE_RSRC_ID_IS (0xFEU)

/** Base structure for all node objects */
struct ipu7_msg_node {
	/** Type here is one of ipu7_msg_node_type */
	struct ia_gofo_tlv_header tlv_header;
	/**
	 *  Identifier of node resource ID in the manifest graph model.  See also
	 *  IPU_MSG_NODE_RSRC_ID_EXT_IP, IPU_MSG_NODE_RSRC_ID_IS
	 */
	uint8_t node_rsrc_id;
	/**
	 * Zero-based index of this node in the fgraph.  Must be numbered in the
	 * same order as the node appears in the fgraph's node-list.
	 */
	uint8_t node_ctx_id;
	/** Number of fragments for all future frames in this node context */
	uint8_t num_frags;
	/** Alignment.  Set to zero. */
	uint8_t reserved[1];
	/**
	 * List of configuration alternatives for this node.  At least one is required.
	 * All TLV profile types must derive from ipu7_msg_node_profile.  Profile order is
	 * significant as task generation will be defined by the the order of the profiles
	 * in the list.  See struct ipu7_msg_node_profile.
	 */
	struct ia_gofo_tlv_list profiles_list;
	/**
	 * List of terminal objects of type @see ipu7_msg_term or derivations.
	 * Only enabled terminals are allowed - and even then they are optional.  Generally,
	 * terminals need not be in the list if they have no special options as the superteb
	 * defines which are enabled.
	 */
	struct ia_gofo_tlv_list terms_list;
	/** Extension options per node */
	struct ia_gofo_tlv_list node_options;
};

/** Enumeration of node option types */
enum ipu7_msg_node_option_types {
	/** All TLV headers must define value zero as padding */
	IPU_MSG_NODE_OPTION_TYPES_PADDING = 0,
	/** Number of items in this enumeration */
	IPU_MSG_NODE_OPTION_TYPES_N
};

#pragma pack(pop)

/** @} */

/** Compile time checks only - this function is not to be called! */
static inline void ipu7_msg_node_test_func(void)
{
	CHECK_ALIGN32(struct ipu7_msg_node);
	CHECK_ALIGN32(struct ipu7_msg_node_profile);
	CHECK_ALIGN32(struct ipu7_msg_cb_profile);
}

#pragma pack(push, 1)

/** Type of link for PARSING.  Currently only support a generic type. */
enum ipu7_msg_link_type {
	/** Type zero is always padding */
	IPU_MSG_LINK_TYPE_PAD = 0,
	/** Basic generic link */
	IPU_MSG_LINK_TYPE_GENERIC = 1,
	IPU_MSG_LINK_TYPE_N
};

/**
 * Enumeration of known link options
 * Link extension can be attached to the terminal through these options.
 */
enum ipu7_msg_link_option_types {
	/** Type zero is always padding */
	IPU_MSG_LINK_OPTION_TYPES_PADDING = 0,
	/** Compression parameters - see ipu7_msg_link_cmprs_option  */
	IPU_MSG_LINK_OPTION_TYPES_CMPRS = 1,
	/** Number of link options */
	IPU_MSG_LINK_OPTION_TYPES_N
};

/**
 * Possible values for struct ipu7_msg_link_cmprs_plane_desc.bit_depth.
 * Added here as an enum for reference only.
 */
enum ipu7_msg_link_cmprs_option_bit_depth {
	IPU_MSG_LINK_CMPRS_OPTION_8BPP = 0,
	IPU_MSG_LINK_CMPRS_OPTION_10BPP = 1,
	IPU_MSG_LINK_CMPRS_OPTION_12BPP = 2,
};

/**
 *  Denominator value for usage of space saving ratio field
 *
 *  See ipu7_msg_link_cmprs_option::space_saving_ratio_numerator
 */
#define IPU_MSG_LINK_CMPRS_SPACE_SAVING_DENOM (128U)

/** Number of lossy compression algorithm parameters. */
#define IPU_MSG_LINK_CMPRS_LOSSY_CFG_PAYLOAD_SIZE (5U)

/**
 *  Maximum configurable space saving ratio numerator for footprint compression
 *
 *  See ipu7_msg_link_cmprs_option::space_saving_ratio_numerator
 */
#define IPU_MSG_LINK_CMPRS_SPACE_SAVING_NUM_MAX (IPU_MSG_LINK_CMPRS_SPACE_SAVING_DENOM - 1U)

/**
 *  Per-plane information required for compression
 */
struct ipu7_msg_link_cmprs_plane_desc {
	/** 1 if plane exists, otherwise 0.  If 0, the rest of the structure is to be ignored. */
	uint8_t plane_enable;

	/**
	 *  1 if compression is enabled, else 0.
	 *  Note that one plane may be compressed while the other(s), if any, are not.
	 */
	uint8_t cmprs_enable;

	/** Plane id for the encoder, must match the arch spec */
	uint8_t encoder_plane_id;

	/** Plane id for the decoder, must match the arch spec */
	uint8_t decoder_plane_id;

	/**
	 *  1 if lossy compression is enabled (with or without footprint compression),
	 *  else 0 for lossless compression.
	 */
	uint8_t cmprs_is_lossy;

	/**
	 *  1 if footprint compression is enabled, else 0.
	 *  Enabling footprint compression is valid only when cmprs_is_lossy is enabled.
	 */
	uint8_t cmprs_is_footprint;

	/**
	 * Planar bits per pixel.  This field contains a HW register mapped value.
	 * See enum ipu7_msg_link_cmprs_option_bit_depth.
	 */
	uint8_t bit_depth;

	/**
	 *  Footprint space savings scaled to IPU_MSG_LINK_CMPRS_SPACE_SAVING_DENOM,
	 *  such that:
	 *  space_saving_ratio = 1 -
	 *      (space_saving_numerator / IPU_MSG_LINK_CMPRS_SPACE_SAVING_DENOM)
	 *
	 *  For example, a value of (IPU_MSG_LINK_CMPRS_SPACE_SAVING_DENOM / 4) here results in a
	 *  75% memory footprint reduction.
	 *
	 *  Largest value is IPU_MSG_LINK_CMPRS_SPACE_SAVING_NUM_MAX
	 *
	 *  Not used for lossless compression mode.
	 *
	 *  @note Corresponds to HW register FP_CRATIO, but the term "compression ratio" there seems
	 *  to be a misnomer, with an inverse relationship to the more accepted cratio definition,
	 *  which is why "space saving" is used here instead.
	 *
	 */
	uint8_t space_saving_numerator;

	/**
	 *  Offset in bytes of the plane buffer from the beginning of the buffer.
	 *  First plane should start at offset zero.
	 *
	 *  Buffer base pointer + pixels_offset must be 4KB page aligned.
	 */
	uint32_t pixels_offset;

	/**
	 *  Offset in bytes of the compression Tile Status meta-data from the beginning of the
	 *  link buffer.
	 *
	 *  Buffer base pointer + ts_offset must be at least cache-line (64B) aligned.  On some platforms,
	 *  may need to be 4KB page aligned for best performance.
	 */
	uint32_t ts_offset;

	/**
	 *  Stride of a row of *compression tiles* in bytes
	 *
	 *  Calculated from the first byte in a compression tile to the first byte in the next row of
	 *  compression tiles.  Compression tile dimensions depend on the link and plane.
	 *
	 *  Note the implied requirement that the plane's line stride be a multiple of the compression tile
	 *  width *at least*.  Some planes may have more strict requirements.
	 *
	 */
	uint32_t tile_row_to_tile_row_stride;

	/**
	 *  Height in compression tiles
	 *
	 *  The plane's height be a multiple of the compression tile height *at least*, where compression
	 *  tile dimensions depend on the link and plane.  Some planes may have more strict requirements.
	 */
	uint32_t rows_of_tiles;

	/**
	 *  Lossy compression configuration data written as-is to the hardware.
	 *  Will be ignored when compression is lossless (i.e. cmprs_is_lossy is 0)
	 *
	 * LOSSY_CFG0 – Relevant to both BW and FP compression
	 * 	[0:7]		BW_CRATIO_PLUS
	 * 	[8:15]		BW_CRATIO_MINS
	 * 	[16:23]		BW_INST_UPPER
	 * 	[24:31]		BW_INST_LOWER
	 *
	 * LOSSY_CFG1  – Relevant to both BW and FP compression
	 * 	[0:7]		INIT_HISTORY
	 *
	 * LOSSY_CFG2 – Relevant to both BW and FP compression
	 * 	[0:4]		INIT_QP
	 * 	[8:12]		MAX_QP
	 * 	[16:20]		MIN_QP
	 * 	[24:25]		MA_WIN_SIZE
	 *
	 * LOSSY_CFG3 – Relevant only to FP compression
	 * 	[8:11]		MAX_QP_INC
	 * 	[16:19]		MAX_QP_DEC
	 * 	[24:26]		QP_INC_RST_VALUE
	 *
	 * LOSSY_CFG4 – Relevant only to FP compression
	 * 	[0:3]		LOG_FP_GUARD_BAND
	 */
	uint32_t lossy_cfg[IPU_MSG_LINK_CMPRS_LOSSY_CFG_PAYLOAD_SIZE];
};

/** Maximum number of planes supported by the compression feature */
#define IPU_MSG_LINK_CMPRS_MAX_PLANES (2U)

/** Value for ipu7_msg_link_cmprs_option::align_interval when streaming is disabled */
#define IPU_MSG_LINK_CMPRS_NO_ALIGN_INTERVAL (0U)

/** Maximum value allowed in ipu7_msg_link_cmprs_option::align_interval when streaming is enabled */
#define IPU_MSG_LINK_CMPRS_MIN_ALIGN_INTERVAL (16U)

/** Maximum value allowed in ipu7_msg_link_cmprs_option::align_interval */
#define IPU_MSG_LINK_CMPRS_MAX_ALIGN_INTERVAL (1024U)

/**
 * Additional node-task request data specific to the compression on
 * links between nodes.  Only relevant for link endpoints where compression support
 * is available on both sides of the link.
 *
 * If compression is not enabled for the link, this option must not be instantiated.
 *
 */
struct ipu7_msg_link_cmprs_option {
	/** Header common to all link options.  TLV type ID here is IPU_MSG_LINK_OPTION_TYPES_CMPRS. */
	struct ia_gofo_tlv_header header;

	/**
	 *  Size of the buffer allocated for the compressed frame, including all pixel planes
	 *  and any tile status planes.
	 *
	 *  May be larger than the uncompressed buffer size (see ipu7_msg_term::payload_size) in
	 *  many compression configurations as the compression meta-data takes up additional space.
	 */
	uint32_t cmprs_buf_size;

	/**
	 *  Interval of TS alignment in buffer chasing streaming modes (i.e. BCSM or BCLM).  Must be the
	 *  same as the streaming pointer update granularity set in the hardware streaming configuration,
	 *  which is opaque to this SW-FW interface.  Set to IPU_MSG_LINK_CMPRS_NO_ALIGN_INTERVAL for SOFF
	 *  or DOFF links.
	 *
	 *  For streaming links, valid values are power-of-2 only, from
	 *  IPU_MSG_LINK_CMPRS_MIN_ALIGN_INTERVAL to IPU_MSG_LINK_CMPRS_MAX_ALIGN_INTERVAL, inclusive. When
	 *  compression is enabled on streamng links, this is a constraint on the pointer update granularity
	 *  as well.
	 *
	 *  @note (Theoretically, there are special cases where IPU_MSG_LINK_CMPRS_NO_ALIGN_INTERVAL here
	 *        could be set for a buffer chasing streaming link, but this would be very fragile and is
	 *        not supported in this interface.)
	 *
	 *  @todo OPEN: Should this be defined in pow2 exponents like HW?  Tend no.
	 */
	uint16_t align_interval;

	/** Reserved for alignment.  Set to zero. */
	uint8_t reserved[2];

	/** Compressed configuration and dimensions description, per plane. */
	struct ipu7_msg_link_cmprs_plane_desc plane_descs[IPU_MSG_LINK_CMPRS_MAX_PLANES];
};

/** EndPoint of a link comprised of node and terminal ID's */
struct ipu7_msg_link_ep {
	/** Node ctx ID as described in the list of nodes in the fgraph */
	uint8_t node_ctx_id;
	/** Node terminal ID from manifest */
	uint8_t term_id;
};

/** EndPoint pair uniquely indentifying the link */
struct ipu7_msg_link_ep_pair {
	/** Source side of the link */
	struct ipu7_msg_link_ep ep_src;
	/** Destination (sink) side of the link */
	struct ipu7_msg_link_ep ep_dst;
};

/**
 * All local links (links between nodes within a subsystem) require this value to be set.
 * There is no need for true foreign ID in this case, since the same firmware manages both side of the link.
 */
#define IPU_MSG_LINK_FOREIGN_KEY_NONE UINT16_MAX
/** Maximum number of foreign key IDs that can allocated by the host */
#define IPU_MSG_LINK_FOREIGN_KEY_MAX (64U)

/**
 * Links with SOFF streaming mode require this value to be set.
 * All local links (links between nodes within a subsystem) require this value to be set.
 * There is no need for real pbk and slot id in this case,
 * since the same firmware manages both sides of the link.
 */
#define IPU_MSG_LINK_PBK_ID_DONT_CARE UINT8_MAX
/**
 * Same as @see 'IPU_MSG_LINK_PBK_ID_DONT_CARE'
 */
#define IPU_MSG_LINK_PBK_SLOT_ID_DONT_CARE UINT8_MAX

/**
 *  Placeholder terminal ID for link endpoints in EXT_IP nodes
 *
 *  On parsing, firmware will replace this value with a concrete terminal ID of its choice.
 *
 *  See IPU_MSG_NODE_RSRC_ID_EXT_IP
 */
#define IPU_MSG_LINK_TERM_ID_DONT_CARE (0xFFU)

/**
 * Link ABI object.  The link describes a directional data flow
 * where "data" here is in the most general sense (configuration, image, stats, etc.)
 *
 * Note that the data flow implies a dependency relationship between producer and consumer
 */
struct ipu7_msg_link {
	/**Type here is one of ipu7_msg_link_type */
	struct ia_gofo_tlv_header tlv_header;
	/** Identifer of the link's terminal endpoints */
	struct ipu7_msg_link_ep_pair endpoints;
	/**
	 * Unique identifier for links that connect to a node outside of the local graph.
	 * Unique foreign key is allocated by the host and the value is arbitrary (except local links).
	 * Local links must use IPU_MSG_LINK_FOREIGN_KEY_NONE.
	 * The maximum value is IPU_MSG_LINK_FOREIGN_KEY_MAX.
	 */
	uint16_t foreign_key;
	/**
	 * Data movement mechanisms between producer and consumer
	 * @see enum ia_gofo_msg_link_streaming_mode
	 */
	uint8_t streaming_mode;
	/**
	 * PBK instance for cross IPs connection.
	 * Local links must use IPU_MSG_LINK_PBK_ID_DONT_CARE.
	 * Links with SOFF streaming mode must use IPU_MSG_LINK_PBK_ID_DONT_CARE.
	 *
	 * @see enum ia_gofo_soc_pbk_instance_id
	 */
	uint8_t pbk_id;
	/**
	 * Relevant entry id in the PBK that manages producer-consumer handshake.
	 * Local links must use IPU_MSG_LINK_PBK_SLOT_ID_DONT_CARE.
	 * Links with SOFF streaming mode must use IPU_MSG_LINK_PBK_SLOT_ID_DONT_CARE.
	 */
	uint8_t pbk_slot_id;
	/**
	 * delayed processing link flag
	 * A delayed link is a link between the producer of a previous frame and a consumer of that data on the next frame.
	 * Set to 1 to mark as delayed, 0 if not.
	 */
	uint8_t delayed_link;
	/** Align to 32 bits.  Set to zero. */
	uint8_t reserved[2];
	/** Extension fields for links */
	struct ia_gofo_tlv_list link_options;
};

#pragma pack(pop)

/** @} */

/** Compile time checks only - this function is not to be called! */
static inline void ipu7_msg_abi_link_test_func(void)
{
	CHECK_ALIGN32(struct ipu7_msg_link);
	CHECK_ALIGN32(struct ipu7_msg_link_cmprs_option);
	CHECK_ALIGN32(struct ipu7_msg_link_cmprs_plane_desc);
}

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
enum ipu7_msg_dev_state {
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
enum ipu7_msg_graph_state {
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
enum ipu7_msg_task_state {
	/** Task is complete */
	IPU_MSG_TASK_STATE_DONE = 0,
	/** Task has been sent, but is not yet complete. */
	IPU_MSG_TASK_STATE_WAIT_DONE = 1,
	IPU_MSG_TASK_STATE_N
};

/**
 *  Error groups define broad categories of errors in ack messages
 *  @todo log messages too?
 *  These are the basic groups common to all applications based on
 *  this messaging protocol.
 *  Some groups ID are reserved for application specific extensions.
 *  See IPU_MSG_ERR_GROUP_APP_EXT_START
 */
enum ipu7_msg_err_groups {
	/** Reserved value.  Used only with IPU_MSG_ERR_OK & IPU_MSG_ERR_UNSPECIFED */
	IPU_MSG_ERR_GROUP_RESERVED = IA_GOFO_MSG_ERR_GROUP_RESERVED,
	/** Generic message general errors that aren't specific to another group */
	IPU_MSG_ERR_GROUP_GENERAL = IA_GOFO_MSG_ERR_GROUP_GENERAL,
	/** Device open/close errors */
	IPU_MSG_ERR_GROUP_DEVICE = 2,
	/** Graph open/close errors */
	IPU_MSG_ERR_GROUP_GRAPH = 3,
	/** Task request errors */
	IPU_MSG_ERR_GROUP_TASK = 4,

	/**
	 *  Number of items in this enumeration.
	 *  Cannot be larger than IPU_MSG_ERR_GROUP_APP_EXT_START
	 */
	IPU_MSG_ERR_GROUP_N,
};

#pragma pack(push, 1)

/**
 *  Request a processing task.
 *
 *  Only may be sent once the graph has been opened successfully, which
 *  also creates the node contexts and their associated task queues.
 */
struct ipu7_msg_task {
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
	 *  1 --> send done, 0 --> don't send.  @see ipu7_msg_task_done
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
	ipu7_msg_teb_t payload_reuse_bm;

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
struct ipu7_msg_task_done {
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
enum ipu7_msg_err_task {
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
static inline void ipu7_msg_task_test_func(void)
{
	CHECK_ALIGN64(struct ipu7_msg_task);
	CHECK_ALIGN64(struct ipu7_msg_task_done);
	CHECK_ALIGN32(struct ipu7_msg_link_cmprs_option);
}

#pragma pack(push, 1)

/**
 * Enumeration of terminal types
 * @todo As terminals are described by their attributes in the master
 * graph, it is unclear if this is at all necessary
 */
enum ipu7_msg_term_type {
	/** Type zero is always padding */
	IPU_MSG_TERM_TYPE_PAD = 0,
	/** Generic terminal - and the base type for all others */
	IPU_MSG_TERM_TYPE_BASE,
	/** Number of terminal types */
	IPU_MSG_TERM_TYPE_N,
};

/**
 *  Terminal events that FW will reflect to client SW.

 *  If an event is not directly available in hardware, firmware may
 *  internally and silently work-around this at its discretion.
 */
/** No event notification request  */
#define IPU_MSG_TERM_EVENT_TYPE_NONE		0U
/** Progress event. */
#define IPU_MSG_TERM_EVENT_TYPE_PROGRESS	1U
/** Number of terminal event types   */
#define IPU_MSG_TERM_EVENT_TYPE_N			(IPU_MSG_TERM_EVENT_TYPE_PROGRESS + 1U)

/**
 * Base terminal structure
 *
 * Note the lack of an options field.
 * This was omitted to save space as most terminals probably won't need this.
 * This may be added as a subclass extension or alternatively,
 * node options could be used instead at the price
 * of specifying the terminal ID in the option.
 */
struct ipu7_msg_term {
	/** Type here is one of ipu7_msg_term_type. */
	struct ia_gofo_tlv_header tlv_header;
	/**
	 *  ID of terminal in the static node description.
	 *  @note Only connect terminals support events.
	*/
	uint8_t term_id;
	/**
	 *  Request terminal event.  See IPU_MSG_TERM_EVENT_TYPE_BIT macros.
	 *
	 *  FW will send an event notification message to client SW when the event occurs.
	 *  When multiple event types are supported, they may be requested together by
	 *  bitwise OR'ing their bitmapped values here.
	 */
	uint8_t event_req_bm;
	/** Reserved for alignment padding.  Set to zero. */
	uint8_t reserved[2];
	/**
	 *  Size of payload associated with this terminal without any compression that may
	 *  optionally be applied, if supported.
	 */
	uint32_t payload_size;
	/** Terminal options */
	struct ia_gofo_tlv_list term_options;
};

/**
 * Enumeration of known terminal options
 * Terminal meta-data can be attached to the terminal through these options.
 */
enum ipu7_msg_term_option_types {
	IPU_MSG_TERM_OPTION_TYPES_PADDING = 0,
	/** Number of terminal options */
	IPU_MSG_TERM_OPTION_TYPES_N
};

/**
 *  Terminal event notification message.  Sent when an requested event occurs.
 *
 *  See ipu7_msg_term::event_req_bm
 */
struct ipu7_msg_term_event {
	/**
	 *  ABI message header.  Type will be IPU_MSG_TYPE_TERM_EVENT.
	 *  user_token will be taken from the task wherein the event occurred.
	 */
	struct ia_gofo_msg_header header;
	/**  Identifier of the graph containing the node context */
	uint8_t graph_id;
	/**  Identifier of the frame where the event occurred */
	uint8_t frame_id;
	/** Node context identifier as specified in the graph open message */
	uint8_t node_ctx_id;
	/**
	 *  Index of node context configuration profile (variant) for
	 *  the node task where the event occurred
	 */
	uint8_t profile_idx;
	/**  Identifier of the fragment where the event occurred */
	uint8_t frag_id;
	/** ID of terminal in the static node description. */
	uint8_t term_id;
	/** Terminal event who's occurrence is now being notified.  See ipu7_msg_term_event_type.  */
	uint8_t event_type;
	/** Reserved for alignment padding.  Set to zero. */
	uint8_t reserved[1];

	/**
	 * Event Timestamp.
	 *
	 * Should be set as close as possible to the event that triggered this notification message. Units
	 * are at the discretion of the sender, and there is no guarantee of time source sync or a
	 * proportional relationship with the client SW clock or even the stability of the time source.
	 *
	 * With all these caveats, this timestamp is only debug aid at best and should not be relied on for
	 * operational use.  The client SW can use the reception time of the message as the next best thing,
	 * if necessary.
	 */
	uint64_t event_ts;
};

#pragma pack(pop)

/** @} */

/** Compile time checks only - this function is not to be called! */
static inline void ipu7_msg_term_test_func(void)
{
	CHECK_ALIGN32(struct ipu7_msg_term);
	CHECK_ALIGN64(struct ipu7_msg_term_event);
}

#pragma pack(push, 1)

/**
 *  Device messages map for messages Device Open and Close
 *  @{
 */
#define IPU_MSG_DEVICE_SEND_MSG_ENABLED 1U
#define IPU_MSG_DEVICE_SEND_MSG_DISABLED 0U

#define IPU_MSG_DEVICE_OPEN_SEND_RESP (1U << 0U)
#define IPU_MSG_DEVICE_OPEN_SEND_IRQ (1U << 1U)

#define IPU_MSG_DEVICE_CLOSE_SEND_RESP (1U << 0U)
#define IPU_MSG_DEVICE_CLOSE_SEND_IRQ (1U << 1U)
/** @} */

/**
 * Device open message.  Must be sent before any other message.
 * Negotiates protocol version with firmare and sets global parameters
 * In products with both secured and non-secure drivers,
 * this message must be sent by both drivers separately, each on
 * its own device queue.  This implies that each driver will have its
 * own logical instantiation of the same IPU hardware; We call this a
 * "device context".
 *
 * No other message types may be sent until this one is positively ACK'd
 */
struct ipu7_msg_dev_open {
	/** Common ABI message header.  Type will be IPU_MSG_TYPE_DEV_OPEN */
	struct ia_gofo_msg_header header;

	/** Maximum number of graphs that may be opened for this device */
	uint32_t max_graphs;

	/**
	 * Device messages map, configurable send response message and interrupt trigger for that message
	 *
	 * See IPU_MSG_DEVICE_OPEN_SEND for more information about supported device messages.
	 */
	uint8_t dev_msg_map;

	/**
	 * Enables power gating for PSYS power domain.
	 * Write 1 to enable power save mode, 0 to disable it.
	 */
	uint8_t enable_power_gating;

	/** Reserved for alignment.  Set to zero. */
	uint8_t reserved[2];
};

/** Acknowledges a device open message */
struct ipu7_msg_dev_open_ack {
	/**
	 * ABI ACK message header.  Includes the common message header too.
	 * Type will be IPU_MSG_TYPE_DEV_OPEN_ACK
	 */
	struct ia_gofo_msg_header_ack header;
};

/**
 * Device close message.  No other messages except device open
 * may be sent after this message is sent.
 * In products with both secured and non-secure drivers, this message must
 * be sent by the secure driver.
 */
struct ipu7_msg_dev_close {
	/** Common ABI message header.  Type will be IPU_MSG_TYPE_DEV_CLOSE */
	struct ia_gofo_msg_header header;

	/**
	 * Device messages map, configurable send response message and interrupt trigger for that message
	 *
	 * See IPU_MSG_DEVICE_CLOSE_SEND for more information about supported device messages.
	 */
	uint8_t dev_msg_map;

	/** Reserved for alignment.  Set to zero. */
	uint8_t reserved[7];
};

/** Acknowledges a device close message. */
struct ipu7_msg_dev_close_ack {
	/**
	 * ABI ACK message header.  Includes the common message header too.
	 * Type will be IPU_MSG_TYPE_DEV_CLOSE_ACK
	 */
	struct ia_gofo_msg_header_ack header;
};

/**
 *  Error detail enumeration for error group IPU_MSG_ERR_GROUP_DEVICE used exclusively
 *  in the device open ack and device close ack messages
 */
enum ipu7_msg_err_device {
	/** No error */
	IPU_MSG_ERR_DEVICE_OK = IA_GOFO_MSG_ERR_OK,
	/**
	 *  Attempt to reserve too many graphs in a device context.  err_detail[0] is the
	 *  offending max_graphs. err_detail[1] is firmware maximum.
	 */
	IPU_MSG_ERR_DEVICE_MAX_GRAPHS = 1,
	/** Message map configuration is wrong, Send msg response disabled but IRQ for that msg enabled */
	IPU_MSG_ERR_DEVICE_MSG_MAP = 2,
	/** Size of enumeration */
	IPU_MSG_ERR_DEVICE_N
};

#pragma pack(pop)

/** @} */

/** Compile time checks only - this function is not to be called! */
static inline void ipu7_msg_device_test_func(void)
{
	CHECK_ALIGN32(struct ia_gofo_version_s);
	CHECK_ALIGN64(struct ipu7_msg_dev_open);
	CHECK_ALIGN64(struct ipu7_msg_dev_open_ack);
	CHECK_ALIGN64(struct ipu7_msg_dev_close);
	CHECK_ALIGN64(struct ipu7_msg_dev_close_ack);
}

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
struct ipu7_msg_graph_open {
	/** Common ABI message header.  Type will be IPU_MSG_TYPE_GRAPH_OPEN */
	struct ia_gofo_msg_header header;
	/**
	 * List of *enabled* node objects that are active in the graph.
	 * Must be of type ipu7_msg_node_t or a derivative.
	 */
	struct ia_gofo_tlv_list nodes;
	/**
	 * List of *enabled* link objects.  Must be a of type ipu7_msg_link or a derivative.
	 * Links not specified here are assumed to be to/from
	 * the host and with full buffer atomicity.
	 */
	struct ia_gofo_tlv_list links;
	/**
	 * Identifier of graph to be opened.  Must be 0 <= graph_id < max_graphs,
	 * where max_graphs was declared in the device_open message.  See ipu7_msg_dev_open.
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

/** Enumeration of fgraph open ACK option types for use in ipu7_msg_graph_open_ack */
enum ipu7_msg_graph_ack_option_types {
	/** All TLV headers must define value zero as padding */
	IPU_MSG_GRAPH_ACK_OPTION_TYPES_PADDING = 0,
	/**
	 * Queue info set by FW for each managed node.
	 * See ipu7_msg_graph_open_ack_node_info_t
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
struct ipu7_msg_graph_open_ack_task_q_info {
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
 * Extension option types are defined in ipu7_msg_graph_ack_option_types
 */
struct ipu7_msg_graph_open_ack {
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
struct ipu7_msg_graph_close {
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
struct ipu7_msg_graph_close_ack {
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
enum ipu7_msg_err_graph {
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
static inline void ipu7_msg_graph_test_func(void)
{
	CHECK_ALIGN64(struct ipu7_msg_graph_open);
	CHECK_ALIGN64(struct ipu7_msg_graph_open_ack);
	CHECK_ALIGN64(struct ipu7_msg_graph_close);
	CHECK_ALIGN64(struct ipu7_msg_graph_close_ack);
	CHECK_ALIGN32(struct ipu7_msg_graph_open_ack_task_q_info);
}

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

#endif