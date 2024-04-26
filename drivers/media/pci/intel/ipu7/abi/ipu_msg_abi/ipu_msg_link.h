// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2013 - 2024 Intel Corporation

#ifndef IPU_MSG_LINK_H_INCLUDED__
#define IPU_MSG_LINK_H_INCLUDED__

/**
 * @addtogroup ipu_msg_abi
 * @{
 */

#include "ipu_msg_header.h"

#pragma pack(push, 1)

/** Type of link for PARSING.  Currently only support a generic type. */
enum ipu_msg_link_type {
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
enum ipu_msg_link_option_types {
	/** Type zero is always padding */
	IPU_MSG_LINK_OPTION_TYPES_PADDING = 0,
	/** Compression parameters - see ipu_msg_link_cmprs_option  */
	IPU_MSG_LINK_OPTION_TYPES_CMPRS = 1,
	/** Number of link options */
	IPU_MSG_LINK_OPTION_TYPES_N
};

/**
 * Possible values for struct ipu_msg_link_cmprs_plane_desc.bit_depth.
 * Added here as an enum for reference only.
 */
enum ipu_msg_link_cmprs_option_bit_depth {
	IPU_MSG_LINK_CMPRS_OPTION_8BPP = 0,
	IPU_MSG_LINK_CMPRS_OPTION_10BPP = 1,
	IPU_MSG_LINK_CMPRS_OPTION_12BPP = 2,
};

/**
 *  Denominator value for usage of space saving ratio field
 *
 *  See ipu_msg_link_cmprs_option::space_saving_ratio_numerator
 */
#define IPU_MSG_LINK_CMPRS_SPACE_SAVING_DENOM (128U)

/** Number of lossy compression algorithm parameters. */
#define IPU_MSG_LINK_CMPRS_LOSSY_CFG_PAYLOAD_SIZE (5U)

/**
 *  Maximum configurable space saving ratio numerator for footprint compression
 *
 *  See ipu_msg_link_cmprs_option::space_saving_ratio_numerator
 */
#define IPU_MSG_LINK_CMPRS_SPACE_SAVING_NUM_MAX (IPU_MSG_LINK_CMPRS_SPACE_SAVING_DENOM - 1U)

/**
 *  Per-plane information required for compression
 */
struct ipu_msg_link_cmprs_plane_desc {
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
	 * See enum ipu_msg_link_cmprs_option_bit_depth.
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

/** Value for ipu_msg_link_cmprs_option::align_interval when streaming is disabled */
#define IPU_MSG_LINK_CMPRS_NO_ALIGN_INTERVAL (0U)

/** Maximum value allowed in ipu_msg_link_cmprs_option::align_interval when streaming is enabled */
#define IPU_MSG_LINK_CMPRS_MIN_ALIGN_INTERVAL (16U)

/** Maximum value allowed in ipu_msg_link_cmprs_option::align_interval */
#define IPU_MSG_LINK_CMPRS_MAX_ALIGN_INTERVAL (1024U)

/**
 * Additional node-task request data specific to the compression on
 * links between nodes.  Only relevant for link endpoints where compression support
 * is available on both sides of the link.
 *
 * If compression is not enabled for the link, this option must not be instantiated.
 *
 */
struct ipu_msg_link_cmprs_option {
	/** Header common to all link options.  TLV type ID here is IPU_MSG_LINK_OPTION_TYPES_CMPRS. */
	struct ia_gofo_tlv_header header;

	/**
	 *  Size of the buffer allocated for the compressed frame, including all pixel planes
	 *  and any tile status planes.
	 *
	 *  May be larger than the uncompressed buffer size (see ipu_msg_term::payload_size) in
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
	struct ipu_msg_link_cmprs_plane_desc plane_descs[IPU_MSG_LINK_CMPRS_MAX_PLANES];
};

/** EndPoint of a link comprised of node and terminal ID's */
struct ipu_msg_link_ep {
	/** Node ctx ID as described in the list of nodes in the fgraph */
	uint8_t node_ctx_id;
	/** Node terminal ID from manifest */
	uint8_t term_id;
};

/** EndPoint pair uniquely indentifying the link */
struct ipu_msg_link_ep_pair {
	/** Source side of the link */
	struct ipu_msg_link_ep ep_src;
	/** Destination (sink) side of the link */
	struct ipu_msg_link_ep ep_dst;
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
struct ipu_msg_link {
	/**Type here is one of ipu_msg_link_type */
	struct ia_gofo_tlv_header tlv_header;
	/** Identifer of the link's terminal endpoints */
	struct ipu_msg_link_ep_pair endpoints;
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
static inline void ipu_msg_abi_link_test_func(void)
{
	CHECK_ALIGN32(struct ipu_msg_link);
	CHECK_ALIGN32(struct ipu_msg_link_cmprs_option);
	CHECK_ALIGN32(struct ipu_msg_link_cmprs_plane_desc);
}

#endif
