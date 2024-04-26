// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2013 - 2024 Intel Corporation

#ifndef IPU_INSYS_ABI_H_INCLUDED__
#define IPU_INSYS_ABI_H_INCLUDED__

/**
 * @defgroup insys_msg_abi Elements for INSYS Message ABI
 * @{
 */

#include "ipu_insys_types.h"
#include "ipu_insys_q_defs.h"
#include "ia_gofo_common_abi.h"
#include "ia_gofo_msg_link.h"
#include "ia_gofo_msg_err.h"

#pragma pack(push, 1)

/**
 *  @brief Generic resolution structure
 */
struct ipu_insys_resolution {
	/** Width */
	uint32_t width;
	/** Height */
	uint32_t height;
};

/**
 *  @brief Output pin payload
 */
struct ipu_insys_capture_output_pin_payload {
	/** Points to output pin buffer - buffer identifier */
	uint64_t user_token;
	/** Points to output pin buffer - CSS Virtual Address */
	ia_gofo_addr_t addr;
	/** Padding */
	uint8_t pad[4];
};

/**
 *  @brief define to where the output pin connected
 */
struct ipu_insys_output_link {
	/**
	 * Number of Allocated lines in output buffer
	 *
	 * Only used in BCSM, set to 0 if no use.
	 *  this is the number of lines available for cyclical use and MUST be smaller than
	 *  the output image height.
	 */
	uint32_t buffer_lines;
	/** Globally unique identifier for links that connect to a node outside of the local graph */
	uint16_t foreign_key;
	/**
	 * number of lines to be written before sending a pointer update
	 * - must be multiple of height
	 */
	uint16_t granularity_pointer_update;
	/** Data movement mechanisms between producer and consumer, see ia_gofo_msg_link_streaming_mode */
	uint8_t msg_link_streaming_mode;
	/**
	 * PBK instance for cross IPs connection.
	 * When use ISYS internal PBK use IPU_MSG_LINK_PBK_ID_DONT_CARE.
	 * Links with SOFF streaming mode must use IPU_MSG_LINK_PBK_ID_DONT_CARE.
	 *
	 * @see enum ia_gofo_soc_pbk_instance_id
	 */
	uint8_t pbk_id;
	/**
	 * Relevant entry id in the PBK that manages producer-consumer handshake.
	 * When use ISYS internal PBK use IPU_MSG_LINK_PBK_ID_DONT_CARE.
	 * Links with SOFF streaming mode must use IPU_MSG_LINK_PBK_SLOT_ID_DONT_CARE.
	 */
	uint8_t pbk_slot_id;
	/** to which ip the link connected (ddr/psys/...) , see ipu_insys_output_link_dest */
	uint8_t dest;
	/**
	 * is the pbk managed by hw or sw
	 *
	 * - HW managed -
	 * HW will confirm that the link buffer has been fully consumed by the consumer before allowed the
	 * producer to transmit the next frame.
	 *
	 * -SW managed-
	 * the SW verify the shared buffer is not been used by anyone else, including consumers of the
	 * previous frames.
	 *
	 * Usage: HW_MANAGED - 0, SW_MANAGED = 1
	 *	BCSM single or private buffer: HW_MANAGED
	 *	BCSM double buffer: SW_MANAGED
	 *	Other: SW_MANAGED
	 */
	uint8_t use_sw_managed;
	uint8_t pad[3];
};

/**
 *  @brief define the output cropping
 *  if both line top and line bot are 0, cropping is disabled.
 */
struct ipu_insys_output_cropping {
	/** When cropping is enabled, all lines before this line will be cropped. */
	uint16_t line_top;
	/** When cropping is enabled, all lines after this line will be cropped. */
	uint16_t line_bottom;
};

/**
 *  @brief Output decompression
 */
struct ipu_insys_output_dpcm {
	/** 0 - dpcm disabled, 1 enabled */
	uint8_t enable;
	/** type of compression, see ipu_insys_dpcm_type */
	uint8_t type;
	/** predictor of compression, see ipu_insys_dpcm_predictor */
	uint8_t predictor;
	/** not used */
	uint8_t pad;
};

/**
 *  @brief Output Pin information and details
 */
struct ipu_insys_output_pin {
	/** output pin link */
	struct ipu_insys_output_link link;
	/** output pin crop */
	struct ipu_insys_output_cropping crop;
	/** output decompression */
	struct ipu_insys_output_dpcm dpcm;
	/** output stride in Bytes (not valid for statistics) */
	uint32_t stride;
	/** frame format type, see ipu_insys_frame_format_type */
	uint16_t ft;
	/** assert if pin event should trigger irq */
	uint8_t send_irq;
	/** input pin id/index which is source of the data for this output pin */
	uint8_t input_pin_id;
	/**
	 * Indicating an early acknowledge.
	 * 1 - Will configure an early ack
	 * 0 - will use default behavior of end of frame ack.
	 */
	uint8_t early_ack_en;
	/** Padding */
	uint8_t pad[3];
};

/**
 * @brief Stream-level configuration of a single output pin affecting all frames in the stream
 */
struct ipu_insys_input_pin {
	/** input resolution */
	struct ipu_insys_resolution input_res;
	/**
	 *  Sync Event messages configuration map
	 *
	 *  Consists of different event message types.
	 *  Each type can be either configured to send event message response, send an interrupt with event message.
	 *  When only the interrupt option is configured by setting the relevant bit,
	 *  the event message will be sent together with an interrupt.
	 */
	uint16_t sync_msg_map;
	/** mipi data type, see ipu_insys_mipi_data_type */
	uint8_t dt;
	/**
	 * Disable unpacking of the MIPI packet
	 * store the incoming stream as is to DDR without the header
	 * (the header will be removed)
	 */
	uint8_t disable_mipi_unpacking;
	/** Defines whether MIPI data is encapsulated in some other data type, see ipu_insys_mipi_dt_rename_mode */
	uint8_t dt_rename_mode;
	/** mapped_dt */
	uint8_t mapped_dt;
	uint8_t pad[2];
};

 /**
  * @brief ISYS stream configuration top level data structure
 */
struct ipu_insys_stream_cfg {
	/** Input pin descriptors */
	struct ipu_insys_input_pin input_pins[MAX_IPINS];
	/** Output pin descriptors */
	struct ipu_insys_output_pin output_pins[MAX_OPINS];
	/**
	 *  Map describing Stream enabled messages Response Send and Interrupt
	 */
	uint16_t stream_msg_map;
	/** Specifies port_id see enum ipu_insys_mipi_port  */
	uint8_t port_id;
	/** MIPI Virtual Channel (up to 16 virtual per physical channel), see ipu_insys_mipi_vc */
	uint8_t vc;
	/** Number of input pins */
	uint8_t nof_input_pins;
	/** Number of output pins */
	uint8_t nof_output_pins;
	uint8_t pad[2];
};

/**
 *  @brief Frame buffer set
 */
struct ipu_insys_buffset {
	/** Output pin addresses */
	struct ipu_insys_capture_output_pin_payload output_pins[MAX_OPINS];
	/** Capture message map, see IPU_INSYS_FRAME_MSG */
	uint8_t capture_msg_map;
	/** Frame number associated with this buffer set */
	uint8_t frame_id;
	/*
	 * Intentional frame drop boolean flag.
	 * 0 for no drop, else drop frame.
	 */
	uint8_t skip_frame;
	/** Padding */
	uint8_t pad[5];
};

/**
 *  @brief Response information
 */
struct ipu_insys_resp {
	/**  */
	uint64_t buf_id;
	/** This var is only valid for pin event related responses, contains pin addresses */
	struct ipu_insys_capture_output_pin_payload pin;
	/**
	 *  Generic error structure.  err_header.error_code == 0 means ACK,
	 *  all others are an error
	 */
	struct ia_gofo_msg_err error_info;
	/** Time information for an event when available */
	uint32_t timestamp[2];
	/** Response type, see ipu_insys_resp_type */
	uint8_t type;
	/** Data movement mechanisms between producer and consumer, see ia_gofo_msg_link_streaming_mode */
	uint8_t msg_link_streaming_mode;
	/** Stream ID this response corresponds to */
	uint8_t stream_id;
	/** Pin ID that the pin payload corresponds to */
	uint8_t pin_id;
	/** Valid for STREAM_START_AND_CAPTURE_DONE, STREAM_CAPTURE_DONE and STREAM_CAPTURE_DISCARDED */
	uint8_t frame_id;
	/** Confirmation of intentional frame drop. See ipu_insys_buffset::skip_frame */
	uint8_t skip_frame;
	/** Padding */
	uint8_t pad[2];
};

/**
 *  @brief Response queue token
 */
struct ipu_insys_resp_queue_token {
	/** Response information */
	struct ipu_insys_resp resp_info;
};

/**
 *  @brief Send queue token
 */
struct ipu_insys_send_queue_token {
	/** Buffer handle  */
	uint64_t buf_handle;
	/** addr */
	ia_gofo_addr_t addr;
	/** Stream ID */
	uint16_t stream_id;
	/** Send type, see ipu_insys_send_type */
	uint8_t send_type;
	/** Flags provided with the message, see ipu_insys_send_queue_token_flag */
	uint8_t flag;
};

#pragma pack(pop)

/** @} */

/** Compile time checks only - this function is not to be called! */
static inline void ipu_insys_types_test_func(void)
{
	CHECK_ALIGN32(struct ipu_insys_resolution);
	CHECK_ALIGN64(struct ipu_insys_capture_output_pin_payload);
	CHECK_ALIGN32(struct ipu_insys_output_pin);
	CHECK_ALIGN32(struct ipu_insys_input_pin);
	CHECK_ALIGN32(struct ipu_insys_output_cropping);
	CHECK_ALIGN32(struct ipu_insys_stream_cfg);
	CHECK_ALIGN64(struct ipu_insys_buffset);
	CHECK_ALIGN64(struct ipu_insys_resp);
	CHECK_ALIGN64(struct ipu_insys_resp_queue_token);
	CHECK_ALIGN64(struct ipu_insys_send_queue_token);
	CHECK_ALIGN32(struct ipu_insys_output_link);
}

#endif
