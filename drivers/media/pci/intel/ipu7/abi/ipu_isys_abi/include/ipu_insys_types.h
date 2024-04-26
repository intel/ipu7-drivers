// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2013 - 2024 Intel Corporation

#ifndef IPU_INSYS_TYPES_H_INCLUDED__
#define IPU_INSYS_TYPES_H_INCLUDED__

#include "ipu_insys_q_defs.h"

/**
 * @addtogroup insys_msg_abi
 * @{
 */

/** Max number of Input/Output Pins */
#define MAX_IPINS (4U)
#define MAX_OPINS (4U)

/** HW limitation for max number of OPINS for a single IPIN */
#define MAX_OPINS_FOR_SINGLE_IPINS (3U)

/*
 * Consider 1 slot per stream since driver is not expected to pipeline
 * device commands for the same stream
 */
#define DEV_SEND_QUEUE_SIZE (IPU_INSYS_STREAM_ID_MAX)

/** Max number of planes for frame formats supported by the FW */
#define PIN_PLANES_MAX (4U)

/*
 * ipu_insys_return_token: data item of exacly 8 bytes (64 bits)
 * which can be used to pass a return token back to the host
*/
typedef uint64_t ipu_insys_return_token;

/** Response type enumeration for response messages to commands (FW to SW) */
enum ipu_insys_resp_type {
	IPU_INSYS_RESP_TYPE_STREAM_OPEN_DONE = 0,
	IPU_INSYS_RESP_TYPE_STREAM_START_AND_CAPTURE_ACK = 1,
	IPU_INSYS_RESP_TYPE_STREAM_CAPTURE_ACK = 2,
	IPU_INSYS_RESP_TYPE_STREAM_ABORT_ACK = 3,
	IPU_INSYS_RESP_TYPE_STREAM_FLUSH_ACK = 4,
	IPU_INSYS_RESP_TYPE_STREAM_CLOSE_ACK = 5,
	IPU_INSYS_RESP_TYPE_PIN_DATA_READY = 6,
	IPU_INSYS_RESP_TYPE_FRAME_SOF = 7,
	IPU_INSYS_RESP_TYPE_FRAME_EOF = 8,
	IPU_INSYS_RESP_TYPE_STREAM_START_AND_CAPTURE_DONE = 9,
	IPU_INSYS_RESP_TYPE_STREAM_CAPTURE_DONE = 10,
	IPU_INSYS_RESP_TYPE_PWM_IRQ = 11,
	N_IPU_INSYS_RESP_TYPE
};

/** Command type enumeration for command messages (SW to FW) */
enum ipu_insys_send_type {
	IPU_INSYS_SEND_TYPE_STREAM_OPEN = 0,
	IPU_INSYS_SEND_TYPE_STREAM_START_AND_CAPTURE = 1,
	IPU_INSYS_SEND_TYPE_STREAM_CAPTURE = 2,
	IPU_INSYS_SEND_TYPE_STREAM_ABORT = 3,
	IPU_INSYS_SEND_TYPE_STREAM_FLUSH = 4,
	IPU_INSYS_SEND_TYPE_STREAM_CLOSE = 5,
	N_IPU_INSYS_SEND_TYPE
};

/**
 * @brief Number of VCs supported port. Equal for all ports.
 * MIPI csi2 spec 2.0 supports up to 16 virtual per physical channel
 */
enum ipu_insys_mipi_vc {
	IPU_INSYS_MIPI_VC_0 = 0,
	IPU_INSYS_MIPI_VC_1 = 1,
	IPU_INSYS_MIPI_VC_2 = 2,
	IPU_INSYS_MIPI_VC_3 = 3,
	IPU_INSYS_MIPI_VC_4 = 4,
	IPU_INSYS_MIPI_VC_5 = 5,
	IPU_INSYS_MIPI_VC_6 = 6,
	IPU_INSYS_MIPI_VC_7 = 7,
	IPU_INSYS_MIPI_VC_8 = 8,
	IPU_INSYS_MIPI_VC_9 = 9,
	IPU_INSYS_MIPI_VC_10 = 10,
	IPU_INSYS_MIPI_VC_11 = 11,
	IPU_INSYS_MIPI_VC_12 = 12,
	IPU_INSYS_MIPI_VC_13 = 13,
	IPU_INSYS_MIPI_VC_14 = 14,
	IPU_INSYS_MIPI_VC_15 = 15,
	N_IPU_INSYS_MIPI_VC
};

/**
 * @brief Number of ports supported .
 */
enum ipu_insys_mipi_port {
	IPU_INSYS_MIPI_PORT_0 = 0,
	IPU_INSYS_MIPI_PORT_1 = 1,
	IPU_INSYS_MIPI_PORT_2 = 2,
	IPU_INSYS_MIPI_PORT_3 = 3,
	IPU_INSYS_MIPI_PORT_4 = 4,
	IPU_INSYS_MIPI_PORT_5 = 5,
	NA_IPU_INSYS_MIPI_PORT
};

/** Supported Pixel Frame formats. Expandable if needed */
enum ipu_insys_frame_format_type {
	IPU_INSYS_FRAME_FORMAT_NV11 = 0,/**< 12 bit YUV 411, Y, UV plane */
	IPU_INSYS_FRAME_FORMAT_NV12 = 1,/**< 12 bit YUV 420, Y, UV plane */
	IPU_INSYS_FRAME_FORMAT_NV12_16 = 2,/**< 16 bit YUV 420, Y, UV plane */
	IPU_INSYS_FRAME_FORMAT_NV12_TILEY = 3,/**< 12 bit YUV 420, Intel
						 *  proprietary tiled format,
						 *  TileY
						 */
	IPU_INSYS_FRAME_FORMAT_NV16 = 4,/**< 16 bit YUV 422, Y, UV plane */
	IPU_INSYS_FRAME_FORMAT_NV21 = 5,/**< 12 bit YUV 420, Y, VU plane */
	IPU_INSYS_FRAME_FORMAT_NV61 = 6,/**< 16 bit YUV 422, Y, VU plane */
	IPU_INSYS_FRAME_FORMAT_YV12 = 7,/**< 12 bit YUV 420, Y, V, U plane */
	IPU_INSYS_FRAME_FORMAT_YV16 = 8,/**< 16 bit YUV 422, Y, V, U plane */
	IPU_INSYS_FRAME_FORMAT_YUV420 = 9,/**< 12 bit YUV 420, Y, U, V plane */
	IPU_INSYS_FRAME_FORMAT_YUV420_10 = 10,/**< yuv420, 10 bits per subpixel */
	IPU_INSYS_FRAME_FORMAT_YUV420_12 = 11,/**< yuv420, 12 bits per subpixel */
	IPU_INSYS_FRAME_FORMAT_YUV420_14 = 12,/**< yuv420, 14 bits per subpixel */
	IPU_INSYS_FRAME_FORMAT_YUV420_16 = 13,/**< yuv420, 16 bits per subpixel */
	IPU_INSYS_FRAME_FORMAT_YUV422 = 14,/**< 16 bit YUV 422, Y, U, V plane */
	IPU_INSYS_FRAME_FORMAT_YUV422_16 = 15,/**< yuv422, 16 bits per subpixel */
	IPU_INSYS_FRAME_FORMAT_UYVY = 16,/**< 16 bit YUV 422, UYVY interleaved */
	IPU_INSYS_FRAME_FORMAT_YUYV = 17,/**< 16 bit YUV 422, YUYV interleaved */
	IPU_INSYS_FRAME_FORMAT_YUV444 = 18,/**< 24 bit YUV 444, Y, U, V plane */
	IPU_INSYS_FRAME_FORMAT_YUV_LINE = 19,/**< Internal format, 2 y lines
					  *   followed by a uvinterleaved line
					  */
	IPU_INSYS_FRAME_FORMAT_RAW8 = 20,	/**< RAW8, 1 plane */
	IPU_INSYS_FRAME_FORMAT_RAW10 = 21,	/**< RAW10, 1 plane */
	IPU_INSYS_FRAME_FORMAT_RAW12 = 22,	/**< RAW12, 1 plane */
	IPU_INSYS_FRAME_FORMAT_RAW14 = 23,	/**< RAW14, 1 plane */
	IPU_INSYS_FRAME_FORMAT_RAW16 = 24,	/**< RAW16, 1 plane */
	IPU_INSYS_FRAME_FORMAT_RGB565 = 25,/**< 16 bit RGB, 1 plane. Each 3 sub
					 *  pixels are packed into one 16 bit
					 *  value, 5 bits for R, 6 bits for G
					 *  and 5 bits for B.
					 */
	IPU_INSYS_FRAME_FORMAT_PLANAR_RGB888 = 26,	/**< 24 bit RGB, 3 planes */
	IPU_INSYS_FRAME_FORMAT_RGBA888 = 27,/**< 32 bit RGBA, 1 plane,
					 *   A=Alpha (alpha is unused)
					 */
	IPU_INSYS_FRAME_FORMAT_QPLANE6 = 28,/**< Internal, for advanced ISP */
	IPU_INSYS_FRAME_FORMAT_BINARY_8 = 29,/**< byte stream, used for jpeg. */
	N_IPU_INSYS_FRAME_FORMAT
};

/** Temporary for driver compatibility */
#define IPU_INSYS_FRAME_FORMAT_RAW (IPU_INSYS_FRAME_FORMAT_RAW16)

/**
 *  Supported MIPI data type. Keep in sync array in ia_css_isys_private.c
 */
enum ipu_insys_mipi_data_type {
	/** SYNCHRONIZATION SHORT PACKET DATA TYPES */
	IPU_INSYS_MIPI_DATA_TYPE_FRAME_START_CODE	= 0x00,
	IPU_INSYS_MIPI_DATA_TYPE_FRAME_END_CODE	= 0x01,
	IPU_INSYS_MIPI_DATA_TYPE_LINE_START_CODE	= 0x02,	/* Optional */
	IPU_INSYS_MIPI_DATA_TYPE_LINE_END_CODE	= 0x03,	/* Optional */
	/** Reserved 0x04-0x07 */
	IPU_INSYS_MIPI_DATA_TYPE_RESERVED_0x04	= 0x04,
	IPU_INSYS_MIPI_DATA_TYPE_RESERVED_0x05	= 0x05,
	IPU_INSYS_MIPI_DATA_TYPE_RESERVED_0x06	= 0x06,
	IPU_INSYS_MIPI_DATA_TYPE_RESERVED_0x07	= 0x07,
	/** GENERIC SHORT PACKET DATA TYPES
	 * They are used to keep the timing information for the
	 * opening/closing of shutters, triggering of flashes and etc.
	 */
	/** Generic Short Packet Code 1 */
	IPU_INSYS_MIPI_DATA_TYPE_GENERIC_SHORT1	= 0x08,
	/** Generic Short Packet Code 2 */
	IPU_INSYS_MIPI_DATA_TYPE_GENERIC_SHORT2	= 0x09,
	/** Generic Short Packet Code 3 */
	IPU_INSYS_MIPI_DATA_TYPE_GENERIC_SHORT3	= 0x0A,
	/** Generic Short Packet Code 4 */
	IPU_INSYS_MIPI_DATA_TYPE_GENERIC_SHORT4	= 0x0B,
	/** Generic Short Packet Code 5 */
	IPU_INSYS_MIPI_DATA_TYPE_GENERIC_SHORT5	= 0x0C,
	/** Generic Short Packet Code 6 */
	IPU_INSYS_MIPI_DATA_TYPE_GENERIC_SHORT6	= 0x0D,
	/** Generic Short Packet Code 7 */
	IPU_INSYS_MIPI_DATA_TYPE_GENERIC_SHORT7	= 0x0E,
	/** Generic Short Packet Code 8 */
	IPU_INSYS_MIPI_DATA_TYPE_GENERIC_SHORT8	= 0x0F,
	/** GENERIC LONG PACKET DATA TYPES */
	IPU_INSYS_MIPI_DATA_TYPE_NULL			= 0x10,
	IPU_INSYS_MIPI_DATA_TYPE_BLANKING_DATA	= 0x11,
	/** Embedded 8-bit non Image Data */
	IPU_INSYS_MIPI_DATA_TYPE_EMBEDDED		= 0x12,
	/** Reserved 0x13-0x17 */
	IPU_INSYS_MIPI_DATA_TYPE_RESERVED_0x13	= 0x13,
	IPU_INSYS_MIPI_DATA_TYPE_RESERVED_0x14	= 0x14,
	IPU_INSYS_MIPI_DATA_TYPE_RESERVED_0x15	= 0x15,
	IPU_INSYS_MIPI_DATA_TYPE_RESERVED_0x16	= 0x16,
	IPU_INSYS_MIPI_DATA_TYPE_RESERVED_0x17	= 0x17,
	/** YUV DATA TYPES */
	/** 8 bits per subpixel */
	IPU_INSYS_MIPI_DATA_TYPE_YUV420_8		= 0x18,
	/** 10 bits per subpixel */
	IPU_INSYS_MIPI_DATA_TYPE_YUV420_10		= 0x19,
	/** 8 bits per subpixel */
	IPU_INSYS_MIPI_DATA_TYPE_YUV420_8_LEGACY	= 0x1A,
	/** Reserved 0x1B */
	IPU_INSYS_MIPI_DATA_TYPE_RESERVED_0x1B	= 0x1B,
	/** YUV420 8-bit (Chroma Shifted Pixel Sampling) */
	IPU_INSYS_MIPI_DATA_TYPE_YUV420_8_SHIFT	= 0x1C,
	/** YUV420 10-bit (Chroma Shifted Pixel Sampling) */
	IPU_INSYS_MIPI_DATA_TYPE_YUV420_10_SHIFT	= 0x1D,
	/** UYVY..UVYV, 8 bits per subpixel */
	IPU_INSYS_MIPI_DATA_TYPE_YUV422_8		= 0x1E,
	/** UYVY..UVYV, 10 bits per subpixel */
	IPU_INSYS_MIPI_DATA_TYPE_YUV422_10		= 0x1F,
	/** RGB DATA TYPES */
	IPU_INSYS_MIPI_DATA_TYPE_RGB_444		= 0x20,
	/** BGR..BGR, 5 bits per subpixel */
	IPU_INSYS_MIPI_DATA_TYPE_RGB_555		= 0x21,
	/** BGR..BGR, 5 bits B and R, 6 bits G */
	IPU_INSYS_MIPI_DATA_TYPE_RGB_565		= 0x22,
	/** BGR..BGR, 6 bits per subpixel */
	IPU_INSYS_MIPI_DATA_TYPE_RGB_666		= 0x23,
	/** BGR..BGR, 8 bits per subpixel */
	IPU_INSYS_MIPI_DATA_TYPE_RGB_888		= 0x24,
	/** Reserved 0x25-0x27 */
	IPU_INSYS_MIPI_DATA_TYPE_RESERVED_0x25	= 0x25,
	IPU_INSYS_MIPI_DATA_TYPE_RESERVED_0x26	= 0x26,
	IPU_INSYS_MIPI_DATA_TYPE_RESERVED_0x27	= 0x27,
	/** RAW DATA TYPES */
	/** RAW data, 6 bits per pixel */
	IPU_INSYS_MIPI_DATA_TYPE_RAW_6		= 0x28,
	/** RAW data, 7 bits per pixel */
	IPU_INSYS_MIPI_DATA_TYPE_RAW_7		= 0x29,
	/** RAW data, 8 bits per pixel */
	IPU_INSYS_MIPI_DATA_TYPE_RAW_8		= 0x2A,
	/** RAW data, 10 bits per pixel */
	IPU_INSYS_MIPI_DATA_TYPE_RAW_10		= 0x2B,
	/** RAW data, 12 bits per pixel */
	IPU_INSYS_MIPI_DATA_TYPE_RAW_12		= 0x2C,
	/** RAW data, 14 bits per pixel */
	IPU_INSYS_MIPI_DATA_TYPE_RAW_14		= 0x2D,
	/** Reserved 0x2E-2F are used with assigned meaning */
	/** RAW data, 16 bits per pixel, not specified in CSI-MIPI standard */
	IPU_INSYS_MIPI_DATA_TYPE_RAW_16		= 0x2E,
	/** Binary byte stream, which is target at JPEG, not specified in
	 *  CSI-MIPI standard
	 */
	IPU_INSYS_MIPI_DATA_TYPE_BINARY_8		= 0x2F, /** Should be RAW20 **/
	/** USER DEFINED 8-BIT DATA TYPES */
	/** For example, the data transmitter (e.g. the SoC sensor) can keep
	 * the JPEG data as the User Defined Data Type 4 and the MPEG data as
	 * the User Defined Data Type 7.
	 */
	/** User defined 8-bit data type 1 */
	IPU_INSYS_MIPI_DATA_TYPE_USER_DEF1		= 0x30,
	/** User defined 8-bit data type 2 */
	IPU_INSYS_MIPI_DATA_TYPE_USER_DEF2		= 0x31,
	/** User defined 8-bit data type 3 */
	IPU_INSYS_MIPI_DATA_TYPE_USER_DEF3		= 0x32,
	/** User defined 8-bit data type 4 */
	IPU_INSYS_MIPI_DATA_TYPE_USER_DEF4		= 0x33,
	/** User defined 8-bit data type 5 */
	IPU_INSYS_MIPI_DATA_TYPE_USER_DEF5		= 0x34,
	/** User defined 8-bit data type 6 */
	IPU_INSYS_MIPI_DATA_TYPE_USER_DEF6		= 0x35,
	/** User defined 8-bit data type 7 */
	IPU_INSYS_MIPI_DATA_TYPE_USER_DEF7		= 0x36,
	/** User defined 8-bit data type 8 */
	IPU_INSYS_MIPI_DATA_TYPE_USER_DEF8		= 0x37,
	/** Reserved 0x38-0x3F */
	IPU_INSYS_MIPI_DATA_TYPE_RESERVED_0x38	= 0x38,
	IPU_INSYS_MIPI_DATA_TYPE_RESERVED_0x39	= 0x39,
	IPU_INSYS_MIPI_DATA_TYPE_RESERVED_0x3A	= 0x3A,
	IPU_INSYS_MIPI_DATA_TYPE_RESERVED_0x3B	= 0x3B,
	IPU_INSYS_MIPI_DATA_TYPE_RESERVED_0x3C	= 0x3C,
	IPU_INSYS_MIPI_DATA_TYPE_RESERVED_0x3D	= 0x3D,
	IPU_INSYS_MIPI_DATA_TYPE_RESERVED_0x3E	= 0x3E,
	IPU_INSYS_MIPI_DATA_TYPE_RESERVED_0x3F	= 0x3F,

	/** Keep always last and max value */
	N_IPU_INSYS_MIPI_DATA_TYPE			= 0x40
};

/**
 * Describes if long MIPI packets have
 * DT with some other DT format.
 */
enum ipu_insys_mipi_dt_rename_mode {
	IPU_INSYS_MIPI_DT_NO_RENAME = 0,
	IPU_INSYS_MIPI_DT_RENAMED_MODE = 1,
	N_IPU_INSYS_MIPI_DT_MODE
};

/**
 * @brief IS Send Message message enable or disable values
 * @{
 */
#define IPU_INSYS_SEND_MSG_ENABLED 1U
#define IPU_INSYS_SEND_MSG_DISABLED 0U
/** @} */

/**
 * @brief Per stream configuration of sync messages
 *
 * Sync messages are configured per stream, send response and response with trigger.
 * These messages written to a map of one byte, raise bit to enable.
 * Each frame in this stream will have the very same configuration,
 * as it was configured in a stream.
 *
 * When enabling IRQ sync message one must enable send response as well.
 * Meaning there is no case for send interrupt without sending response message.
 *
 * It is possible to set bit to send response only, sending without interrupts.
 * @{
 */
#define IPU_INSYS_STREAM_SYNC_MSG_SEND_RESP_SOF (1U << 0U)
#define IPU_INSYS_STREAM_SYNC_MSG_SEND_RESP_EOF (1U << 1U)
#define IPU_INSYS_STREAM_SYNC_MSG_SEND_IRQ_SOF (1U << 2U)
#define IPU_INSYS_STREAM_SYNC_MSG_SEND_IRQ_EOF (1U << 3U)
#define IPU_INSYS_STREAM_SYNC_MSG_SEND_RESP_SOF_DISCARDED (1U << 4U)
#define IPU_INSYS_STREAM_SYNC_MSG_SEND_RESP_EOF_DISCARDED (1U << 5U)
#define IPU_INSYS_STREAM_SYNC_MSG_SEND_IRQ_SOF_DISCARDED (1U << 6U)
#define IPU_INSYS_STREAM_SYNC_MSG_SEND_IRQ_EOF_DISCARDED (1U << 7U)
/** @} */

/** Enable all Stream Sync Message Send Response */
#define IPU_INSYS_STREAM_SYNC_MSG_ENABLE_MSG_SEND_RESP ( \
	IPU_INSYS_STREAM_SYNC_MSG_SEND_RESP_SOF | \
	IPU_INSYS_STREAM_SYNC_MSG_SEND_RESP_EOF | \
	IPU_INSYS_STREAM_SYNC_MSG_SEND_RESP_SOF_DISCARDED | \
	IPU_INSYS_STREAM_SYNC_MSG_SEND_RESP_EOF_DISCARDED)

/** Enable all Stream Sync Message Send IRQ */
#define IPU_INSYS_STREAM_SYNC_MSG_ENABLE_MSG_SEND_IRQ ( \
	IPU_INSYS_STREAM_SYNC_MSG_SEND_IRQ_SOF | \
	IPU_INSYS_STREAM_SYNC_MSG_SEND_IRQ_EOF | \
	IPU_INSYS_STREAM_SYNC_MSG_SEND_IRQ_SOF_DISCARDED | \
	IPU_INSYS_STREAM_SYNC_MSG_SEND_IRQ_EOF_DISCARDED)

/**
 * @brief Per stream configuration of response messages and interrupt for these messages
 *
 * Each message that FW responds with can be configured to either send this message or not.
 * Each message can be sent as response but interrupt for this message can be configurable.
 * It is not possible to enable interrupt for a message but disable message itself.
 *
 * Send response meaning sending a message.
 * Send interrupt meaning sending an interrupt request right after a message was sent.
 *
 * @{
 */
#define IPU_INSYS_STREAM_MSG_SEND_RESP_STREAM_OPEN_DONE (1U << 0U)
#define IPU_INSYS_STREAM_MSG_SEND_IRQ_STREAM_OPEN_DONE (1U << 1U)
#define IPU_INSYS_STREAM_MSG_SEND_RESP_STREAM_START_ACK (1U << 2U)
#define IPU_INSYS_STREAM_MSG_SEND_IRQ_STREAM_START_ACK (1U << 3U)
#define IPU_INSYS_STREAM_MSG_SEND_RESP_STREAM_CLOSE_ACK (1U << 4U)
#define IPU_INSYS_STREAM_MSG_SEND_IRQ_STREAM_CLOSE_ACK (1U << 5U)
#define IPU_INSYS_STREAM_MSG_SEND_RESP_STREAM_FLUSH_ACK (1U << 6U)
#define IPU_INSYS_STREAM_MSG_SEND_IRQ_STREAM_FLUSH_ACK (1U << 7U)
/*
 * Note: there is uint32_t cast for (1U) as WA for the KW checker
 * MISRA.SHIFT.RANGE.201 which interprets 1U's type as unsigned char rather
 * than unsigned int.
 */
#define IPU_INSYS_STREAM_MSG_SEND_RESP_STREAM_ABORT_ACK ((uint32_t)(1U) << 8U)
#define IPU_INSYS_STREAM_MSG_SEND_IRQ_STREAM_ABORT_ACK ((uint32_t)(1U) << 9U)
/** @} */

/** Enable all Stream Message Send Response */
#define IPU_INSYS_STREAM_ENABLE_MSG_SEND_RESP ( \
	IPU_INSYS_STREAM_MSG_SEND_RESP_STREAM_OPEN_DONE | \
	IPU_INSYS_STREAM_MSG_SEND_RESP_STREAM_START_ACK | \
	IPU_INSYS_STREAM_MSG_SEND_RESP_STREAM_CLOSE_ACK | \
	IPU_INSYS_STREAM_MSG_SEND_RESP_STREAM_FLUSH_ACK | \
	IPU_INSYS_STREAM_MSG_SEND_RESP_STREAM_ABORT_ACK)

/** Enable all Stream Message Send IRQ */
#define IPU_INSYS_STREAM_ENABLE_MSG_SEND_IRQ ( \
	IPU_INSYS_STREAM_MSG_SEND_IRQ_STREAM_OPEN_DONE | \
	IPU_INSYS_STREAM_MSG_SEND_IRQ_STREAM_START_ACK | \
	IPU_INSYS_STREAM_MSG_SEND_IRQ_STREAM_CLOSE_ACK | \
	IPU_INSYS_STREAM_MSG_SEND_IRQ_STREAM_FLUSH_ACK | \
	IPU_INSYS_STREAM_MSG_SEND_IRQ_STREAM_ABORT_ACK)

/**
 * Per frame configuration of response or interrupt for these messages
 * Each message that FW responds with can be configured to either send IRQ for this message or not.
 *
 * It is not possible to enable interrupt for a message but disable message itself.
 * @{
 */
#define IPU_INSYS_FRAME_MSG_SEND_RESP_CAPTURE_ACK (1U << 0U)
#define IPU_INSYS_FRAME_MSG_SEND_IRQ_CAPTURE_ACK (1U << 1U)
#define IPU_INSYS_FRAME_MSG_SEND_RESP_CAPTURE_DONE (1U << 2U)
#define IPU_INSYS_FRAME_MSG_SEND_IRQ_CAPTURE_DONE (1U << 3U)
#define IPU_INSYS_FRAME_MSG_SEND_RESP_PIN_DATA_READY (1U << 4U)
#define IPU_INSYS_FRAME_MSG_SEND_IRQ_PIN_DATA_READY (1U << 5U)
/** @} */

/** Enable all Frame Message Send response message */
#define IPU_INSYS_FRAME_ENABLE_MSG_SEND_RESP ( \
	IPU_INSYS_FRAME_MSG_SEND_RESP_CAPTURE_ACK | \
	IPU_INSYS_FRAME_MSG_SEND_RESP_CAPTURE_DONE | \
	IPU_INSYS_FRAME_MSG_SEND_RESP_PIN_DATA_READY)

/** Enable all Frame Message Send IRQ */
#define IPU_INSYS_FRAME_ENABLE_MSG_SEND_IRQ ( \
	IPU_INSYS_FRAME_MSG_SEND_IRQ_CAPTURE_ACK | \
	IPU_INSYS_FRAME_MSG_SEND_IRQ_CAPTURE_DONE | \
	IPU_INSYS_FRAME_MSG_SEND_IRQ_PIN_DATA_READY)

/** Describe the destination options for output pins */
enum ipu_insys_output_link_dest {
	/**
	 *  Destination is memory *for CPU SW processing*
	 *
	 *  @note On platforms where bus snooping is supported but optional, snooping will be enabled for this
	 *        destination.
	 */
	IPU_INSYS_OUTPUT_LINK_DEST_MEM = 0,
	/** Destination is IPU PSYS for processing */
	IPU_INSYS_OUTPUT_LINK_DEST_PSYS = 1,
	/** Destination is any other non-CPU or non-IPU entity such as GPU */
	IPU_INSYS_OUTPUT_LINK_DEST_IPU_EXTERNAL = 2
};

/**
 * Describes type of decompression
 */
enum ipu_insys_dpcm_type {
	IPU_INSYS_DPCM_TYPE_DISABLED = 0,
	IPU_INSYS_DPCM_TYPE_10_8_10 = 1,
	IPU_INSYS_DPCM_TYPE_12_8_12 = 2,
	IPU_INSYS_DPCM_TYPE_12_10_12 = 3,
	N_IPU_INSYS_DPCM_TYPE
};

/**
 * Describes type of predictor
 */
enum ipu_insys_dpcm_predictor {
	IPU_INSYS_DPCM_PREDICTOR_1 = 0,
	IPU_INSYS_DPCM_PREDICTOR_2 = 1,
	N_IPU_INSYS_DPCM_PREDICTOR
};

/** Send queue token flag */
enum ipu_insys_send_queue_token_flag {
	/** Default value for send queue token flag */
	IPU_INSYS_SEND_QUEUE_TOKEN_FLAG_NONE = 0,
	/** Intructs Flush forced command, used for HKR1 INSYS streaming to PSYS in BCSM mode */
	IPU_INSYS_SEND_QUEUE_TOKEN_FLAG_FLUSH_FORCE = 1
};

/** @} */

#endif
