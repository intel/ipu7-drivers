/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (C) 2020 - 2024 Intel Corporation */

#ifndef IPU7_FW_ISYS_ABI_H
#define IPU7_FW_ISYS_ABI_H

#include "ipu7_fw_common_abi.h"
#include "ipu7_fw_isys_abi.h"

/** INYSYS queue defs - general */
/** MSG, LOG, RESERVED output queues */
#define IPU_INSYS_MAX_OUTPUT_QUEUES (3U)
/** Max number of supported virtual streams */
#define IPU_INSYS_STREAM_ID_MAX (16U)
/** DEV + MSG (for each stream) */
#define IPU_INSYS_MAX_INPUT_QUEUES (IPU_INSYS_STREAM_ID_MAX + 1U)
/** INSYS queue defs - output queues */
#define IPU_INSYS_OUTPUT_FIRST_QUEUE	(0U)
#define IPU_INSYS_OUTPUT_LAST_QUEUE	(IPU_INSYS_MAX_OUTPUT_QUEUES - 1U)
/** OUT queues */
#define IPU_INSYS_OUTPUT_MSG_QUEUE		(IPU_INSYS_OUTPUT_FIRST_QUEUE)
#define IPU_INSYS_OUTPUT_LOG_QUEUE		(IPU_INSYS_OUTPUT_FIRST_QUEUE + 1U)
#define IPU_INSYS_OUTPUT_RESERVED_QUEUE	(IPU_INSYS_OUTPUT_LAST_QUEUE)
/** INSYS queue defs - input queues */
#define IPU_INSYS_INPUT_FIRST_QUEUE	(IPU_INSYS_MAX_OUTPUT_QUEUES)
#define IPU_INSYS_INPUT_LAST_QUEUE	(IPU_INSYS_INPUT_FIRST_QUEUE + IPU_INSYS_MAX_INPUT_QUEUES - 1U)
/** IN queues */
#define IPU_INSYS_INPUT_DEV_QUEUE		(IPU_INSYS_INPUT_FIRST_QUEUE)
#define IPU_INSYS_INPUT_MSG_QUEUE		(IPU_INSYS_INPUT_FIRST_QUEUE + 1U)
#define IPU_INSYS_INPUT_MSG_MAX_QUEUE	(IPU_INSYS_MAX_INPUT_QUEUES - 1U)

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
 * ipu7_insys_return_token: data item of exacly 8 bytes (64 bits)
 * which can be used to pass a return token back to the host
*/
typedef uint64_t ipu7_insys_return_token;

/** Response type enumeration for response messages to commands (FW to SW) */
enum ipu7_insys_resp_type {
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
enum ipu7_insys_send_type {
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
enum ipu7_insys_mipi_vc {
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
enum ipu7_insys_mipi_port {
	IPU_INSYS_MIPI_PORT_0 = 0,
	IPU_INSYS_MIPI_PORT_1 = 1,
	IPU_INSYS_MIPI_PORT_2 = 2,
	IPU_INSYS_MIPI_PORT_3 = 3,
	IPU_INSYS_MIPI_PORT_4 = 4,
	IPU_INSYS_MIPI_PORT_5 = 5,
	NA_IPU_INSYS_MIPI_PORT
};

/** Supported Pixel Frame formats. Expandable if needed */
enum ipu7_insys_frame_format_type {
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
enum ipu7_insys_mipi_data_type {
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
enum ipu7_insys_mipi_dt_rename_mode {
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
enum ipu7_insys_output_link_dest {
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
enum ipu7_insys_dpcm_type {
	IPU_INSYS_DPCM_TYPE_DISABLED = 0,
	IPU_INSYS_DPCM_TYPE_10_8_10 = 1,
	IPU_INSYS_DPCM_TYPE_12_8_12 = 2,
	IPU_INSYS_DPCM_TYPE_12_10_12 = 3,
	N_IPU_INSYS_DPCM_TYPE
};

/**
 * Describes type of predictor
 */
enum ipu7_insys_dpcm_predictor {
	IPU_INSYS_DPCM_PREDICTOR_1 = 0,
	IPU_INSYS_DPCM_PREDICTOR_2 = 1,
	N_IPU_INSYS_DPCM_PREDICTOR
};

/** Send queue token flag */
enum ipu7_insys_send_queue_token_flag {
	/** Default value for send queue token flag */
	IPU_INSYS_SEND_QUEUE_TOKEN_FLAG_NONE = 0,
	/** Intructs Flush forced command, used for HKR1 INSYS streaming to PSYS in BCSM mode */
	IPU_INSYS_SEND_QUEUE_TOKEN_FLAG_FLUSH_FORCE = 1
};

#pragma pack(push, 1)

/**
 *  @brief Generic resolution structure
 */
struct ipu7_insys_resolution {
	/** Width */
	uint32_t width;
	/** Height */
	uint32_t height;
};

/**
 *  @brief Output pin payload
 */
struct ipu7_insys_capture_output_pin_payload {
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
struct ipu7_insys_output_link {
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
	/** to which ip the link connected (ddr/psys/...) , see ipu7_insys_output_link_dest */
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
struct ipu7_insys_output_cropping {
	/** When cropping is enabled, all lines before this line will be cropped. */
	uint16_t line_top;
	/** When cropping is enabled, all lines after this line will be cropped. */
	uint16_t line_bottom;
};

/**
 *  @brief Output decompression
 */
struct ipu7_insys_output_dpcm {
	/** 0 - dpcm disabled, 1 enabled */
	uint8_t enable;
	/** type of compression, see ipu7_insys_dpcm_type */
	uint8_t type;
	/** predictor of compression, see ipu7_insys_dpcm_predictor */
	uint8_t predictor;
	/** not used */
	uint8_t pad;
};

/**
 *  @brief Output Pin information and details
 */
struct ipu7_insys_output_pin {
	/** output pin link */
	struct ipu7_insys_output_link link;
	/** output pin crop */
	struct ipu7_insys_output_cropping crop;
	/** output decompression */
	struct ipu7_insys_output_dpcm dpcm;
	/** output stride in Bytes (not valid for statistics) */
	uint32_t stride;
	/** frame format type, see ipu7_insys_frame_format_type */
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
struct ipu7_insys_input_pin {
	/** input resolution */
	struct ipu7_insys_resolution input_res;
	/**
	 *  Sync Event messages configuration map
	 *
	 *  Consists of different event message types.
	 *  Each type can be either configured to send event message response, send an interrupt with event message.
	 *  When only the interrupt option is configured by setting the relevant bit,
	 *  the event message will be sent together with an interrupt.
	 */
	uint16_t sync_msg_map;
	/** mipi data type, see ipu7_insys_mipi_data_type */
	uint8_t dt;
	/**
	 * Disable unpacking of the MIPI packet
	 * store the incoming stream as is to DDR without the header
	 * (the header will be removed)
	 */
	uint8_t disable_mipi_unpacking;
	/** Defines whether MIPI data is encapsulated in some other data type, see ipu7_insys_mipi_dt_rename_mode */
	uint8_t dt_rename_mode;
	/** mapped_dt */
	uint8_t mapped_dt;
	uint8_t pad[2];
};

 /**
  * @brief ISYS stream configuration top level data structure
 */
struct ipu7_insys_stream_cfg {
	/** Input pin descriptors */
	struct ipu7_insys_input_pin input_pins[MAX_IPINS];
	/** Output pin descriptors */
	struct ipu7_insys_output_pin output_pins[MAX_OPINS];
	/**
	 *  Map describing Stream enabled messages Response Send and Interrupt
	 */
	uint16_t stream_msg_map;
	/** Specifies port_id see enum ipu7_insys_mipi_port  */
	uint8_t port_id;
	/** MIPI Virtual Channel (up to 16 virtual per physical channel), see ipu7_insys_mipi_vc */
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
struct ipu7_insys_buffset {
	/** Output pin addresses */
	struct ipu7_insys_capture_output_pin_payload output_pins[MAX_OPINS];
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
struct ipu7_insys_resp {
	/**  */
	uint64_t buf_id;
	/** This var is only valid for pin event related responses, contains pin addresses */
	struct ipu7_insys_capture_output_pin_payload pin;
	/**
	 *  Generic error structure.  err_header.error_code == 0 means ACK,
	 *  all others are an error
	 */
	struct ia_gofo_msg_err error_info;
	/** Time information for an event when available */
	uint32_t timestamp[2];
	/** Response type, see ipu7_insys_resp_type */
	uint8_t type;
	/** Data movement mechanisms between producer and consumer, see ia_gofo_msg_link_streaming_mode */
	uint8_t msg_link_streaming_mode;
	/** Stream ID this response corresponds to */
	uint8_t stream_id;
	/** Pin ID that the pin payload corresponds to */
	uint8_t pin_id;
	/** Valid for STREAM_START_AND_CAPTURE_DONE, STREAM_CAPTURE_DONE and STREAM_CAPTURE_DISCARDED */
	uint8_t frame_id;
	/** Confirmation of intentional frame drop. See ipu7_insys_buffset::skip_frame */
	uint8_t skip_frame;
	/** Padding */
	uint8_t pad[2];
};

/**
 *  @brief Response queue token
 */
struct ipu7_insys_resp_queue_token {
	/** Response information */
	struct ipu7_insys_resp resp_info;
};

/**
 *  @brief Send queue token
 */
struct ipu7_insys_send_queue_token {
	/** Buffer handle  */
	uint64_t buf_handle;
	/** addr */
	ia_gofo_addr_t addr;
	/** Stream ID */
	uint16_t stream_id;
	/** Send type, see ipu7_insys_send_type */
	uint8_t send_type;
	/** Flags provided with the message, see ipu7_insys_send_queue_token_flag */
	uint8_t flag;
};

#pragma pack(pop)

/** @} */

/** Compile time checks only - this function is not to be called! */
static inline void ipu7_insys_types_test_func(void)
{
	CHECK_ALIGN32(struct ipu7_insys_resolution);
	CHECK_ALIGN64(struct ipu7_insys_capture_output_pin_payload);
	CHECK_ALIGN32(struct ipu7_insys_output_pin);
	CHECK_ALIGN32(struct ipu7_insys_input_pin);
	CHECK_ALIGN32(struct ipu7_insys_output_cropping);
	CHECK_ALIGN32(struct ipu7_insys_stream_cfg);
	CHECK_ALIGN64(struct ipu7_insys_buffset);
	CHECK_ALIGN64(struct ipu7_insys_resp);
	CHECK_ALIGN64(struct ipu7_insys_resp_queue_token);
	CHECK_ALIGN64(struct ipu7_insys_send_queue_token);
	CHECK_ALIGN32(struct ipu7_insys_output_link);
}

/* Note that the below enums representing ipu message error are located here temporarily */
/**
 *  Error detail enumeration for error group INSYS_MSG_ERR_GROUP_STREAM used exclusively
 *  in the stream ack messages
 */
enum insys_msg_err_stream {
	/** No error */
	INSYS_MSG_ERR_STREAM_OK = IA_GOFO_MSG_ERR_OK,
	/**
	 * Invalid stream ID.
	 * err_detail[0] is the offending stream_id
	 * err_detail[1] is the max stream_id
	 */
	INSYS_MSG_ERR_STREAM_STREAM_ID = 1,
	/** Stream Output Pins count violation, err_detail[0] is the offending number of output pins */
	INSYS_MSG_ERR_STREAM_MAX_OPINS = 2,
	/** Stream Input Pins count violation, err_detail[0] is the offending number of input pins */
	INSYS_MSG_ERR_STREAM_MAX_IPINS = 3,
	/**
	 * Stream message map violation, err_detail[0] is the offending messages map
	 * Enabling IRQ for a message but disabling response for the same message is not allowed
	 */
	INSYS_MSG_ERR_STREAM_STREAM_MESSAGES_MAP = 4,
	/**
	 * Sync messages map violation, err_detail[0] is the offending messages map
	 * Enabling IRQ for a message but disabling response for the same message is not allowed
	 * err_detail[0] is sync message map
	 * err_detail[1] is pin_id
	 */
	INSYS_MSG_ERR_STREAM_SYNC_MESSAGES_MAP = 5,
	/**
	 * Output pin's sensor type is invalid.
	 * err_detail[0] is pin_id
	 * err_detail[1] is sensor type
	 */
	INSYS_MSG_ERR_STREAM_SENSOR_TYPE = 6,
	/**
	 * foreign key is invalid.
	 * err_detail[0] is the output pin id
	 * err_detail[1] is the offending foreign_key
	 */
	INSYS_MSG_ERR_STREAM_FOREIGN_KEY = 7,
	/**
	 * streaming mode is invalid.
	 * err_detail[0] is the output pin id
	 * err_detail[1] is the streaming mode
	 */
	INSYS_MSG_ERR_STREAM_STREAMING_MODE = 8,
	/**
	 * DPCM enable value is invalid.
	 * err_detail[0] is the output pin id
	 * err_detail[1] is the DPCM enable value
	 */
	INSYS_MSG_ERR_STREAM_DPCM_EN = 9,
	/**
	 * DPCM type value is invalid.
	 * err_detail[0] is the output pin id
	 * err_detail[1] is the DPCM type value
	 */
	INSYS_MSG_ERR_STREAM_DPCM_TYPE = 10,
	/**
	 * DPCM predictor value is invalid.
	 * err_detail[0] is the output pin id
	 * err_detail[1] is the DPCM predictor value
	 */
	INSYS_MSG_ERR_STREAM_DPCM_PREDICTOR = 11,
	/**
	 * Granularity pointer update is invalid.
	 * err_detail[0] is the output pin id
	 * err_detail[1] is the granularity pointer update
	 */
	INSYS_MSG_ERR_STREAM_GRANULARITY_POINTER_UPDATE = 12,
	/** MPF Entry LUT entry resources are all busy, err_detail[0] is MPF device id */
	INSYS_MSG_ERR_STREAM_MPF_LUT_ENTRY_RESOURCES_BUSY = 13,
	/** MPF Device ID being used is invalid, err_detail[0] is the offending mpf device id */
	INSYS_MSG_ERR_STREAM_MPF_DEV_ID = 14,
	/**
	 * Buffer lines is invalid
	 * err_detail[0] is the output pin id
	 * err_detail[1] is the offending buffer lines value
	 */
	INSYS_MSG_ERR_STREAM_BUFFER_LINES = 15,
	/** Failure matching an output pin to input pin, err_detail[0] is the offending input pin id */
	INSYS_MSG_ERR_STREAM_IPIN_ID = 16,
	/**
	 * Data type is invalid
	 * err_detail[0] is the output pin id
	 * err_detail[1] is the offending data type
	 */
	INSYS_MSG_ERR_STREAM_DATA_TYPE = 17,
	/**
	 * Invalid stream streaming protocol state, can be internal fw error related to streaming protocol
	 * err_detail[0] is the foreign key
	 * err_detail[1] is the streaming mode
	 */
	INSYS_MSG_ERR_STREAM_STREAMING_PROTOCOL_STATE = 18,
	/**
	 * Failure flushing Syscom IN queue
	 * err_detail[0] is the stream id
	 * err_detail[1] is the queue id
	 */
	INSYS_MSG_ERR_STREAM_SYSCOM_FLUSH = 19,
	/** Stream MIPI VC violation, err_detail[0] is the offending value for MIPI VC */
	INSYS_MSG_ERR_STREAM_MIPI_VC = 20,
	/** Stream source violation, err_detail[0] is the offending value for src-stream source */
	INSYS_MSG_ERR_STREAM_STREAM_SRC = 21,
	/**
	 * pbk_id is invalid
	 * err_detail[0] is the output pin id
	 * err_detail[1] is the violating pbk_id
	 */
	INSYS_MSG_ERR_STREAM_PBK_ID = 22,
	/** Command queue deallocation failure, err_detail[0] is the stream id */
	INSYS_MSG_ERR_STREAM_CMD_QUEUE_DEALLOCATE = 23,
	/**
	 * There are no free resources to acquire from.
	 * err_detail[0] is the stream id
	 */
	INSYS_MSG_ERR_STREAM_INSUFFICIENT_RESOURCES = 24,
	/** Failure acquiring resources based on provided input pin configuration */
	INSYS_MSG_ERR_STREAM_IPIN_CONFIGURATION = 25,
	/**
	 * The state of the stream is not valid
	 * err_detail[0] is the stream id
	 * err_detail[1] is the state
	 */
	INSYS_MSG_ERR_STREAM_INVALID_STATE = 26,
	/**
	 * sw managed is invalid
	 * err_detail[0] is the output pin id
	 * err_detail[1] is the offending SW managed value
	 */
	INSYS_MSG_ERR_STREAM_SW_MANAGED = 27,
	/**
	 * pbk_slot_id is invalid
	 * err_detail[0] is the output pin id
	 * err_detail[1] is the violating pbk_slot_id
	 */
	INSYS_MSG_ERR_STREAM_PBK_SLOT_ID = 28,
	/**
	 * hw flush command is timedout
	 * err_detail[0] is the stream id
	 */
	INSYS_MSG_ERR_STREAM_FLUSH_TIMEOUT = 29,
	/**
	 * inpur_pin width is invalid
	 * err_detail[0] is the violating width
	 * err_detail[1] is the input pin id
	 */
	INSYS_MSG_ERR_STREAM_IPIN_WIDTH = 30,
	/**
	 * inpur_pin height is invalid
	 * err_detail[0] is the violating height
	 * err_detail[1] is the input pin id
	 */
	INSYS_MSG_ERR_STREAM_IPIN_HEIGHT = 31,
	/**
	 * Output pin early ack indication configuration error
	 * Configuring all output pins to an early ack results in violation.
	 * Configuring an early ack for a stream with single output pin results in violation.
	 * err_detail[0] is the output pin number
	 * err_detail[1] is the violating early ack configuration
	 */
	INSYS_MSG_ERR_STREAM_OUTPUT_PIN_EARLY_ACK_EN = 32,
	/**
	 * Inconsistency detected. ((width * bits_per_pixel_by_packing) / 8) must be <= stride
	 * err_detail[0] is ((width * bits_per_pixel_by_packing) / 8)
	 * err_detail[1] is the stride
	 */
	INSYS_MSG_ERR_STREAM_INCONSISTENT_PARAMS = 33,
	/**
	 * Unsupported plane count. Currently, only single plane is supported
	 * err_detail[0] is the invalid plane count
	 */
	INSYS_MSG_ERR_STREAM_PLANE_COUNT = 34,
	/**
	 * Frame format type is invalid
	 * err_detail[0] is the output pin id
	 * err_detail[1] is the offending frame format type
	 */
	INSYS_MSG_ERR_STREAM_FRAME_FORMAT_TYPE = 35,
	/** number of ipu message errors for stream type */
	INSYS_MSG_ERR_STREAM_N
};

/**
 *  Error detail enumeration for error group INSYS_MSG_ERR_GROUP_CAPTURE used exclusively
 *  in the capture ack messages
 */
enum insys_msg_err_capture {
	/** No error */
	INSYS_MSG_ERR_CAPTURE_OK = IA_GOFO_MSG_ERR_OK,
	/**
	 * Invalid stream ID.
	 * err_detail[0] is the offending stream_id
	 * err_detail[1] is the max stream_id
	 */
	INSYS_MSG_ERR_CAPTURE_STREAM_ID = 1,
	/**
	 * Invalid frame payload ptr.
	 * Received NULL where a non-NULL value is required or other illegal value.
	 * err_detail[0] is the offending offending pointer value
	 * err_detail[1] is the stream id
	 */
	INSYS_MSG_ERR_CAPTURE_PAYLOAD_PTR = 2,
	/** No memory slot left for another capture command, err_detail[0] is the stream id */
	INSYS_MSG_ERR_CAPTURE_MEM_SLOT = 3,
	/**
	 * Invalid streaming mode
	 * err_detail[0] is the offending streaming mode
	 * err_detail[1] is the output pin
	 */
	INSYS_MSG_ERR_CAPTURE_STREAMING_MODE = 4,
	/**
	 * There was no free available slot for capture command.
	 * err_detail[0] is the stream id
	 */
	INSYS_MSG_ERR_CAPTURE_AVAILABLE_CMD_SLOT = 5,
	/**
	 * An error consuming command
	 * err_detail[0] is the stream id
	 * err_detail[1] is the output pin id
	 */
	INSYS_MSG_ERR_CAPTURE_CONSUMED_CMD_SLOT = 6,
	/**
	 * Command slot payload pointer is null
	 * err_detail[0] is the stream id
	 */
	INSYS_MSG_ERR_CAPTURE_CMD_SLOT_PAYLOAD_PTR = 7,
	/**
	 * Error occurs during preparation of command before submitting to hw
	 * err_detail[0] is the stream id
	 */
	INSYS_MSG_ERR_CAPTURE_CMD_PREPARE = 8,
	/**
	 * Invalid output pin id
	 * err_detail[0] is the offending output pin id
	 * err_detail[1] is the stream id
	 */
	INSYS_MSG_ERR_CAPTURE_OUTPUT_PIN = 9,
	/**
	 * Received a small packet, without command, dropping the frame.
	 * err_detail[0] is the stream id
	 * err_detail[1] is the frame id to be dropped
	 */
	INSYS_MSG_ERR_CAPTURE_SYNC_FRAME_DROP = 10,
	/**
	 * Frame messages map violation, err_detail[0] is the offending messages map.
	 * Enabling IRQ for a message while disabling response for the same message is not allowed.
	 */
	INSYS_MSG_ERR_CAPTURE_FRAME_MESSAGES_MAP = 11,
	/**
	 * Timeout on this capture request
	 * err_detail[0] is the stream id
	 * err_detail[1] is the output pin
	 */
	INSYS_MSG_ERR_CAPTURE_TIMEOUT = 12,
	/**
	 * The state of the stream is not valid
	 * err_detail[0] is the stream id
	 * err_detail[1] is the state
	 */
	INSYS_MSG_ERR_CAPTURE_INVALID_STREAM_STATE = 13,
	/**
	 * Multiple packet header errors
	 * err_detail[0] is output pin
	 */
	INSYS_MSG_ERR_CAPTURE_HW_ERR_MULTIBIT_PH_ERROR_DETECTED = 14,
	/**
	 * Payload checksum (CRC) error
	 * err_detail[0] is output pin
	 */
	INSYS_MSG_ERR_CAPTURE_HW_ERR_PAYLOAD_CRC_ERROR = 15,
	/**
	 * Input data loss happening before CSI-FE due to elastic FIFO overflow in CSI2PPI
	 * err_detail[0] is output pin
	 */
	INSYS_MSG_ERR_CAPTURE_HW_ERR_INPUT_DATA_LOSS_ELASTIC_FIFO_OVFL  = 16,
	/**
	 * Pixel Buffer overflowing and GDA dropping writes.
	 * Resulting in losing frame data due to system backpressure or programming errors.
	 * err_detail[0] is output pin
	 */
	INSYS_MSG_ERR_CAPTURE_HW_ERR_PIXEL_BUFFER_OVERFLOW = 17,
	/**
	 * The incoming frame dimensions, incoming from the sensor, do not match the expected frame dimension values.
	 * err_detail[0] is output pin
	 */
	INSYS_MSG_ERR_CAPTURE_HW_ERR_BAD_FRAME_DIM = 18,
	/**
	 * Missing short synchronization packets (FS/FE). FS after FS/FE after FE.
	 * err_detail[0] is output pin
	 */
	INSYS_MSG_ERR_CAPTURE_HW_ERR_PHY_SYNC_ERR = 19,
	/**
	 * Frame was partially masked with secure touch enabled.
	 * err_detail[0] is output pin
	 */
	INSYS_MSG_ERR_CAPTURE_HW_ERR_SECURE_TOUCH = 20,
	/**
	 * A master slave IID capture error occurred.
	 * Happens when a slave IID doesn’t receive FS during capture window.
	 * err_detail[0] is output pin
	 */
	INSYS_MSG_ERR_CAPTURE_HW_ERR_MASTER_SLAVE_SYNC_ERR = 21,
	/**
	 * Frame skip requested but at least one of the output pin addresses is not null
	 * err_detail[0] is the offending output address
	 * err_detail[1] is the problem output pin
	 */
	INSYS_MSG_ERR_CAPTURE_FRAME_SKIP_ERR = 22,
	/**
	 * FE input FIFO overflow
	 * HW Can't keep up with the incoming rate
	 * err_detail[0] is output pin
	 */
	INSYS_MSG_ERR_CAPTURE_FE_INPUT_FIFO_OVERFLOW_ERR = 23,
	/**
	 * FW failed submitting the buffer-set to HW
	 * err_detail[0] is the stream id
	 * err_detail[1] is output pin
	 */
	INSYS_MSG_ERR_CAPTURE_CMD_SUBMIT_TO_HW = 24,
	/** number of ipu message errors for capture type */
	INSYS_MSG_ERR_CAPTURE_N
};

/**
 *  Error groups define broad categories of errors in ack messages.
 *  This enumeration defines the error groups for INSYS.
 */
enum insys_msg_err_groups {
	/** Reserved value.  Used only with IA_GOFO_MSG_ERR_OK & IA_GOFO_MSG_ERR_UNSPECIFED */
	INSYS_MSG_ERR_GROUP_RESERVED = IA_GOFO_MSG_ERR_GROUP_RESERVED,
	/**
	 *  Generic message general errors that aren't specific to another group.  See ia_gofo_msg_err_general for
	 *  individual errors.
	 */
	INSYS_MSG_ERR_GROUP_GENERAL = IA_GOFO_MSG_ERR_GROUP_GENERAL,
	/** Stream open/start/flush/abort/stop messages errors.  See insys_msg_err_stream for individual errors. */
	INSYS_MSG_ERR_GROUP_STREAM = 2,
	/** Capture messages errors.  See insys_msg_err_capture for individual errors. */
	INSYS_MSG_ERR_GROUP_CAPTURE = 3,

	/**
	 *  Number of items in this enumeration.
	 */
	INSYS_MSG_ERR_GROUP_N,
};

#endif