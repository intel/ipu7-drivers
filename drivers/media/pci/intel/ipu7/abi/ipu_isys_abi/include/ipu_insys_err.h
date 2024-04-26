// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2013 - 2024 Intel Corporation

#ifndef IPU_INSYS_ERR_H_INCLUDED__
#define IPU_INSYS_ERR_H_INCLUDED__

#include "insys_msg_err_codes.h"
#include "ia_gofo_msg_err.h"

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

#endif
