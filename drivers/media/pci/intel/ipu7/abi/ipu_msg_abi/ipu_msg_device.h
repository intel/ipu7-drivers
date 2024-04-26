// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2013 - 2024 Intel Corporation

#ifndef IPU_MSG_DEVICE_H_INCLUDED__
#define IPU_MSG_DEVICE_H_INCLUDED__

/**
 * @addtogroup ipu_msg_abi
 * @{
 */

#include "ia_gofo_common_abi.h"
#include "ia_gofo_msg_header.h"
#include "ipu_msg_state.h"
#include "ipu_msg_err_codes.h"

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
struct ipu_msg_dev_open {
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
struct ipu_msg_dev_open_ack {
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
struct ipu_msg_dev_close {
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
struct ipu_msg_dev_close_ack {
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
enum ipu_msg_err_device {
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
static inline void ipu_msg_device_test_func(void)
{
	CHECK_ALIGN32(struct ia_gofo_version_s);
	CHECK_ALIGN64(struct ipu_msg_dev_open);
	CHECK_ALIGN64(struct ipu_msg_dev_open_ack);
	CHECK_ALIGN64(struct ipu_msg_dev_close);
	CHECK_ALIGN64(struct ipu_msg_dev_close_ack);
}

#endif
