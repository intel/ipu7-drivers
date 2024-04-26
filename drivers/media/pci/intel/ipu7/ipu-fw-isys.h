/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2013 - 2024 Intel Corporation */

#ifndef IPU_FW_ISYS_H
#define IPU_FW_ISYS_H

#include "ipu-syscom.h"
#include "ia_gofo_msg_err.h"
#include "ia_gofo_msg_log.h"
#include "ipu_msg_link.h"
#include "ipu_insys_abi.h"
#include "ipu_insys_types.h"
#include "ipu_insys_err.h"

#define IPU7_STREAM_ID_MAX 16
#define IPU7_NONSECURE_STREAM_ID_MAX 12
#define IPU7_DEV_SEND_QUEUE_SIZE (IPU7_STREAM_ID_MAX)
#define IPU7_NOF_SRAM_BLOCKS_MAX (IPU7_STREAM_ID_MAX)
#define IPU7_N_MAX_MSG_SEND_QUEUES (IPU7_STREAM_ID_MAX)

/* Max number of planes for frame formats supported by the FW */
#define IPU_PIN_PLANES_MAX 4

/**
 * enum ipu_fw_isys_stream_source: Specifies a source for a stream
 */
enum ipu_fw_isys_stream_source {
	IPU_FW_ISYS_STREAM_SRC_PORT_0 = 0,
	IPU_FW_ISYS_STREAM_SRC_PORT_1,
	IPU_FW_ISYS_STREAM_SRC_PORT_2,
	IPU_FW_ISYS_STREAM_SRC_PORT_3,
	IPU_FW_ISYS_STREAM_SRC_PORT_4,
	IPU_FW_ISYS_STREAM_SRC_PORT_5,
	IPU_FW_ISYS_STREAM_SRC_PORT_6,
	IPU_FW_ISYS_STREAM_SRC_PORT_7,
	IPU_FW_ISYS_STREAM_SRC_PORT_8,
	IPU_FW_ISYS_STREAM_SRC_PORT_9,
	IPU_FW_ISYS_STREAM_SRC_PORT_10,
	IPU_FW_ISYS_STREAM_SRC_PORT_11,
	IPU_FW_ISYS_STREAM_SRC_PORT_12,
	IPU_FW_ISYS_STREAM_SRC_PORT_13,
	IPU_FW_ISYS_STREAM_SRC_PORT_14,
	IPU_FW_ISYS_STREAM_SRC_PORT_15,
	IPU_FW_ISYS_STREAM_SRC_MIPIGEN_0,
	IPU_FW_ISYS_STREAM_SRC_MIPIGEN_1,
	IPU_FW_ISYS_STREAM_SRC_MIPIGEN_2,
	IPU_FW_ISYS_STREAM_SRC_MIPIGEN_3,
	IPU_FW_ISYS_STREAM_SRC_MIPIGEN_4,
	IPU_FW_ISYS_STREAM_SRC_MIPIGEN_5,
	IPU_FW_ISYS_STREAM_SRC_MIPIGEN_6,
	IPU_FW_ISYS_STREAM_SRC_MIPIGEN_7,
	IPU_FW_ISYS_STREAM_SRC_MIPIGEN_8,
	IPU_FW_ISYS_STREAM_SRC_MIPIGEN_9,
	N_IPU_FW_ISYS_STREAM_SRC
};

enum ipu_fw_isys_sensor_info {
	/* VC1 */
	IPU_FW_ISYS_SENSOR_DATA_1 = 1,
	IPU_FW_ISYS_SENSOR_DATA_2 = 2,
	IPU_FW_ISYS_SENSOR_DATA_3 = 3,
	IPU_FW_ISYS_SENSOR_DATA_4 = 4,
	IPU_FW_ISYS_SENSOR_DATA_5 = 5,
	IPU_FW_ISYS_SENSOR_DATA_6 = 6,
	IPU_FW_ISYS_SENSOR_DATA_7 = 7,
	IPU_FW_ISYS_SENSOR_DATA_8 = 8,
	IPU_FW_ISYS_SENSOR_DATA_9 = 9,
	IPU_FW_ISYS_SENSOR_DATA_10 = 10,
	IPU_FW_ISYS_SENSOR_PDAF_1 = 11,
	IPU_FW_ISYS_SENSOR_PDAF_2 = 12,
	/* VC0 */
	IPU_FW_ISYS_SENSOR_METADATA = 13,
	IPU_FW_ISYS_SENSOR_DATA_11 = 14,
	IPU_FW_ISYS_SENSOR_DATA_12 = 15,
	IPU_FW_ISYS_SENSOR_DATA_13 = 16,
	IPU_FW_ISYS_SENSOR_DATA_14 = 17,
	IPU_FW_ISYS_SENSOR_DATA_15 = 18,
	IPU_FW_ISYS_SENSOR_DATA_16 = 19,
	N_IPU_FW_ISYS_SENSOR_INFO,
	IPU_FW_ISYS_VC1_SENSOR_DATA_START = IPU_FW_ISYS_SENSOR_DATA_1,
	IPU_FW_ISYS_VC1_SENSOR_DATA_END = IPU_FW_ISYS_SENSOR_DATA_10,
	IPU_FW_ISYS_VC0_SENSOR_DATA_START = IPU_FW_ISYS_SENSOR_DATA_11,
	IPU_FW_ISYS_VC0_SENSOR_DATA_END = IPU_FW_ISYS_SENSOR_DATA_16,
	IPU_FW_ISYS_VC1_SENSOR_PDAF_START = IPU_FW_ISYS_SENSOR_PDAF_1,
	IPU_FW_ISYS_VC1_SENSOR_PDAF_END = IPU_FW_ISYS_SENSOR_PDAF_2,
};

struct ipu_isys;

/* From here on type defines not coming from the ISYSAPI interface */

#ifndef UINT8_MAX
#define UINT8_MAX       (0xffUL)
#endif

#ifndef UINT16_MAX
#define UINT16_MAX       (0xffffUL)
#endif

/*
 * Max number of supported SRAM buffer partitions.
 * It refers to the size of stream partitions.
 * These partitions are further subpartitioned internally
 * by the FW, but by declaring statically the stream
 * partitions we solve the buffer fragmentation issue
 */
#define IPU_NOF_SRAM_BLOCKS_MAX (IPU_INSYS_STREAM_ID_MAX)

enum ipu_isys_sensor_type {
	/* non-snoopable to PSYS */
	IPU_ISYS_VC1_SENSOR_DATA = 0,
	/* non-snoopable for PDAF */
	IPU_ISYS_VC1_SENSOR_PDAF,
	/* snoopable to CPU */
	IPU_ISYS_VC0_SENSOR_METADATA,
	/* snoopable to CPU */
	IPU_ISYS_VC0_SENSOR_DATA,
	N_IPU_ISYS_SENSOR_TYPE
};

/* msg type to string for debug purpose */
struct resp_to_msg {
	enum ipu_insys_resp_type type;
	const char *msg;
};

static const struct resp_to_msg is_fw_msg[] = {
	{IPU_INSYS_RESP_TYPE_STREAM_OPEN_DONE,
	"IPU_INSYS_RESP_TYPE_STREAM_OPEN_DONE"},
	{IPU_INSYS_RESP_TYPE_STREAM_START_AND_CAPTURE_ACK,
	"IPU_INSYS_RESP_TYPE_STREAM_START_AND_CAPTURE_ACK"},
	{IPU_INSYS_RESP_TYPE_STREAM_CAPTURE_ACK,
	"IPU_INSYS_RESP_TYPE_STREAM_CAPTURE_ACK"},
	{IPU_INSYS_RESP_TYPE_STREAM_ABORT_ACK,
	"IPU_INSYS_RESP_TYPE_STREAM_ABORT_ACK"},
	{IPU_INSYS_RESP_TYPE_STREAM_FLUSH_ACK,
	"IPU_INSYS_RESP_TYPE_STREAM_FLUSH_ACK"},
	{IPU_INSYS_RESP_TYPE_STREAM_CLOSE_ACK,
	"IPU_INSYS_RESP_TYPE_STREAM_CLOSE_ACK"},
	{IPU_INSYS_RESP_TYPE_PIN_DATA_READY,
	"IPU_INSYS_RESP_TYPE_PIN_DATA_READY"},
	{IPU_INSYS_RESP_TYPE_FRAME_SOF, "IPU_INSYS_RESP_TYPE_FRAME_SOF"},
	{IPU_INSYS_RESP_TYPE_FRAME_EOF, "IPU_INSYS_RESP_TYPE_FRAME_EOF"},
	{IPU_INSYS_RESP_TYPE_STREAM_START_AND_CAPTURE_DONE,
	"IPU_INSYS_RESP_TYPE_STREAM_START_AND_CAPTURE_DONE"},
	{IPU_INSYS_RESP_TYPE_STREAM_CAPTURE_DONE,
	"IPU_INSYS_RESP_TYPE_STREAM_CAPTURE_DONE"},
	{N_IPU_INSYS_RESP_TYPE, "N_IPU_INSYS_RESP_TYPE"},
};

/**
 * struct ipu_isys_resolution
 *
 * Generic resolution structure.
 */
struct ipu_isys_resolution {
	/** Width */
	u32 width;
	/** Height */
	u32 height;
};

int ipu_fw_isys_init(struct ipu_isys *isys);
void ipu_fw_isys_release(struct ipu_isys *isys);
int ipu_fw_isys_open(struct ipu_isys *isys);
int ipu_fw_isys_close(struct ipu_isys *isys);

void ipu_fw_isys_dump_stream_cfg(struct device *dev,
				 struct ipu_insys_stream_cfg *stream_cfg);
void ipu_fw_isys_dump_frame_buff_set(struct device *dev,
				     struct ipu_insys_buffset *buf,
				     unsigned int outputs);
int ipu_fw_isys_simple_cmd(struct ipu_isys *isys,
			   const unsigned int stream_handle,
			   enum ipu_insys_send_type send_type);
int ipu_fw_isys_complex_cmd(struct ipu_isys *isys,
			    const unsigned int stream_handle,
			    void *cpu_mapped_buf,
			    dma_addr_t dma_mapped_buf,
			    size_t size, enum ipu_insys_send_type send_type);
struct ipu_insys_resp *ipu_fw_isys_get_resp(struct ipu_isys *isys,
					    struct ipu_insys_resp *response);
int ipu_fw_isys_get_log(struct ipu_isys *isys);
void ipu_fw_isys_put_resp(struct ipu_syscom_context *syscom,
			  unsigned int queue);
#endif
