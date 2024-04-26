/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2013 - 2024 Intel Corporation */

#ifndef IPU_ISYS_H
#define IPU_ISYS_H

#include <linux/pm_qos.h>
#include <linux/spinlock.h>

#include <media/v4l2-device.h>
#include <media/media-device.h>

#include "ipu.h"
#include "ipu-isys-media.h"
#include "ipu-isys-csi2.h"
#include "ipu-isys-csi2-be.h"
#ifdef CONFIG_VIDEO_INTEL_IPU_MGC
#include "ipu-isys-tpg.h"
#endif
#include "ipu-isys-video.h"
#include "ipu-fw-isys.h"

#define IPU_ISYS_ENTITY_PREFIX		"Intel IPU7"

/*
 * FW support max 8 streams
 */
#define IPU_ISYS_MAX_STREAMS		8

#define IPU_ISYS_2600_MEM_LINE_ALIGN	64

/* for TPG */
#define IPU_ISYS_FREQ		533000000UL

/*
 * Current message queue configuration. These must be big enough
 * so that they never gets full. Queues are located in system memory
 */
#define IPU_ISYS_SIZE_RECV_QUEUE 40
#define IPU_ISYS_SIZE_LOG_QUEUE 256
#define IPU_ISYS_SIZE_SEND_QUEUE 40
#define IPU_ISYS_NUM_RECV_QUEUE 1

/*
 * Device close takes some time from last ack message to actual stopping
 * of the SP processor. As long as the SP processor runs we can't proceed with
 * clean up of resources.
 */
#define IPU_ISYS_OPEN_TIMEOUT_US		1000
#define IPU_ISYS_OPEN_RETRY		1000
#define IPU_ISYS_TURNOFF_DELAY_US		1000
#define IPU_ISYS_TURNOFF_TIMEOUT		1000
#define IPU_LIB_CALL_TIMEOUT_JIFFIES \
	msecs_to_jiffies(IPU_LIB_CALL_TIMEOUT_MS)

#define IPU_ISYS_MIN_WIDTH		2U
#define IPU_ISYS_MIN_HEIGHT		2U
#define IPU_ISYS_MAX_WIDTH		8160U
#define IPU_ISYS_MAX_HEIGHT		8190U

#define NR_OF_CSI2_BE_SOC_DEV		8U

struct task_struct;

struct isys_fw_log {
	struct mutex mutex; /* protect whole struct */
	void *head;
	void *addr;
	u32 count; /* running counter of log */
	u32 size; /* actual size of log content, in bits */
};

struct ipu_isys_sensor_info {
	unsigned int vc1_data_start;
	unsigned int vc1_data_end;
	unsigned int vc0_data_start;
	unsigned int vc0_data_end;
	unsigned int vc1_pdaf_start;
	unsigned int vc1_pdaf_end;
	unsigned int sensor_metadata;
};

/*
 * struct ipu_isys
 *
 * @media_dev: Media device
 * @v4l2_dev: V4L2 device
 * @adev: ISYS bus device
 * @power: Is ISYS powered on or not?
 * @isr_bits: Which bits does the ISR handle?
 * @power_lock: Serialise access to power (power state in general)
 * @csi2_rx_ctrl_cached: cached shared value between all CSI2 receivers
 * @streams_lock: serialise access to streams
 * @streams: streams per firmware stream ID
 * @syscom: fw communication layer context
 * @line_align: line alignment in memory
 * @reset_needed: Isys requires d0i0->i3 transition
 * @video_opened: total number of opened file handles on video nodes
 * @mutex: serialise access isys video open/release related operations
 * @stream_mutex: serialise stream start and stop, queueing requests
 * @pdata: platform data pointer
 * @csi2: CSI-2 receivers
#ifdef CONFIG_VIDEO_INTEL_IPU_MGC
 * @tpg: test pattern generators
#endif
 */
struct ipu_isys {
	struct media_device media_dev;
	struct v4l2_device v4l2_dev;
	struct ipu_bus_device *adev;

	int power;
	spinlock_t power_lock;	/* Serialise access to power */
	u32 isr_csi2_mask;
	u32 csi2_rx_ctrl_cached;
	spinlock_t streams_lock;
	struct ipu_isys_stream streams[IPU_ISYS_MAX_STREAMS];
	int streams_ref_count[IPU_ISYS_MAX_STREAMS];
	unsigned int line_align;
	u32 phy_rext_cal;
	bool reset_needed;
	bool icache_prefetch;
	bool csi2_cse_ipc_not_supported;
	unsigned int video_opened;
	unsigned int stream_opened;
	struct ipu_isys_sensor_info sensor_info;
	unsigned int sensor_types[N_IPU_ISYS_SENSOR_TYPE];

#ifdef CONFIG_DEBUG_FS
	struct dentry *debugfsdir;
#endif
	struct mutex mutex;	/* Serialise isys video open/release related */
	struct mutex stream_mutex;	/* Stream start, stop, queueing reqs */

	struct ipu_isys_pdata *pdata;

	struct ipu_isys_csi2 *csi2;
#ifdef CONFIG_VIDEO_INTEL_IPU_MGC
	struct ipu_isys_tpg *tpg;
#endif
	struct ipu_isys_csi2_be_soc csi2_be_soc[NR_OF_CSI2_BE_SOC_DEV];
	struct isys_fw_log *fw_log;

	struct list_head requests;
	struct pm_qos_request pm_qos;
	spinlock_t listlock;	/* Protect framebuflist */
	struct list_head framebuflist;
	struct list_head framebuflist_fw;
	struct v4l2_async_notifier notifier;

	struct ipu_insys_config *subsys_config;
	dma_addr_t subsys_config_dma_addr;
};

struct isys_fw_msgs {
	union {
		u64 dummy;
		struct ipu_insys_buffset frame;
		struct ipu_insys_stream_cfg stream;
	} fw_msg;
	struct list_head head;
	dma_addr_t dma_addr;
};

#define to_frame_msg_buf(a) (&(a)->fw_msg.frame)
#define to_stream_cfg_msg_buf(a) (&(a)->fw_msg.stream)
#define to_dma_addr(a) ((a)->dma_addr)

struct isys_fw_msgs *ipu_get_fw_msg_buf(struct ipu_isys_stream *stream);
void ipu_put_fw_msg_buf(struct ipu_isys *isys, u64 data);
void ipu_cleanup_fw_msg_bufs(struct ipu_isys *isys);

extern const struct v4l2_ioctl_ops ipu_isys_ioctl_ops;

int isys_isr_one(struct ipu_bus_device *adev);
irqreturn_t isys_isr(struct ipu_bus_device *adev);

#endif /* IPU_ISYS_H */
