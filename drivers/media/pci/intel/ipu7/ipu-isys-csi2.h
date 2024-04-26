/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2013 - 2024 Intel Corporation */

#ifndef IPU_ISYS_CSI2_H
#define IPU_ISYS_CSI2_H

#include <media/media-entity.h>
#include <media/v4l2-device.h>

#include "ipu-isys-queue.h"
#include "ipu-isys-subdev.h"
#include "ipu-isys-video.h"

struct ipu_isys;
struct ipu_isys_csi2_timing;
struct ipu_isys_csi2_pdata;
struct ipu_isys_stream;
struct v4l2_subdev;

#define NR_OF_CSI2_SINK_PADS		1
#define CSI2_PAD_SINK			0
#define NR_OF_CSI2_SOURCE_PADS		1
#define CSI2_PAD_SOURCE			1
#define NR_OF_CSI2_PADS	(NR_OF_CSI2_SINK_PADS + NR_OF_CSI2_SOURCE_PADS)

#define IPU_SKEW_CAL_LIMIT_HZ (1500000000ul / 2)

#define CSI2_CSI_RX_DLY_CNT_TERMEN_CLANE_A		0
#define CSI2_CSI_RX_DLY_CNT_TERMEN_CLANE_B		0
#define CSI2_CSI_RX_DLY_CNT_SETTLE_CLANE_A		95
#define CSI2_CSI_RX_DLY_CNT_SETTLE_CLANE_B		-8

#define CSI2_CSI_RX_DLY_CNT_TERMEN_DLANE_A		0
#define CSI2_CSI_RX_DLY_CNT_TERMEN_DLANE_B		0
#define CSI2_CSI_RX_DLY_CNT_SETTLE_DLANE_A		85
#define CSI2_CSI_RX_DLY_CNT_SETTLE_DLANE_B		-2

/*
 * struct ipu_isys_csi2
 *
 * @nlanes: number of lanes in the receiver
 */
struct ipu_isys_csi2 {
	struct ipu_isys_csi2_pdata *pdata;
	struct ipu_isys *isys;
	struct ipu_isys_subdev asd;
	struct ipu_isys_video av;

	void __iomem *base;
	u32 receiver_errors;
	u32 legacy_irq_mask;
	unsigned int nlanes;
	unsigned int index;
};

struct ipu_isys_csi2_timing {
	u32 ctermen;
	u32 csettle;
	u32 dtermen;
	u32 dsettle;
};

/*
 * This structure defines the MIPI packet header output
 * from IPU MIPI receiver. Due to hardware conversion,
 * this structure is not the same as defined in CSI-2 spec.
 */
struct ipu_isys_mipi_packet_header {
	u32 word_count:16, dtype:13, sync:2, stype:1;
	u32 sid:4, port_id:4, reserved:23, odd_even:1;
} __packed;

/*
 * This structure defines the trace message content
 * for CSI2 receiver monitor messages.
 */
struct ipu_isys_csi2_monitor_message {
	u64 fe:1,
	    fs:1,
	    pe:1,
	    ps:1,
	    le:1,
	    ls:1,
	    reserved1:2,
	    sequence:2,
	    reserved2:2,
	    flash_shutter:4,
	    error_cause:12,
	    fifo_overrun:1,
	    crc_error:2,
	    reserved3:1,
	    timestamp_l:16,
	    port:4, vc:2, reserved4:2, frame_sync:4, reserved5:4;
	u64 reserved6:3,
	    cmd:2, reserved7:1, monitor_id:7, reserved8:1, timestamp_h:50;
} __packed;

#define to_ipu_isys_csi2(sd) container_of(to_ipu_isys_subdev(sd), \
					struct ipu_isys_csi2, asd)

int ipu_isys_csi2_get_link_freq(struct ipu_isys_csi2 *csi2, __s64 *link_freq);
int ipu_isys_csi2_init(struct ipu_isys_csi2 *csi2,
		       struct ipu_isys *isys,
		       void __iomem *base, unsigned int index);
void ipu_isys_csi2_cleanup(struct ipu_isys_csi2 *csi2);
void ipu_isys_csi2_sof_event(struct ipu_isys_csi2 *csi2);
void ipu_isys_csi2_eof_event(struct ipu_isys_csi2 *csi2);

#endif /* IPU_ISYS_CSI2_H */
