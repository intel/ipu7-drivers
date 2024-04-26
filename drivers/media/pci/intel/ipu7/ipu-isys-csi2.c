// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2013 - 2024 Intel Corporation

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/version.h>

#include <media/ipu-isys.h>
#include <media/media-entity.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>

#include "ipu.h"
#include "ipu-bus.h"
#include "ipu-buttress.h"
#include "ipu-isys.h"
#include "ipu-isys-csi2.h"
#include "ipu-isys-subdev.h"
#include "ipu-isys-video.h"
#include "ipu-platform-regs.h"
#include "ipu-isys-csi2-regs.h"
#include "ipu7-isys-csi-phy.h"

static const u32 csi2_supported_codes_pad_sink[] = {
	MEDIA_BUS_FMT_Y10_1X10,
	MEDIA_BUS_FMT_RGB565_1X16,
	MEDIA_BUS_FMT_RGB888_1X24,
	MEDIA_BUS_FMT_UYVY8_1X16,
	MEDIA_BUS_FMT_YUYV8_1X16,
	MEDIA_BUS_FMT_YUYV10_1X20,
	MEDIA_BUS_FMT_SBGGR10_1X10,
	MEDIA_BUS_FMT_SGBRG10_1X10,
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SBGGR10_DPCM8_1X8,
	MEDIA_BUS_FMT_SGBRG10_DPCM8_1X8,
	MEDIA_BUS_FMT_SGRBG10_DPCM8_1X8,
	MEDIA_BUS_FMT_SRGGB10_DPCM8_1X8,
	MEDIA_BUS_FMT_SBGGR12_1X12,
	MEDIA_BUS_FMT_SGBRG12_1X12,
	MEDIA_BUS_FMT_SGRBG12_1X12,
	MEDIA_BUS_FMT_SRGGB12_1X12,
	MEDIA_BUS_FMT_SBGGR8_1X8,
	MEDIA_BUS_FMT_SGBRG8_1X8,
	MEDIA_BUS_FMT_SGRBG8_1X8,
	MEDIA_BUS_FMT_SRGGB8_1X8,
	0,
};

static const u32 csi2_supported_codes_pad_source[] = {
	MEDIA_BUS_FMT_Y10_1X10,
	MEDIA_BUS_FMT_RGB565_1X16,
	MEDIA_BUS_FMT_RGB888_1X24,
	MEDIA_BUS_FMT_UYVY8_1X16,
	MEDIA_BUS_FMT_YUYV8_1X16,
	MEDIA_BUS_FMT_YUYV10_1X20,
	MEDIA_BUS_FMT_SBGGR10_1X10,
	MEDIA_BUS_FMT_SGBRG10_1X10,
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SBGGR12_1X12,
	MEDIA_BUS_FMT_SGBRG12_1X12,
	MEDIA_BUS_FMT_SGRBG12_1X12,
	MEDIA_BUS_FMT_SRGGB12_1X12,
	MEDIA_BUS_FMT_SBGGR8_1X8,
	MEDIA_BUS_FMT_SGBRG8_1X8,
	MEDIA_BUS_FMT_SGRBG8_1X8,
	MEDIA_BUS_FMT_SRGGB8_1X8,
	0,
};

static const u32 *csi2_supported_codes[NR_OF_CSI2_PADS];

static const struct v4l2_subdev_internal_ops csi2_sd_internal_ops = {
	.open = ipu_isys_subdev_open,
};

int ipu_isys_csi2_get_link_freq(struct ipu_isys_csi2 *csi2, __s64 *link_freq)
{
	struct ipu_isys_stream *stream =
		container_of(media_entity_pipeline(&csi2->asd.sd.entity),
			     struct ipu_isys_stream, pipe);
	struct v4l2_subdev *ext_sd =
	    media_entity_to_v4l2_subdev(stream->external->entity);
	struct v4l2_ext_control c = {.id = V4L2_CID_LINK_FREQ, };
	struct v4l2_ext_controls cs = {.count = 1,
		.controls = &c,
	};
	struct v4l2_querymenu qm = {.id = c.id, };
	int ret;

	if (!ext_sd) {
		WARN_ON(1);
		return -ENODEV;
	}
	ret = v4l2_g_ext_ctrls(ext_sd->ctrl_handler, ext_sd->devnode,
			       ext_sd->v4l2_dev->mdev, &cs);
	if (ret) {
		dev_info(&csi2->isys->adev->dev, "can't get link frequency\n");
		return ret;
	}

	qm.index = c.value;

	ret = v4l2_querymenu(ext_sd->ctrl_handler, &qm);
	if (ret) {
		dev_info(&csi2->isys->adev->dev, "can't get menu item\n");
		return ret;
	}

	dev_dbg(&csi2->isys->adev->dev, "link frequency %lld\n", qm.value);

	if (!qm.value)
		return -EINVAL;
	*link_freq = qm.value;
	return 0;
}

static int subscribe_event(struct v4l2_subdev *sd, struct v4l2_fh *fh,
			   struct v4l2_event_subscription *sub)
{
	struct ipu_isys_csi2 *csi2 = to_ipu_isys_csi2(sd);

	dev_dbg(&csi2->isys->adev->dev, "subscribe event(type %u id %u)\n",
		sub->type, sub->id);

	switch (sub->type) {
	case V4L2_EVENT_FRAME_SYNC:
		return v4l2_event_subscribe(fh, sub, 10, NULL);
	case V4L2_EVENT_CTRL:
		return v4l2_ctrl_subscribe_event(fh, sub);
	default:
		return -EINVAL;
	}
}

static const struct v4l2_subdev_core_ops csi2_sd_core_ops = {
	.subscribe_event = subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

/*
 * The input system CSI2+ receiver has several
 * parameters affecting the receiver timings. These depend
 * on the MIPI bus frequency F in Hz (sensor transmitter rate)
 * as follows:
 *	register value = (A/1e9 + B * UI) / COUNT_ACC
 * where
 *	UI = 1 / (2 * F) in seconds
 *	COUNT_ACC = counter accuracy in seconds
 *	For legacy IPU,  COUNT_ACC = 0.125 ns
 *
 * A and B are coefficients from the table below,
 * depending whether the register minimum or maximum value is
 * calculated.
 *				       Minimum     Maximum
 * Clock lane			       A     B     A     B
 * reg_rx_csi_dly_cnt_termen_clane     0     0    38     0
 * reg_rx_csi_dly_cnt_settle_clane    95    -8   300   -16
 * Data lanes
 * reg_rx_csi_dly_cnt_termen_dlane0    0     0    35     4
 * reg_rx_csi_dly_cnt_settle_dlane0   85    -2   145    -6
 * reg_rx_csi_dly_cnt_termen_dlane1    0     0    35     4
 * reg_rx_csi_dly_cnt_settle_dlane1   85    -2   145    -6
 * reg_rx_csi_dly_cnt_termen_dlane2    0     0    35     4
 * reg_rx_csi_dly_cnt_settle_dlane2   85    -2   145    -6
 * reg_rx_csi_dly_cnt_termen_dlane3    0     0    35     4
 * reg_rx_csi_dly_cnt_settle_dlane3   85    -2   145    -6
 *
 * We use the minimum values of both A and B.
 */

#define DIV_SHIFT	8

static u32 calc_timing(s32 a, int b, s64 link_freq, int accinv)
{
	return accinv * a + (accinv * b * (500000000 >> DIV_SHIFT)
			     / (int)(link_freq >> DIV_SHIFT));
}

static int
ipu_isys_csi2_calc_timing(struct ipu_isys_csi2 *csi2,
			  struct ipu_isys_csi2_timing *timing, u32 accinv)
{
	__s64 link_freq;
	int ret;

	ret = ipu_isys_csi2_get_link_freq(csi2, &link_freq);
	if (ret)
		return ret;

	timing->ctermen = calc_timing(CSI2_CSI_RX_DLY_CNT_TERMEN_CLANE_A,
				      CSI2_CSI_RX_DLY_CNT_TERMEN_CLANE_B,
				      link_freq, accinv);
	timing->csettle = calc_timing(CSI2_CSI_RX_DLY_CNT_SETTLE_CLANE_A,
				      CSI2_CSI_RX_DLY_CNT_SETTLE_CLANE_B,
				      link_freq, accinv);
	dev_dbg(&csi2->isys->adev->dev, "ctermen %u\n", timing->ctermen);
	dev_dbg(&csi2->isys->adev->dev, "csettle %u\n", timing->csettle);

	timing->dtermen = calc_timing(CSI2_CSI_RX_DLY_CNT_TERMEN_DLANE_A,
				      CSI2_CSI_RX_DLY_CNT_TERMEN_DLANE_B,
				      link_freq, accinv);
	timing->dsettle = calc_timing(CSI2_CSI_RX_DLY_CNT_SETTLE_DLANE_A,
				      CSI2_CSI_RX_DLY_CNT_SETTLE_DLANE_B,
				      link_freq, accinv);
	dev_dbg(&csi2->isys->adev->dev, "dtermen %u\n", timing->dtermen);
	dev_dbg(&csi2->isys->adev->dev, "dsettle %u\n", timing->dsettle);

	return 0;
}

static void ipu_isys_csi2_be_enable(struct v4l2_subdev *sd, bool enable)
{
	struct ipu_isys_csi2 *csi2 = to_ipu_isys_csi2(sd);
	struct ipu_device *isp = csi2->isys->adev->isp;
	unsigned int offset, mask;

	if (!enable) {
		/* disable CSI2 legacy error irq */
		offset = IS_IO_CSI2_ERR_LEGACY_IRQ_CTL_BASE(csi2->index);
		mask = IPU7_CSI_RX_ERROR_IRQ_MASK;
		writel(mask, csi2->base + offset + IRQ_CTL_CLEAR);
		writel(0, csi2->base + offset + IRQ_CTL_MASK);
		writel(0, csi2->base + offset + IRQ_CTL_ENABLE);

		/* disable CSI2 legacy sync irq */
		offset = IS_IO_CSI2_SYNC_LEGACY_IRQ_CTL_BASE(csi2->index);
		mask = IPU7_CSI_RX_SYNC_IRQ_MASK;
		writel(mask, csi2->base + offset + IRQ_CTL_CLEAR);
		writel(0, csi2->base + offset + IRQ_CTL_MASK);
		writel(0, csi2->base + offset + IRQ_CTL_ENABLE);

		if (is_ipu7p5(isp->hw_ver)) {
			writel(0, csi2->base + offset + IRQ1_CTL_CLEAR);
			writel(0, csi2->base + offset + IRQ1_CTL_MASK);
			writel(0, csi2->base + offset + IRQ1_CTL_ENABLE);
		}

		return;
	}

	/* enable CSI2 legacy error irq */
	offset = IS_IO_CSI2_ERR_LEGACY_IRQ_CTL_BASE(csi2->index);
	mask = IPU7_CSI_RX_ERROR_IRQ_MASK;
	writel(mask, csi2->base + offset + IRQ_CTL_CLEAR);
	writel(mask, csi2->base + offset + IRQ_CTL_MASK);
	writel(mask, csi2->base + offset + IRQ_CTL_ENABLE);

	/* enable CSI2 legacy sync irq */
	offset = IS_IO_CSI2_SYNC_LEGACY_IRQ_CTL_BASE(csi2->index);
	mask = IPU7_CSI_RX_SYNC_IRQ_MASK;
	writel(mask, csi2->base + offset + IRQ_CTL_CLEAR);
	writel(mask, csi2->base + offset + IRQ_CTL_MASK);
	writel(mask, csi2->base + offset + IRQ_CTL_ENABLE);

	mask = IPU7P5_CSI_RX_SYNC_FE_IRQ_MASK;
	if (is_ipu7p5(isp->hw_ver)) {
		writel(mask, csi2->base + offset + IRQ1_CTL_CLEAR);
		writel(mask, csi2->base + offset + IRQ1_CTL_MASK);
		writel(mask, csi2->base + offset + IRQ1_CTL_ENABLE);
	}
}

static int ipu_isys_csi2_set_stream(struct v4l2_subdev *sd,
				    struct ipu_isys_csi2_timing timing,
				    unsigned int nlanes, int enable)
{
	struct ipu_isys_csi2 *csi2 = to_ipu_isys_csi2(sd);
	struct ipu_isys *isys = csi2->isys;
	struct device *dev = &isys->adev->dev;
	void __iomem *base = isys->pdata->base;
	struct ipu_isys_stream *stream =
		container_of(media_entity_pipeline(&sd->entity),
			     struct ipu_isys_stream, pipe);
	struct ipu_isys_csi2_config *cfg =
		v4l2_get_subdev_hostdata(media_entity_to_v4l2_subdev
					 (stream->external->entity));
	unsigned int port, offset, val;
	int ret;

	port = cfg->port;
	dev_dbg(dev, "stream %s CSI2-%u with %u lanes\n", enable ? "on" : "off",
		port, nlanes);

	if (!enable) {
		ipu_isys_csi2_be_enable(sd, 0);

		offset = IS_IO_GPREGS_BASE;
		val = readl(base + offset + CSI_PORT_CLK_GATE);
		writel(~(1 << port) & val, base + offset + CSI_PORT_CLK_GATE);

		/* power down */
		ipu7_isys_csi_phy_powerdown(isys, cfg);
		writel(0x4, base + offset + CLK_DIV_FACTOR_APB_CLK);

		return ret;
	}

	offset = IS_IO_GPREGS_BASE;
	/* set csi port is using by SW */
	val = readl(base + offset + CSI_PORT_CLK_GATE);
	/* port AB support aggregation, configure 2 ports */
	writel(0x3 << (port & 0x2) | val, base + offset + CSI_PORT_CLK_GATE);
	writel(0x2, base + offset + CLK_DIV_FACTOR_APB_CLK);
	dev_dbg(dev, "port %u PORT_CLK_GATE = %08x DIV_FACTOR_APB_CLK=0x%08x\n",
		port, readl(base + offset + CSI_PORT_CLK_GATE),
		readl(base + offset + CLK_DIV_FACTOR_APB_CLK));
	if (port == 0 && nlanes == 4) {
		dev_info(dev, "CSI port %u in aggregation mode\n", port);
		writel(0x1, base + offset + CSI_PORTAB_AGGREGATION);
	}

	/* input is coming from CSI receiver (sensor) */
	offset = IS_IO_CSI2_ADPL_PORT_BASE(port);
	writel(CSI_SENSOR_INPUT, base + offset + CSI2_ADPL_INPUT_MODE);

	/* Enable DPHY power */
	ret = ipu7_isys_csi_phy_powerup(isys, cfg);
	if (ret) {
		dev_err(dev, "CSI-%d PHY power up failed %d\n", cfg->port, ret);
		return ret;
	}
	ipu_isys_csi2_be_enable(sd, 1);

	return 0;
}

#define CSI2_ACCINV	8

static int set_stream(struct v4l2_subdev *sd, int enable)
{
	struct ipu_isys_csi2 *csi2 = to_ipu_isys_csi2(sd);
	struct ipu_isys_stream *stream =
		to_ipu_isys_pipeline(media_entity_pipeline(&sd->entity));
	struct ipu_isys_csi2_config *cfg;
	struct v4l2_subdev *ext_sd;
	struct ipu_isys_csi2_timing timing = {0};
	unsigned int nlanes;
	int ret;

	dev_dbg(&csi2->isys->adev->dev, "csi2 stream %s\n",
		enable ? "on" : "off");
	if (!stream->external->entity) {
		WARN_ON(1);
		return -ENODEV;
	}
	ext_sd = media_entity_to_v4l2_subdev(stream->external->entity);
	cfg = v4l2_get_subdev_hostdata(ext_sd);

	if (!enable) {
		ipu_isys_csi2_set_stream(sd, timing, 0, enable);
		return 0;
	}

	nlanes = cfg->nlanes;

	dev_dbg(&csi2->isys->adev->dev, "lane nr %d.\n", nlanes);

	ret = ipu_isys_csi2_calc_timing(csi2, &timing, CSI2_ACCINV);
	if (ret)
		return ret;

	ret = ipu_isys_csi2_set_stream(sd, timing, nlanes, enable);

	return ret;
}

static int csi2_link_validate(struct media_link *link)
{
	struct media_pipeline *media_pipe;
	struct ipu_isys_csi2 *csi2;
	struct ipu_isys_stream *stream;

	if (!link->sink->entity || !link->source->entity)
		return -EINVAL;
	media_pipe = media_entity_pipeline(link->sink->entity);
	if (!media_pipe)
		return -EINVAL;
	csi2 =
	    to_ipu_isys_csi2(media_entity_to_v4l2_subdev(link->sink->entity));
	stream = to_ipu_isys_pipeline(media_pipe);
	csi2->receiver_errors = 0;
	stream->csi2 = csi2;

	return v4l2_subdev_link_validate(link);
}

static const struct v4l2_subdev_video_ops csi2_sd_video_ops = {
	.s_stream = set_stream,
};

static int ipu_isys_csi2_get_fmt(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_format *fmt)
{
	return ipu_isys_subdev_get_ffmt(sd, state, fmt);
}

static int ipu_isys_csi2_set_fmt(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_format *fmt)
{
	return ipu_isys_subdev_set_ffmt(sd, state, fmt);
}

static int __subdev_link_validate(struct v4l2_subdev *sd,
				  struct media_link *link,
				  struct v4l2_subdev_format *source_fmt,
				  struct v4l2_subdev_format *sink_fmt)
{
	return ipu_isys_subdev_link_validate(sd, link, source_fmt, sink_fmt);
}

static const struct v4l2_subdev_pad_ops csi2_sd_pad_ops = {
	.link_validate = __subdev_link_validate,
	.get_fmt = ipu_isys_csi2_get_fmt,
	.set_fmt = ipu_isys_csi2_set_fmt,
	.enum_mbus_code = ipu_isys_subdev_enum_mbus_code,
};

static const struct v4l2_subdev_ops csi2_sd_ops = {
	.core = &csi2_sd_core_ops,
	.video = &csi2_sd_video_ops,
	.pad = &csi2_sd_pad_ops,
};

static struct media_entity_operations csi2_entity_ops = {
	.link_validate = csi2_link_validate,
};

static void csi2_set_ffmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *fmt)
{
	enum isys_subdev_prop_tgt tgt = IPU_ISYS_SUBDEV_PROP_TGT_SINK_FMT;
	struct v4l2_mbus_framefmt *ffmt =
		__ipu_isys_get_ffmt(sd, state, fmt->pad,
				    fmt->which);

	if (fmt->format.field != V4L2_FIELD_ALTERNATE)
		fmt->format.field = V4L2_FIELD_NONE;

	if (fmt->pad == CSI2_PAD_SINK) {
		*ffmt = fmt->format;
		ipu_isys_subdev_fmt_propagate(sd, state, &fmt->format, NULL,
					      tgt, fmt->pad, fmt->which);
		return;
	}

	if (sd->entity.pads[fmt->pad].flags & MEDIA_PAD_FL_SOURCE) {
		ffmt->width = fmt->format.width;
		ffmt->height = fmt->format.height;
		ffmt->field = fmt->format.field;
		ffmt->code =
		    ipu_isys_subdev_code_to_uncompressed(fmt->format.code);
		return;
	}

	WARN_ON(1);
}

void ipu_isys_csi2_cleanup(struct ipu_isys_csi2 *csi2)
{
	if (!csi2->isys)
		return;

	v4l2_device_unregister_subdev(&csi2->asd.sd);
	ipu_isys_subdev_cleanup(&csi2->asd);
	csi2->isys = NULL;
}

int ipu_isys_csi2_init(struct ipu_isys_csi2 *csi2,
		       struct ipu_isys *isys,
		       void __iomem *base, unsigned int index)
{
	struct device *dev = &isys->adev->dev;
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
		.pad = CSI2_PAD_SINK,
		.format = {
			   .width = 4096,
			   .height = 3072,
			  },
	};
	unsigned int i;
	int ret;

	dev_dbg(dev, "csi-%d base = 0x%p\n", index, base);
	csi2->isys = isys;
	csi2->base = base;
	csi2->index = index;

	if (is_ipu7p5(isys->adev->isp->hw_ver))
		csi2->legacy_irq_mask = 0x7 << (index * 3);
	else
		csi2->legacy_irq_mask = 0x3 << (index * 2);

	dev_dbg(dev, "csi-%d legacy irq mask = 0x%x\n", index,
		csi2->legacy_irq_mask);

	csi2->asd.sd.entity.ops = &csi2_entity_ops;
	csi2->asd.isys = isys;

	csi2->asd.source = IPU_INSYS_MIPI_PORT_0 + index;
	csi2_supported_codes[CSI2_PAD_SINK] = csi2_supported_codes_pad_sink;

	for (i = 0; i < NR_OF_CSI2_SOURCE_PADS; i++)
		csi2_supported_codes[i + 1] = csi2_supported_codes_pad_source;

	csi2->asd.supported_codes = csi2_supported_codes;
	csi2->asd.set_ffmt = csi2_set_ffmt;

	csi2->asd.sd.flags |= V4L2_SUBDEV_FL_HAS_EVENTS;
	csi2->asd.sd.internal_ops = &csi2_sd_internal_ops;

	ret = ipu_isys_subdev_init(&csi2->asd, &csi2_sd_ops, 0,
				   NR_OF_CSI2_PADS,
				   NR_OF_CSI2_SOURCE_PADS,
				   NR_OF_CSI2_SINK_PADS,
				   csi2->asd.sd.flags);
	if (ret)
		goto fail;

	v4l2_set_subdevdata(&csi2->asd.sd, &csi2->asd);

	snprintf(csi2->asd.sd.name, sizeof(csi2->asd.sd.name),
		 IPU_ISYS_ENTITY_PREFIX " CSI-2 %u", index);

	ret = v4l2_device_register_subdev(&isys->v4l2_dev, &csi2->asd.sd);
	if (ret) {
		dev_err(dev, "can't register v4l2 subdev (%d)\n", ret);
		goto fail;
	}

	mutex_lock(&csi2->asd.mutex);
	__ipu_isys_subdev_set_ffmt(&csi2->asd.sd, NULL, &fmt);
	mutex_unlock(&csi2->asd.mutex);

	return 0;

fail:
	ipu_isys_csi2_cleanup(csi2);

	return ret;
}

void ipu_isys_csi2_sof_event(struct ipu_isys_csi2 *csi2)
{
	struct ipu_isys_stream *stream = NULL;
	struct v4l2_event ev = {
		.type = V4L2_EVENT_FRAME_SYNC,
	};
	struct video_device *vdev = csi2->asd.sd.devnode;
	unsigned long flags;
	unsigned int i;

	spin_lock_irqsave(&csi2->isys->streams_lock, flags);

	for (i = 0; i < IPU_ISYS_MAX_STREAMS; i++) {
		if (csi2->isys->streams_ref_count[i] > 0 &&
		    csi2->isys->streams[i].csi2 == csi2) {
			stream = &csi2->isys->streams[i];
			break;
		}
	}

	/* Pipe already vanished */
	if (!stream) {
		spin_unlock_irqrestore(&csi2->isys->streams_lock, flags);
		return;
	}

	ev.u.frame_sync.frame_sequence = atomic_fetch_inc(&stream->sequence);
	spin_unlock_irqrestore(&csi2->isys->streams_lock, flags);

	v4l2_event_queue(vdev, &ev);
	dev_dbg(&csi2->isys->adev->dev,
		"sof_event::csi2-%i sequence: %i\n",
		csi2->index, ev.u.frame_sync.frame_sequence);
}

void ipu_isys_csi2_eof_event(struct ipu_isys_csi2 *csi2)
{
	struct ipu_isys_stream *stream = NULL;
	unsigned long flags;
	unsigned int i;
	u32 frame_sequence;

	spin_lock_irqsave(&csi2->isys->streams_lock, flags);

	for (i = 0; i < IPU_ISYS_MAX_STREAMS; i++) {
		if (csi2->isys->streams_ref_count[i] > 0 &&
		    csi2->isys->streams[i].csi2 == csi2) {
			stream = &csi2->isys->streams[i];
			break;
		}
	}

	if (stream) {
		frame_sequence = atomic_read(&stream->sequence);
		spin_unlock_irqrestore(&csi2->isys->streams_lock, flags);

		dev_dbg(&csi2->isys->adev->dev,
			"eof_event::csi2-%i sequence: %i\n",
			csi2->index, frame_sequence);
		return;
	}

	spin_unlock_irqrestore(&csi2->isys->streams_lock, flags);
}
