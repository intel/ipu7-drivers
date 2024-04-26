// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2014 - 2024 Intel Corporation

#include <linux/device.h>
#include <linux/module.h>

#include <media/ipu-isys.h>
#include <media/media-entity.h>
#include <media/v4l2-device.h>

#include "ipu.h"
#include "ipu-bus.h"
#include "ipu-isys.h"
#include "ipu-isys-csi2-be.h"
#include "ipu-isys-csi2-regs.h"
#include "ipu-isys-subdev.h"
#include "ipu-isys-video.h"

/*
 * Raw bayer format pixel order MUST BE MAINTAINED in groups of four codes.
 * Otherwise pixel order calculation below WILL BREAK!
 */
static const u32 csi2_be_soc_supported_codes_pad[] = {
	MEDIA_BUS_FMT_Y10_1X10,
	MEDIA_BUS_FMT_RGB565_1X16,
	MEDIA_BUS_FMT_RGB888_1X24,
	MEDIA_BUS_FMT_UYVY8_1X16,
	MEDIA_BUS_FMT_YUYV8_1X16,
	MEDIA_BUS_FMT_SBGGR12_1X12,
	MEDIA_BUS_FMT_SGBRG12_1X12,
	MEDIA_BUS_FMT_SGRBG12_1X12,
	MEDIA_BUS_FMT_SRGGB12_1X12,
	MEDIA_BUS_FMT_SBGGR10_1X10,
	MEDIA_BUS_FMT_SGBRG10_1X10,
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SBGGR8_1X8,
	MEDIA_BUS_FMT_SGBRG8_1X8,
	MEDIA_BUS_FMT_SGRBG8_1X8,
	MEDIA_BUS_FMT_SRGGB8_1X8,
	0,
};

/*
 * Raw bayer format pixel order MUST BE MAINTAINED in groups of four codes.
 * Otherwise pixel order calculation below WILL BREAK!
 */
static const u32 csi2_be_soc_supported_raw_bayer_codes_pad[] = {
	MEDIA_BUS_FMT_SBGGR12_1X12,
	MEDIA_BUS_FMT_SGBRG12_1X12,
	MEDIA_BUS_FMT_SGRBG12_1X12,
	MEDIA_BUS_FMT_SRGGB12_1X12,
	MEDIA_BUS_FMT_SBGGR10_1X10,
	MEDIA_BUS_FMT_SGBRG10_1X10,
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SBGGR8_1X8,
	MEDIA_BUS_FMT_SGBRG8_1X8,
	MEDIA_BUS_FMT_SGRBG8_1X8,
	MEDIA_BUS_FMT_SRGGB8_1X8,
	0,
};

static int get_supported_code_index(u32 code)
{
	unsigned int i;

	for (i = 0; csi2_be_soc_supported_raw_bayer_codes_pad[i]; i++) {
		if (csi2_be_soc_supported_raw_bayer_codes_pad[i] == code)
			return i;
	}
	return -EINVAL;
}

static const u32 *csi2_be_soc_supported_codes[NR_OF_CSI2_BE_SOC_PADS];

static const struct v4l2_subdev_internal_ops csi2_be_soc_sd_internal_ops = {
	.open = ipu_isys_subdev_open,
};

static int set_stream(struct v4l2_subdev *sd, int enable)
{
	return 0;
}

static const struct v4l2_subdev_video_ops csi2_be_soc_sd_video_ops = {
	.s_stream = set_stream,
};

static int
__subdev_link_validate(struct v4l2_subdev *sd, struct media_link *link,
		       struct v4l2_subdev_format *source_fmt,
		       struct v4l2_subdev_format *sink_fmt)
{
	struct ipu_isys_stream *stream =
		container_of(media_entity_pipeline(&sd->entity),
			     struct ipu_isys_stream, pipe);

	stream->csi2_be_soc = to_ipu_isys_csi2_be_soc(sd);
	return ipu_isys_subdev_link_validate(sd, link, source_fmt, sink_fmt);
}

static int
ipu_isys_csi2_be_soc_set_sel(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state,
			     struct v4l2_subdev_selection *sel)
{
	struct ipu_isys_subdev *asd = to_ipu_isys_subdev(sd);
	struct media_pad *pad = &asd->sd.entity.pads[sel->pad];

	if (sel->target == V4L2_SEL_TGT_CROP &&
	    pad->flags & MEDIA_PAD_FL_SOURCE &&
	    asd->valid_tgts[sel->pad].crop) {
		enum isys_subdev_prop_tgt tgt =
		    IPU_ISYS_SUBDEV_PROP_TGT_SOURCE_CROP;
		struct v4l2_mbus_framefmt *ffmt =
			__ipu_isys_get_ffmt(sd, state, sel->pad, sel->which);

		if (get_supported_code_index(ffmt->code) < 0) {
			/* Non-bayer formats can't be odd lines cropped */
			sel->r.left &= ~1;
			sel->r.top &= ~1;
		}

		sel->r.width = clamp(sel->r.width, IPU_ISYS_MIN_WIDTH,
				     IPU_ISYS_MAX_WIDTH);

		sel->r.height = clamp(sel->r.height, IPU_ISYS_MIN_HEIGHT,
				      IPU_ISYS_MAX_HEIGHT);

		*__ipu_isys_get_selection(sd, state, sel->target, sel->pad,
					  sel->which) = sel->r;
		ipu_isys_subdev_fmt_propagate(sd, state, NULL, &sel->r,
					      tgt, sel->pad, sel->which);
		return 0;
	}
	return -EINVAL;
}

static const struct v4l2_subdev_pad_ops csi2_be_soc_sd_pad_ops = {
	.link_validate = __subdev_link_validate,
	.get_fmt = ipu_isys_subdev_get_ffmt,
	.set_fmt = ipu_isys_subdev_set_ffmt,
	.get_selection = ipu_isys_subdev_get_sel,
	.set_selection = ipu_isys_csi2_be_soc_set_sel,
	.enum_mbus_code = ipu_isys_subdev_enum_mbus_code,
};

static const struct v4l2_subdev_ops csi2_be_soc_sd_ops = {
	.video = &csi2_be_soc_sd_video_ops,
	.pad = &csi2_be_soc_sd_pad_ops,
};

static struct media_entity_operations csi2_be_soc_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static void csi2_be_soc_set_ffmt(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *ffmt =
		__ipu_isys_get_ffmt(sd, state, fmt->pad,
				    fmt->which);

	if (sd->entity.pads[fmt->pad].flags & MEDIA_PAD_FL_SINK) {
		if (fmt->format.field != V4L2_FIELD_ALTERNATE)
			fmt->format.field = V4L2_FIELD_NONE;
		*ffmt = fmt->format;

		ipu_isys_subdev_fmt_propagate(sd, state, &fmt->format,
					      NULL,
					      IPU_ISYS_SUBDEV_PROP_TGT_SINK_FMT,
					      fmt->pad, fmt->which);
	} else if (sd->entity.pads[fmt->pad].flags & MEDIA_PAD_FL_SOURCE) {
		struct v4l2_mbus_framefmt *sink_ffmt;
		struct v4l2_rect *r = __ipu_isys_get_selection(sd, state,
			V4L2_SEL_TGT_CROP, fmt->pad, fmt->which);
		struct ipu_isys_subdev *asd = to_ipu_isys_subdev(sd);
		u32 code;
		int idx;

		sink_ffmt = __ipu_isys_get_ffmt(sd, state, 0, fmt->which);
		code = sink_ffmt->code;
		idx = get_supported_code_index(code);

		if (asd->valid_tgts[fmt->pad].crop && idx >= 0) {
			int crop_info = 0;

			/* Only croping odd line at top side. */
			if (r->top & 1)
				crop_info |= CSI2_BE_CROP_VER;

			code = csi2_be_soc_supported_raw_bayer_codes_pad
				[((idx & CSI2_BE_CROP_MASK) ^ crop_info)
				+ (idx & ~CSI2_BE_CROP_MASK)];

		}
		ffmt->code = code;
		ffmt->width = r->width;
		ffmt->height = r->height;
		ffmt->field = sink_ffmt->field;
	}
}

void ipu_isys_csi2_be_soc_cleanup(struct ipu_isys_csi2_be_soc *csi2_be_soc)
{
	v4l2_device_unregister_subdev(&csi2_be_soc->asd.sd);
	ipu_isys_subdev_cleanup(&csi2_be_soc->asd);
	ipu_isys_video_cleanup(&csi2_be_soc->av);
}

int ipu_isys_csi2_be_soc_init(struct ipu_isys_csi2_be_soc *csi2_be_soc,
			      struct ipu_isys *isys, int index)
{
	struct device *dev = &isys->adev->dev;
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
		.pad = CSI2_BE_SOC_PAD_SINK,
		.format = {
			   .width = 4096,
			   .height = 3072,
			   },
	};
	unsigned int i;
	int ret;

	csi2_be_soc->asd.sd.entity.ops = &csi2_be_soc_entity_ops;
	csi2_be_soc->asd.isys = isys;

	for (i = 0; i < NR_OF_CSI2_BE_SOC_PADS; i++)
		csi2_be_soc_supported_codes[i] =
			csi2_be_soc_supported_codes_pad;
	csi2_be_soc->asd.supported_codes = csi2_be_soc_supported_codes;
	csi2_be_soc->asd.set_ffmt = csi2_be_soc_set_ffmt;
	csi2_be_soc->asd.sd.internal_ops = &csi2_be_soc_sd_internal_ops;

	ret = ipu_isys_subdev_init(&csi2_be_soc->asd,
				   &csi2_be_soc_sd_ops, 0,
				   NR_OF_CSI2_BE_SOC_PADS,
				   NR_OF_CSI2_BE_SOC_SOURCE_PADS,
				   NR_OF_CSI2_BE_SOC_SINK_PADS, 0);
	if (ret)
		goto fail;

	csi2_be_soc->asd.valid_tgts[CSI2_BE_SOC_PAD_SOURCE].crop = true;

	snprintf(csi2_be_soc->asd.sd.name, sizeof(csi2_be_soc->asd.sd.name),
		 IPU_ISYS_ENTITY_PREFIX " CSI2 BE SOC %d", index);

	v4l2_set_subdevdata(&csi2_be_soc->asd.sd, &csi2_be_soc->asd);

	ret = v4l2_device_register_subdev(&isys->v4l2_dev,
					  &csi2_be_soc->asd.sd);
	if (ret) {
		dev_err(dev, "can't register v4l2 subdev\n");
		goto fail;
	}

	snprintf(csi2_be_soc->av.vdev.name, sizeof(csi2_be_soc->av.vdev.name),
		 IPU_ISYS_ENTITY_PREFIX " BE SOC capture %d", index);

	fmt.pad = CSI2_BE_SOC_PAD_SINK;
	ipu_isys_subdev_set_ffmt(&csi2_be_soc->asd.sd, NULL, &fmt);
	fmt.pad = CSI2_BE_SOC_PAD_SOURCE;
	ipu_isys_subdev_set_ffmt(&csi2_be_soc->asd.sd, NULL, &fmt);

	/*
	 * Pin type could be overwritten for YUV422 to I420 case, at
	 * set_format phase
	 */
	csi2_be_soc->av.isys = isys;
	csi2_be_soc->av.aq.fill_frame_buff_set_pin =
		ipu_isys_buffer_to_fw_frame_buff_pin;
	csi2_be_soc->av.aq.link_fmt_validate = ipu_isys_link_fmt_validate;
	csi2_be_soc->av.aq.vbq.buf_struct_size =
		sizeof(struct ipu_isys_video_buffer);

	ret = ipu_isys_video_init(&csi2_be_soc->av,
				  &csi2_be_soc->asd.sd.entity,
				  CSI2_BE_SOC_PAD_SOURCE,
				  MEDIA_PAD_FL_SINK,
				  MEDIA_LNK_FL_DYNAMIC);
	if (ret) {
		dev_err(dev, "can't init video node\n");
		goto fail;
	}

	return 0;

fail:
	ipu_isys_csi2_be_soc_cleanup(csi2_be_soc);

	return ret;
}
