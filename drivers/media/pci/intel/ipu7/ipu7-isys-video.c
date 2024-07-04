// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2013 - 2024 Intel Corporation
 */

#include <linux/align.h>
#include <linux/bits.h>
#include <linux/bug.h>
#include <linux/completion.h>
#include <linux/container_of.h>
#include <linux/compat.h>
#include <linux/device.h>
#include <linux/iopoll.h>
#include <linux/list.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/version.h>

#include <media/media-entity.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mc.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-v4l2.h>

#include "abi/ipu7_fw_isys_abi.h"

#include "ipu7.h"
#include "ipu7-bus.h"
#include "ipu7-buttress-regs.h"
#include "ipu7-fw-isys.h"
#include "ipu7-isys.h"
#include "ipu7-isys-video.h"
#include "ipu7-platform-regs.h"

const struct ipu7_isys_pixelformat ipu7_isys_pfmts[] = {
	{V4L2_PIX_FMT_SBGGR12, 16, 12, MEDIA_BUS_FMT_SBGGR12_1X12,
	 IPU_INSYS_FRAME_FORMAT_RAW16},
	{V4L2_PIX_FMT_SGBRG12, 16, 12, MEDIA_BUS_FMT_SGBRG12_1X12,
	 IPU_INSYS_FRAME_FORMAT_RAW16},
	{V4L2_PIX_FMT_SGRBG12, 16, 12, MEDIA_BUS_FMT_SGRBG12_1X12,
	 IPU_INSYS_FRAME_FORMAT_RAW16},
	{V4L2_PIX_FMT_SRGGB12, 16, 12, MEDIA_BUS_FMT_SRGGB12_1X12,
	 IPU_INSYS_FRAME_FORMAT_RAW16},
	{V4L2_PIX_FMT_SBGGR10, 16, 10, MEDIA_BUS_FMT_SBGGR10_1X10,
	 IPU_INSYS_FRAME_FORMAT_RAW16},
	{V4L2_PIX_FMT_SGBRG10, 16, 10, MEDIA_BUS_FMT_SGBRG10_1X10,
	 IPU_INSYS_FRAME_FORMAT_RAW16},
	{V4L2_PIX_FMT_SGRBG10, 16, 10, MEDIA_BUS_FMT_SGRBG10_1X10,
	 IPU_INSYS_FRAME_FORMAT_RAW16},
	{V4L2_PIX_FMT_SRGGB10, 16, 10, MEDIA_BUS_FMT_SRGGB10_1X10,
	 IPU_INSYS_FRAME_FORMAT_RAW16},
	{V4L2_PIX_FMT_SBGGR8, 8, 8, MEDIA_BUS_FMT_SBGGR8_1X8,
	 IPU_INSYS_FRAME_FORMAT_RAW8},
	{V4L2_PIX_FMT_SGBRG8, 8, 8, MEDIA_BUS_FMT_SGBRG8_1X8,
	 IPU_INSYS_FRAME_FORMAT_RAW8},
	{V4L2_PIX_FMT_SGRBG8, 8, 8, MEDIA_BUS_FMT_SGRBG8_1X8,
	 IPU_INSYS_FRAME_FORMAT_RAW8},
	{V4L2_PIX_FMT_SRGGB8, 8, 8, MEDIA_BUS_FMT_SRGGB8_1X8,
	 IPU_INSYS_FRAME_FORMAT_RAW8},
	{V4L2_PIX_FMT_SBGGR12P, 12, 12, MEDIA_BUS_FMT_SBGGR12_1X12,
	 IPU_INSYS_FRAME_FORMAT_RAW12},
	{V4L2_PIX_FMT_SGBRG12P, 12, 12, MEDIA_BUS_FMT_SGBRG12_1X12,
	 IPU_INSYS_FRAME_FORMAT_RAW12},
	{V4L2_PIX_FMT_SGRBG12P, 12, 12, MEDIA_BUS_FMT_SGRBG12_1X12,
	 IPU_INSYS_FRAME_FORMAT_RAW12},
	{V4L2_PIX_FMT_SRGGB12P, 12, 12, MEDIA_BUS_FMT_SRGGB12_1X12,
	 IPU_INSYS_FRAME_FORMAT_RAW12},
	{V4L2_PIX_FMT_SBGGR10P, 10, 10, MEDIA_BUS_FMT_SBGGR10_1X10,
	 IPU_INSYS_FRAME_FORMAT_RAW10},
	{V4L2_PIX_FMT_SGBRG10P, 10, 10, MEDIA_BUS_FMT_SGBRG10_1X10,
	 IPU_INSYS_FRAME_FORMAT_RAW10},
	{V4L2_PIX_FMT_SGRBG10P, 10, 10, MEDIA_BUS_FMT_SGRBG10_1X10,
	 IPU_INSYS_FRAME_FORMAT_RAW10},
	{V4L2_PIX_FMT_SRGGB10P, 10, 10, MEDIA_BUS_FMT_SRGGB10_1X10,
	 IPU_INSYS_FRAME_FORMAT_RAW10},
	{V4L2_PIX_FMT_UYVY, 16, 16, MEDIA_BUS_FMT_UYVY8_1X16,
	 IPU_INSYS_FRAME_FORMAT_UYVY},
	{V4L2_PIX_FMT_YUYV, 16, 16, MEDIA_BUS_FMT_YUYV8_1X16,
	 IPU_INSYS_FRAME_FORMAT_YUYV},
	{V4L2_PIX_FMT_RGB565, 16, 16, MEDIA_BUS_FMT_RGB565_1X16,
	 IPU_INSYS_FRAME_FORMAT_RGB565},
	{V4L2_PIX_FMT_BGR24, 24, 24, MEDIA_BUS_FMT_RGB888_1X24,
	 IPU_INSYS_FRAME_FORMAT_RGBA888},
};

static int video_open(struct file *file)
{
	struct ipu7_isys_video *av = video_drvdata(file);
	struct ipu7_isys *isys = av->isys;
	struct ipu7_bus_device *adev = isys->adev;

	mutex_lock(&isys->mutex);
	if (isys->need_reset) {
		mutex_unlock(&isys->mutex);
		dev_warn(&adev->auxdev.dev, "isys power cycle required\n");
		return -EIO;
	}
	mutex_unlock(&isys->mutex);

	return v4l2_fh_open(file);
}

const struct ipu7_isys_pixelformat *ipu7_isys_get_isys_format(u32 pixelformat)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ipu7_isys_pfmts); i++) {
		const struct ipu7_isys_pixelformat *pfmt = &ipu7_isys_pfmts[i];

		if (pfmt->pixelformat == pixelformat)
			return pfmt;
	}

	return &ipu7_isys_pfmts[0];
}

int ipu7_isys_vidioc_querycap(struct file *file, void *fh,
			      struct v4l2_capability *cap)
{
	struct ipu7_isys_video *av = video_drvdata(file);

	strscpy(cap->driver, IPU_ISYS_NAME, sizeof(cap->driver));
	strscpy(cap->card, av->isys->media_dev.model, sizeof(cap->card));
	return 0;
}

int ipu7_isys_vidioc_enum_fmt(struct file *file, void *fh,
			      struct v4l2_fmtdesc *f)
{
	unsigned int i, num_found;

	for (i = 0, num_found = 0; i < ARRAY_SIZE(ipu7_isys_pfmts); i++) {
		if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
			continue;

		if (f->mbus_code && f->mbus_code != ipu7_isys_pfmts[i].code)
			continue;

		if (num_found < f->index) {
			num_found++;
			continue;
		}

		f->flags = 0;
		f->pixelformat = ipu7_isys_pfmts[i].pixelformat;

		return 0;
	}

	return -EINVAL;
}

static int ipu7_isys_vidioc_enum_framesizes(struct file *file, void *fh,
					    struct v4l2_frmsizeenum *fsize)
{
	unsigned int i;

	if (fsize->index > 0)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(ipu7_isys_pfmts); i++) {
		if (fsize->pixel_format != ipu7_isys_pfmts[i].pixelformat)
			continue;

		fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
		fsize->stepwise.min_width = IPU_ISYS_MIN_WIDTH;
		fsize->stepwise.max_width = IPU_ISYS_MAX_WIDTH;
		fsize->stepwise.min_height = IPU_ISYS_MIN_HEIGHT;
		fsize->stepwise.max_height = IPU_ISYS_MAX_HEIGHT;
		fsize->stepwise.step_width = 2;
		fsize->stepwise.step_height = 2;

		return 0;
	}

	return -EINVAL;
}

static int ipu7_isys_vidioc_g_fmt_vid_cap(struct file *file, void *fh,
					  struct v4l2_format *f)
{
	struct ipu7_isys_video *av = video_drvdata(file);

	f->fmt.pix = av->pix_fmt;

	return 0;
}

static void ipu7_isys_try_fmt_cap(struct ipu7_isys_video *av, u32 type,
				  u32 *format, u32 *width, u32 *height,
				  u32 *bytesperline, u32 *sizeimage)
{
	const struct ipu7_isys_pixelformat *pfmt =
		ipu7_isys_get_isys_format(*format);

	*format = pfmt->pixelformat;
	*width = clamp(*width, IPU_ISYS_MIN_WIDTH, IPU_ISYS_MAX_WIDTH);
	*height = clamp(*height, IPU_ISYS_MIN_HEIGHT, IPU_ISYS_MAX_HEIGHT);

	if (pfmt->bpp != pfmt->bpp_packed)
		*bytesperline = *width * DIV_ROUND_UP(pfmt->bpp, BITS_PER_BYTE);
	else
		*bytesperline = DIV_ROUND_UP(*width * pfmt->bpp, BITS_PER_BYTE);

	*bytesperline = ALIGN(*bytesperline, av->isys->line_align);

	/*
	 * (height + 1) * bytesperline due to a hardware issue: the DMA unit
	 * is a power of two, and a line should be transferred as few units
	 * as possible. The result is that up to line length more data than
	 * the image size may be transferred to memory after the image.
	 * Another limitation is the GDA allocation unit size. For low
	 * resolution it gives a bigger number. Use larger one to avoid
	 * memory corruption.
	 */
	*sizeimage = *bytesperline * *height +
		max(*bytesperline,
		    av->isys->pdata->ipdata->isys_dma_overshoot);
}

static void __ipu_isys_vidioc_try_fmt_vid_cap(struct ipu7_isys_video *av,
					      struct v4l2_format *f)
{
	ipu7_isys_try_fmt_cap(av, f->type, &f->fmt.pix.pixelformat,
			      &f->fmt.pix.width, &f->fmt.pix.height,
			      &f->fmt.pix.bytesperline, &f->fmt.pix.sizeimage);

	f->fmt.pix.field = V4L2_FIELD_NONE;
	f->fmt.pix.colorspace = V4L2_COLORSPACE_RAW;
	f->fmt.pix.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	f->fmt.pix.quantization = V4L2_QUANTIZATION_DEFAULT;
	f->fmt.pix.xfer_func = V4L2_XFER_FUNC_DEFAULT;
}

static int ipu7_isys_vidioc_try_fmt_vid_cap(struct file *file, void *fh,
					    struct v4l2_format *f)
{
	struct ipu7_isys_video *av = video_drvdata(file);

	if (vb2_is_busy(&av->aq.vbq))
		return -EBUSY;

	__ipu_isys_vidioc_try_fmt_vid_cap(av, f);

	return 0;
}

static int ipu7_isys_vidioc_s_fmt_vid_cap(struct file *file, void *fh,
					  struct v4l2_format *f)
{
	struct ipu7_isys_video *av = video_drvdata(file);

	ipu7_isys_vidioc_try_fmt_vid_cap(file, fh, f);
	av->pix_fmt = f->fmt.pix;

	return 0;
}

static int ipu7_isys_vidioc_reqbufs(struct file *file, void *priv,
				    struct v4l2_requestbuffers *p)
{
	struct ipu7_isys_video *av = video_drvdata(file);
	int ret;

	av->aq.vbq.is_multiplanar = V4L2_TYPE_IS_MULTIPLANAR(p->type);
	av->aq.vbq.is_output = V4L2_TYPE_IS_OUTPUT(p->type);

	ret = vb2_queue_change_type(&av->aq.vbq, p->type);
	if (ret)
		return ret;

	return vb2_ioctl_reqbufs(file, priv, p);
}

static int ipu7_isys_vidioc_create_bufs(struct file *file, void *priv,
					struct v4l2_create_buffers *p)
{
	struct ipu7_isys_video *av = video_drvdata(file);
	int ret;

	av->aq.vbq.is_multiplanar = V4L2_TYPE_IS_MULTIPLANAR(p->format.type);
	av->aq.vbq.is_output = V4L2_TYPE_IS_OUTPUT(p->format.type);

	ret = vb2_queue_change_type(&av->aq.vbq, p->format.type);
	if (ret)
		return ret;

	return vb2_ioctl_create_bufs(file, priv, p);
}

/*
 * Return true if an entity directly connected to an Iunit entity is
 * an image source for the ISP. This can be any external directly
 * connected entity or any of the test pattern generators in the
 * Iunit.
 */
static bool is_external(struct ipu7_isys_video *av, struct media_entity *entity)
{
	struct v4l2_subdev *sd;
#ifdef CONFIG_VIDEO_INTEL_IPU7_MGC
	unsigned int i;
#endif

	/* All video nodes are ours. */
	if (!is_media_entity_v4l2_subdev(entity))
		return false;

	sd = media_entity_to_v4l2_subdev(entity);
	if (strncmp(sd->name, IPU_ISYS_ENTITY_PREFIX,
		    strlen(IPU_ISYS_ENTITY_PREFIX)) != 0)
		return true;

#ifdef CONFIG_VIDEO_INTEL_IPU7_MGC
	for (i = 0; i < av->isys->pdata->ipdata->tpg.ntpgs &&
		     av->isys->tpg[i].isys; i++)
		if (entity == &av->isys->tpg[i].asd.sd.entity)
			return true;
#endif

	return false;
}

static int link_validate(struct media_link *link)
{
	struct ipu7_isys_video *av =
		container_of(link->sink, struct ipu7_isys_video, pad);
	struct ipu7_isys_stream *stream = av->stream;
	struct v4l2_mbus_framefmt *ffmt;
	struct ipu7_isys_subdev *asd;
	struct v4l2_subdev *sd;
	u32 code;

	if (!stream || !link->source->entity)
		return -EINVAL;

	sd = media_entity_to_v4l2_subdev(link->source->entity);
	asd = to_ipu7_isys_subdev(sd);
	ffmt = &asd->ffmt[link->source->index];
	code = ipu7_isys_get_isys_format(av->pix_fmt.pixelformat)->code;
	if (ffmt->code != code || ffmt->width != av->pix_fmt.width ||
	    ffmt->height != av->pix_fmt.height) {
		dev_err(&asd->isys->adev->auxdev.dev,
			"vdev link validation failed. %dx%d,%x != %dx%d,%x\n",
			ffmt->width, ffmt->height, ffmt->code,
			av->pix_fmt.width, av->pix_fmt.height, code);
		return -EINVAL;
	}

	if (is_external(av, link->source->entity)) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 0, 0)
		stream->external =
			media_entity_remote_pad(av->vdev.entity.pads);
#else
		stream->external =
			media_pad_remote_pad_first(av->vdev.entity.pads);
#endif
		stream->stream_source  = to_ipu7_isys_subdev(sd)->source;
	}

	return 0;
}

static void get_stream_opened(struct ipu7_isys_video *av)
{
	unsigned long flags;

	spin_lock_irqsave(&av->isys->streams_lock, flags);
	av->isys->stream_opened++;
	spin_unlock_irqrestore(&av->isys->streams_lock, flags);
}

static void put_stream_opened(struct ipu7_isys_video *av)
{
	unsigned long flags;

	spin_lock_irqsave(&av->isys->streams_lock, flags);
	av->isys->stream_opened--;
	spin_unlock_irqrestore(&av->isys->streams_lock, flags);
}

static int get_external_facing_format(struct ipu7_isys_stream *stream,
				      struct v4l2_subdev_format *format)
{
	struct device *dev = &stream->isys->adev->auxdev.dev;
	struct v4l2_subdev *sd;
	struct media_pad *external_facing;

	if (!stream->external->entity) {
		WARN_ON(1);
		return -ENODEV;
	}
	sd = media_entity_to_v4l2_subdev(stream->external->entity);
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 0, 0)
	external_facing = (strncmp(sd->name, IPU_ISYS_ENTITY_PREFIX,
				   strlen(IPU_ISYS_ENTITY_PREFIX)) == 0) ?
		stream->external :
		media_entity_remote_pad(stream->external);
#else
	external_facing = (strncmp(sd->name, IPU_ISYS_ENTITY_PREFIX,
				   strlen(IPU_ISYS_ENTITY_PREFIX)) == 0) ?
		stream->external :
		media_pad_remote_pad_first(stream->external);
#endif
	if (WARN_ON(!external_facing)) {
		dev_warn(dev, "no external facing pad --- driver bug?\n");
		return -EINVAL;
	}

	format->which = V4L2_SUBDEV_FORMAT_ACTIVE;
	format->pad = 0;
	sd = media_entity_to_v4l2_subdev(external_facing->entity);

	return v4l2_subdev_call(sd, pad, get_fmt, NULL, format);
}

void ipu7_isys_prepare_fw_cfg_default(struct ipu7_isys_video *av,
				      struct ipu7_insys_stream_cfg *cfg)
{
	struct ipu7_isys_stream *stream = av->stream;
	const struct ipu7_isys_pixelformat *pfmt;
	struct ipu7_isys_queue *aq = &av->aq;
	struct ipu7_insys_output_pin *pin_info;
	int pin = cfg->nof_output_pins++;

	aq->fw_output = pin;
	stream->output_pins[pin].pin_ready = ipu7_isys_queue_buf_ready;
	stream->output_pins[pin].aq = aq;

	pin_info = &cfg->output_pins[pin];
	/* output pin msg link */
	pin_info->link.buffer_lines = 0;
	pin_info->link.foreign_key = IPU_MSG_LINK_FOREIGN_KEY_NONE;
	pin_info->link.granularity_pointer_update = 0;
	pin_info->link.msg_link_streaming_mode =
		IA_GOFO_MSG_LINK_STREAMING_MODE_SOFF;
	pin_info->link.pbk_id = IPU_MSG_LINK_PBK_ID_DONT_CARE;
	pin_info->link.pbk_slot_id = IPU_MSG_LINK_PBK_SLOT_ID_DONT_CARE;
	pin_info->link.dest = IPU_INSYS_OUTPUT_LINK_DEST_MEM;
	pin_info->link.use_sw_managed = 1;

	/* output pin crop */
	pin_info->crop.line_top = 0;
	pin_info->crop.line_bottom = 0;

	/* output de-compression */
	pin_info->dpcm.enable = 0;

	/* frame fomat type */
	pfmt = ipu7_isys_get_isys_format(av->pix_fmt.pixelformat);
	pin_info->ft = (u16)pfmt->css_pixelformat;

	/* stride in bytes */
	pin_info->stride = av->pix_fmt.bytesperline;
	pin_info->send_irq = 1;
	pin_info->early_ack_en = 0;

	/* input pin id */
	pin_info->input_pin_id = 0;

	cfg->vc = 0;
}

/* Create stream and start it using the CSS FW ABI. */
static int start_stream_firmware(struct ipu7_isys_video *av,
				 struct ipu7_isys_buffer_list *bl)
{
	struct ipu7_isys_stream *stream = av->stream;
	struct device *dev = &av->isys->adev->auxdev.dev;
	struct ipu7_insys_stream_cfg *stream_cfg;
	struct isys_fw_msgs *msg = NULL;
	struct ipu7_insys_buffset *buf = NULL;
	struct ipu7_isys_queue *aq;
	struct v4l2_subdev_format source_fmt = { 0 };
	enum ipu7_insys_send_type send_type;
	const struct ipu7_isys_pixelformat *pfmt;
	int ret, retout, tout;

	ret = get_external_facing_format(stream, &source_fmt);
	if (ret)
		return ret;

	msg = ipu7_get_fw_msg_buf(stream);
	if (!msg)
		return -ENOMEM;

	stream_cfg = &msg->fw_msg.stream;
	stream_cfg->input_pins[0].input_res.width = source_fmt.format.width;
	stream_cfg->input_pins[0].input_res.height = source_fmt.format.height;
	stream_cfg->input_pins[0].dt =
		ipu7_isys_mbus_code_to_data_type(source_fmt.format.code);
	/* It is similar with mipi_store_mod */
	stream_cfg->input_pins[0].disable_mipi_unpacking = 0;
	pfmt = ipu7_isys_get_isys_format(av->pix_fmt.pixelformat);
	if (pfmt->bpp == pfmt->bpp_packed && pfmt->bpp % BITS_PER_BYTE)
		stream_cfg->input_pins[0].disable_mipi_unpacking = 1;
	stream_cfg->input_pins[0].mapped_dt = N_IPU_INSYS_MIPI_DATA_TYPE;
	stream_cfg->input_pins[0].dt_rename_mode = IPU_INSYS_MIPI_DT_NO_RENAME;
	stream_cfg->input_pins[0].sync_msg_map =
		(IPU_INSYS_STREAM_SYNC_MSG_SEND_RESP_SOF |
		 IPU_INSYS_STREAM_SYNC_MSG_SEND_RESP_SOF_DISCARDED);

	stream_cfg->stream_msg_map = IPU_INSYS_STREAM_ENABLE_MSG_SEND_RESP;

	/* if enable polling isys interrupt, the follow values maybe set */
	stream_cfg->stream_msg_map |= IPU_INSYS_STREAM_ENABLE_MSG_SEND_IRQ;
	stream_cfg->input_pins[0].sync_msg_map |=
		(IPU_INSYS_STREAM_SYNC_MSG_SEND_IRQ_SOF |
		 IPU_INSYS_STREAM_SYNC_MSG_SEND_IRQ_SOF_DISCARDED);
	/* TODO: it needs to handle the port_id, it isn't source. */
	stream_cfg->port_id = stream->stream_source;
	stream_cfg->vc = 0;
	stream_cfg->nof_input_pins = 1;

	list_for_each_entry(aq, &stream->queues, node) {
		struct ipu7_isys_video *__av = ipu7_isys_queue_to_video(aq);

		ipu7_isys_prepare_fw_cfg_default(__av, stream_cfg);
	}

	ipu7_fw_isys_dump_stream_cfg(dev, stream_cfg);

	stream->nr_output_pins = stream_cfg->nof_output_pins;

	reinit_completion(&stream->stream_open_completion);

	ret = ipu7_fw_isys_complex_cmd(av->isys, stream->stream_handle,
				       stream_cfg, msg->dma_addr,
				       sizeof(*stream_cfg),
				       IPU_INSYS_SEND_TYPE_STREAM_OPEN);
	if (ret < 0) {
		dev_err(dev, "can't open stream (%d)\n", ret);
		ipu7_put_fw_msg_buf(av->isys, (uintptr_t)stream_cfg);
		return ret;
	}

	get_stream_opened(av);

	tout = wait_for_completion_timeout(&stream->stream_open_completion,
					   FW_CALL_TIMEOUT_JIFFIES);

	ipu7_put_fw_msg_buf(av->isys, (uintptr_t)stream_cfg);

	if (!tout) {
		dev_err(dev, "stream open time out\n");
		ret = -ETIMEDOUT;
		goto out_put_stream_opened;
	}
	if (stream->error) {
		dev_err(dev, "stream open error: %d\n", stream->error);
		ret = -EIO;
		goto out_put_stream_opened;
	}
	dev_dbg(dev, "start stream: open complete\n");

	if (WARN_ON(!bl))
		return -EIO;

	msg = ipu7_get_fw_msg_buf(stream);
	if (!msg) {
		ret = -ENOMEM;
		goto out_put_stream_opened;
	}
	buf = &msg->fw_msg.frame;

	ipu7_isys_buffer_to_fw_frame_buff(buf, stream, bl);
	ipu7_isys_buffer_list_queue(bl, IPU_ISYS_BUFFER_LIST_FL_ACTIVE, 0);

	reinit_completion(&stream->stream_start_completion);

	send_type = IPU_INSYS_SEND_TYPE_STREAM_START_AND_CAPTURE;
	ipu7_fw_isys_dump_frame_buff_set(dev, buf,
					 stream_cfg->nof_output_pins);
	ret = ipu7_fw_isys_complex_cmd(av->isys, stream->stream_handle, buf,
				       msg->dma_addr, sizeof(*buf),
				       send_type);
	if (ret < 0) {
		dev_err(dev, "can't start streaming (%d)\n", ret);
		goto out_stream_close;
	}

	tout = wait_for_completion_timeout(&stream->stream_start_completion,
					   FW_CALL_TIMEOUT_JIFFIES);
	if (!tout) {
		dev_err(dev, "stream start time out\n");
		ret = -ETIMEDOUT;
		goto out_stream_close;
	}
	if (stream->error) {
		dev_err(dev, "stream start error: %d\n", stream->error);
		ret = -EIO;
		goto out_stream_close;
	}
	dev_dbg(dev, "start stream: complete\n");

	return 0;

out_stream_close:
	reinit_completion(&stream->stream_close_completion);

	retout = ipu7_fw_isys_simple_cmd(av->isys, stream->stream_handle,
					 IPU_INSYS_SEND_TYPE_STREAM_CLOSE);
	if (retout < 0) {
		dev_dbg(dev, "can't close stream (%d)\n", retout);
		goto out_put_stream_opened;
	}

	tout = wait_for_completion_timeout(&stream->stream_close_completion,
					   FW_CALL_TIMEOUT_JIFFIES);
	if (!tout)
		dev_err(dev, "stream close time out\n");
	else if (stream->error)
		dev_err(dev, "stream close error: %d\n", stream->error);
	else
		dev_dbg(dev, "stream close complete\n");

out_put_stream_opened:
	put_stream_opened(av);

	return ret;
}

static void stop_streaming_firmware(struct ipu7_isys_video *av)
{
	struct ipu7_isys_stream *stream = av->stream;
	struct device *dev = &av->isys->adev->auxdev.dev;
	int ret, tout;
	enum ipu7_insys_send_type send_type =
		IPU_INSYS_SEND_TYPE_STREAM_FLUSH;

	reinit_completion(&stream->stream_stop_completion);

	ret = ipu7_fw_isys_simple_cmd(av->isys, stream->stream_handle,
				      send_type);

	if (ret < 0) {
		dev_err(dev, "can't stop stream (%d)\n", ret);
		return;
	}

	tout = wait_for_completion_timeout(&stream->stream_stop_completion,
					   FW_CALL_TIMEOUT_JIFFIES);
	if (!tout)
		dev_err(dev, "stream stop time out\n");
	else if (stream->error)
		dev_err(dev, "stream stop error: %d\n", stream->error);
	else
		dev_dbg(dev, "stop stream: complete\n");
}

static void close_streaming_firmware(struct ipu7_isys_video *av)
{
	struct ipu7_isys_stream *stream =  av->stream;
	struct device *dev = &av->isys->adev->auxdev.dev;
	int ret, tout;

	reinit_completion(&stream->stream_close_completion);

	ret = ipu7_fw_isys_simple_cmd(av->isys, stream->stream_handle,
				      IPU_INSYS_SEND_TYPE_STREAM_CLOSE);
	if (ret < 0) {
		dev_err(dev, "can't close stream (%d)\n", ret);
		return;
	}

	tout = wait_for_completion_timeout(&stream->stream_close_completion,
					   FW_CALL_TIMEOUT_JIFFIES);
	if (!tout)
		dev_err(dev, "stream close time out\n");
	else if (stream->error)
		dev_err(dev, "stream close error: %d\n", stream->error);
	else
		dev_dbg(dev, "close stream: complete\n");

	put_stream_opened(av);
}

int ipu7_isys_video_prepare_streaming(struct ipu7_isys_video *av)
{
	struct device *dev = &av->isys->adev->auxdev.dev;
	struct ipu7_isys_stream *stream = av->stream;
	int ret;

	WARN_ON(stream->nr_streaming);
	stream->nr_queues = 0;
	stream->external = NULL;
	atomic_set(&stream->sequence, 0);
	atomic_set(&stream->buf_id, 0);

	stream->asd = NULL;
	stream->seq_index = 0;
	memset(stream->seq, 0, sizeof(stream->seq));

	WARN_ON(!list_empty(&stream->queues));

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
	ret = media_pipeline_start(&av->vdev.entity, &stream->pipe);
#else
	ret = media_pipeline_start(av->vdev.entity.pads, &stream->pipe);
#endif
	if (ret < 0) {
		dev_dbg(dev, "pipeline start failed\n");
		return ret;
	}

	if (!stream->external) {
		dev_err(dev, "no external entity set! Driver bug?\n");
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
		media_pipeline_stop(&av->vdev.entity);
#else
		media_pipeline_stop(av->vdev.entity.pads);
#endif
		return -EINVAL;
	}

	dev_dbg(dev, "prepare stream: external entity %s\n",
		stream->external->entity->name);

	return 0;
}

int ipu7_isys_video_set_streaming(struct ipu7_isys_video *av,
				  unsigned int state,
				  struct ipu7_isys_buffer_list *bl)
{
	struct device *dev = &av->isys->adev->auxdev.dev;
	struct media_device *mdev = av->vdev.entity.graph_obj.mdev;
	struct media_entity_enum entities;
	struct media_graph graph;

	struct media_entity *entity, *entity2;
	struct ipu7_isys_stream *stream  = av->stream;
	struct v4l2_subdev *sd, *esd;
	int ret = 0;

	dev_dbg(dev, "set stream: %d\n", state);

	if (!stream->external->entity) {
		WARN_ON(1);
		return -ENODEV;
	}
	esd = media_entity_to_v4l2_subdev(stream->external->entity);

	if (!state) {
		stop_streaming_firmware(av);

		/* stop external sub-device now. */
		dev_info(dev, "stream off %s\n",
			 stream->external->entity->name);

		v4l2_subdev_call(esd, video, s_stream, state);
	}

	ret = media_graph_walk_init(&graph, mdev);
	if (ret)
		return ret;

	ret = media_entity_enum_init(&entities, mdev);
	if (ret)
		goto out_media_entity_graph_init;

	mutex_lock(&mdev->graph_mutex);

	media_graph_walk_start(&graph, &av->vdev.entity);

	while ((entity = media_graph_walk_next(&graph))) {
		sd = media_entity_to_v4l2_subdev(entity);

		/* Non-subdev nodes can be safely ignored here. */
		if (!is_media_entity_v4l2_subdev(entity))
			continue;

		/* Don't start truly external devices quite yet. */
		if (strncmp(sd->name, IPU_ISYS_ENTITY_PREFIX,
			    strlen(IPU_ISYS_ENTITY_PREFIX)) != 0 ||
		    stream->external->entity == entity)
			continue;

		dev_dbg(dev, "s_stream %s entity %s\n", state ? "on" : "off",
			entity->name);
		ret = v4l2_subdev_call(sd, video, s_stream, state);
		if (!state)
			continue;
		if (ret && ret != -ENOIOCTLCMD) {
			mutex_unlock(&mdev->graph_mutex);
			goto out_media_entity_stop_streaming;
		}

		media_entity_enum_set(&entities, entity);
	}

	mutex_unlock(&mdev->graph_mutex);

	/* Oh crap */
	if (state) {
		ret = start_stream_firmware(av, bl);
		if (ret)
			goto out_media_entity_stop_streaming;

		dev_dbg(dev, "set stream: source %d, stream_handle %d\n",
			stream->stream_source, stream->stream_handle);

		/* Start external sub-device now. */
		dev_info(dev, "stream on %s\n", stream->external->entity->name);

		ret = v4l2_subdev_call(esd, video, s_stream, state);
		if (ret)
			goto out_media_entity_stop_streaming_firmware;
	} else {
		close_streaming_firmware(av);
	}

	media_entity_enum_cleanup(&entities);
	media_graph_walk_cleanup(&graph);
	av->streaming = state;

	return 0;

out_media_entity_stop_streaming_firmware:
	stop_streaming_firmware(av);

out_media_entity_stop_streaming:
	mutex_lock(&mdev->graph_mutex);

	media_graph_walk_start(&graph, &av->vdev.entity);

	while (state && (entity2 = media_graph_walk_next(&graph)) &&
	       entity2 != entity) {
		sd = media_entity_to_v4l2_subdev(entity2);

		if (!media_entity_enum_test(&entities, entity2))
			continue;

		v4l2_subdev_call(sd, video, s_stream, 0);
	}

	mutex_unlock(&mdev->graph_mutex);

	media_entity_enum_cleanup(&entities);

out_media_entity_graph_init:
	media_graph_walk_cleanup(&graph);

	return ret;
}

void ipu7_isys_put_stream(struct ipu7_isys_stream *stream)
{
	unsigned int i;
	unsigned long flags;

	if (WARN_ON_ONCE(!stream))
		return;

	spin_lock_irqsave(&stream->isys->streams_lock, flags);
	for (i = 0; i < IPU_ISYS_MAX_STREAMS; i++) {
		if (&stream->isys->streams[i] == stream) {
			if (stream->isys->streams_ref_count[i] > 0) {
				stream->isys->streams_ref_count[i]--;
			} else {
				dev_warn(&stream->isys->adev->auxdev.dev,
					 "stream %d isn't used\n", i);
			}
			break;
		}
	}
	spin_unlock_irqrestore(&stream->isys->streams_lock, flags);
}

struct ipu7_isys_stream *ipu7_isys_get_stream(struct ipu7_isys *isys)
{
	unsigned int i;
	unsigned long flags;
	struct ipu7_isys_stream *stream = NULL;

	if (!isys)
		return NULL;

	spin_lock_irqsave(&isys->streams_lock, flags);
	for (i = 0; i < IPU_ISYS_MAX_STREAMS; i++) {
		if (!isys->streams_ref_count[i]) {
			isys->streams_ref_count[i]++;
			stream = &isys->streams[i];
			break;
		}
	}
	spin_unlock_irqrestore(&isys->streams_lock, flags);

	return stream;
}

struct ipu7_isys_stream *
ipu7_isys_query_stream_by_handle(struct ipu7_isys *isys,
				 u8 stream_handle)
{
	unsigned long flags;
	struct ipu7_isys_stream *stream = NULL;

	if (!isys)
		return NULL;

	if (stream_handle >= IPU_ISYS_MAX_STREAMS) {
		dev_err(&isys->adev->auxdev.dev,
			"stream_handle %d is invalid\n", stream_handle);
		return NULL;
	}

	spin_lock_irqsave(&isys->streams_lock, flags);
	if (isys->streams_ref_count[stream_handle] > 0) {
		isys->streams_ref_count[stream_handle]++;
		stream = &isys->streams[stream_handle];
	}
	spin_unlock_irqrestore(&isys->streams_lock, flags);

	return stream;
}

static const struct v4l2_ioctl_ops ipu7_v4l2_ioctl_ops = {
	.vidioc_querycap = ipu7_isys_vidioc_querycap,
	.vidioc_enum_fmt_vid_cap = ipu7_isys_vidioc_enum_fmt,
	.vidioc_enum_framesizes = ipu7_isys_vidioc_enum_framesizes,
	.vidioc_g_fmt_vid_cap = ipu7_isys_vidioc_g_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap = ipu7_isys_vidioc_s_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap = ipu7_isys_vidioc_try_fmt_vid_cap,
	.vidioc_reqbufs = ipu7_isys_vidioc_reqbufs,
	.vidioc_create_bufs = ipu7_isys_vidioc_create_bufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
	.vidioc_expbuf = vb2_ioctl_expbuf,
};

static const struct media_entity_operations entity_ops = {
	.link_validate = link_validate,
};

static const struct v4l2_file_operations isys_fops = {
	.owner = THIS_MODULE,
	.poll = vb2_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = vb2_fop_mmap,
	.open = video_open,
	.release = vb2_fop_release,
};

int ipu7_isys_fw_open(struct ipu7_isys *isys)
{
	struct ipu7_bus_device *adev = isys->adev;
	int ret;

	ret = pm_runtime_resume_and_get(&adev->auxdev.dev);
	if (ret < 0)
		return ret;

	mutex_lock(&isys->mutex);

	if (isys->ref_count++)
		goto unlock;

	/*
	 * Buffers could have been left to wrong queue at last closure.
	 * Move them now back to empty buffer queue.
	 */
	ipu7_cleanup_fw_msg_bufs(isys);

	ret = ipu7_fw_isys_open(isys);
	if (ret < 0)
		goto out;

unlock:
	mutex_unlock(&isys->mutex);

	return 0;
out:
	isys->ref_count--;
	mutex_unlock(&isys->mutex);
	pm_runtime_put(&adev->auxdev.dev);

	return ret;
}

void ipu7_isys_fw_close(struct ipu7_isys *isys)
{
	mutex_lock(&isys->mutex);

	isys->ref_count--;

	if (!isys->ref_count)
		ipu7_fw_isys_close(isys);

	mutex_unlock(&isys->mutex);

	if (isys->need_reset)
		pm_runtime_put_sync(&isys->adev->auxdev.dev);
	else
		pm_runtime_put(&isys->adev->auxdev.dev);
}

/*
 * Do everything that's needed to initialise things related to video
 * buffer queue, video node, and the related media entity. The caller
 * is expected to assign isys field and set the name of the video
 * device.
 */
int ipu7_isys_video_init(struct ipu7_isys_video *av)
{
	struct v4l2_format format = {
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
		.fmt.pix = {
			.width = 1920,
			.height = 1080,
		},
	};
	int ret;

	mutex_init(&av->mutex);
	av->vdev.device_caps = V4L2_CAP_STREAMING | V4L2_CAP_IO_MC |
		V4L2_CAP_VIDEO_CAPTURE;
	av->vdev.vfl_dir = VFL_DIR_RX;

	ret = ipu7_isys_queue_init(&av->aq);
	if (ret)
		goto out_mutex_destroy;

	av->pad.flags = MEDIA_PAD_FL_SINK | MEDIA_PAD_FL_MUST_CONNECT;
	ret = media_entity_pads_init(&av->vdev.entity, 1, &av->pad);
	if (ret)
		goto out_vb2_queue_cleanup;

	av->vdev.entity.ops = &entity_ops;
	av->vdev.release = video_device_release_empty;
	av->vdev.fops = &isys_fops;
	av->vdev.v4l2_dev = &av->isys->v4l2_dev;
	av->vdev.ioctl_ops = &ipu7_v4l2_ioctl_ops;
	av->vdev.queue = &av->aq.vbq;
	av->vdev.lock = &av->mutex;

	__ipu_isys_vidioc_try_fmt_vid_cap(av, &format);
	av->pix_fmt = format.fmt.pix;

	set_bit(V4L2_FL_USES_V4L2_FH, &av->vdev.flags);
	video_set_drvdata(&av->vdev, av);

	ret = video_register_device(&av->vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto out_media_entity_cleanup;

	return ret;

out_media_entity_cleanup:
	media_entity_cleanup(&av->vdev.entity);

out_vb2_queue_cleanup:
	vb2_queue_release(&av->aq.vbq);

out_mutex_destroy:
	mutex_destroy(&av->mutex);

	return ret;
}

void ipu7_isys_video_cleanup(struct ipu7_isys_video *av)
{
	vb2_video_unregister_device(&av->vdev);
	media_entity_cleanup(&av->vdev.entity);
	mutex_destroy(&av->mutex);
}
