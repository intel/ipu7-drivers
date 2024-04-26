// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2013 - 2024 Intel Corporation

#include <linux/bits.h>
#include <linux/compat.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/init_task.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/version.h>

#include <uapi/linux/sched/types.h>

#include <media/media-entity.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mc.h>

#include "ipu.h"
#include "ipu-bus.h"
#include "ipu-cpd.h"
#include "ipu-isys.h"
#include "ipu-isys-video.h"
#include "ipu-platform.h"
#include "ipu-platform-regs.h"
#include "ipu-buttress-regs.h"
#include "ipu-fw-isys.h"
#include "ipu-syscom.h"

const struct ipu_isys_pixelformat ipu7_isys_pfmts[] = {
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
	struct ipu_isys_video *av = video_drvdata(file);
	struct ipu_isys *isys = av->isys;
	int ret;

	mutex_lock(&isys->mutex);

	if (isys->reset_needed) {
		mutex_unlock(&isys->mutex);
		dev_warn(&isys->adev->dev, "isys power cycle required\n");
		return -EIO;
	}
	mutex_unlock(&isys->mutex);

	ret = pm_runtime_get_sync(&isys->adev->dev);
	if (ret < 0) {
		pm_runtime_put_noidle(&isys->adev->dev);
		return ret;
	}

	ret = v4l2_fh_open(file);
	if (ret)
		goto out_power_down;

	ret = v4l2_pipeline_pm_get(&av->vdev.entity);
	if (ret)
		goto out_v4l2_fh_release;

	mutex_lock(&isys->mutex);

	if (isys->video_opened++) {
		/* Already open */
		mutex_unlock(&isys->mutex);
		return 0;
	}

	/*
	 * Buffers could have been left to wrong queue at last closure.
	 * Move them now back to empty buffer queue.
	 */
	ipu_cleanup_fw_msg_bufs(isys);

	// TODO: Need to release resource from previous session?

	/* TODO: move the fw open into isys resume */
	ret = ipu_fw_isys_open(av->isys);
	if (ret < 0)
		goto out_fw_close;

	mutex_unlock(&isys->mutex);

	return 0;

out_fw_close:
	ipu_fw_isys_close(av->isys);
	isys->video_opened--;
	mutex_unlock(&isys->mutex);
	v4l2_pipeline_pm_put(&av->vdev.entity);

out_v4l2_fh_release:
	v4l2_fh_release(file);
out_power_down:
	pm_runtime_put(&isys->adev->dev);

	return ret;
}

static int video_release(struct file *file)
{
	struct ipu_isys_video *av = video_drvdata(file);
	int ret = 0;

	vb2_fop_release(file);

	mutex_lock(&av->isys->mutex);

	if (!--av->isys->video_opened) {
		ipu_fw_isys_close(av->isys);
	}

	mutex_unlock(&av->isys->mutex);

	v4l2_pipeline_pm_put(&av->vdev.entity);

	if (av->isys->reset_needed)
		pm_runtime_put_sync(&av->isys->adev->dev);
	else
		pm_runtime_put(&av->isys->adev->dev);

	return ret;
}

static const struct ipu_isys_pixelformat *
ipu_isys_get_pixelformat(struct ipu_isys_video *av, u32 pixelformat)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ipu7_isys_pfmts); i++) {
		const struct ipu_isys_pixelformat *pfmt = &ipu7_isys_pfmts[i];

		if (pfmt->pixelformat == pixelformat)
			return pfmt;
	}

	return &ipu7_isys_pfmts[0];
}

int ipu_isys_vidioc_querycap(struct file *file, void *fh,
			     struct v4l2_capability *cap)
{
	struct ipu_isys_video *av = video_drvdata(file);

	strscpy(cap->driver, IPU_ISYS_NAME, sizeof(cap->driver));
	strscpy(cap->card, av->isys->media_dev.model, sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "PCI:%s",
		 av->isys->media_dev.bus_info);
	return 0;
}

int ipu_isys_vidioc_enum_fmt(struct file *file, void *fh,
			     struct v4l2_fmtdesc *f)
{
	unsigned int i, found = 0;

	if (f->index >= ARRAY_SIZE(ipu7_isys_pfmts))
		return -EINVAL;

	if (!f->mbus_code) {
		f->flags = 0;
		f->pixelformat = ipu7_isys_pfmts[f->index].pixelformat;
		return 0;
	}

	for (i = 0; i < ARRAY_SIZE(ipu7_isys_pfmts); i++) {
		if (f->mbus_code != ipu7_isys_pfmts[i].code)
			continue;

		if (f->index == found) {
			f->flags = 0;
			f->pixelformat = ipu7_isys_pfmts[i].pixelformat;
			return 0;
		}
		found++;
	}

	return -EINVAL;
}

static int vidioc_g_fmt_vid_cap_mplane(struct file *file, void *fh,
				       struct v4l2_format *fmt)
{
	struct ipu_isys_video *av = video_drvdata(file);

	fmt->fmt.pix_mp = av->mpix;

	return 0;
}

static const struct ipu_isys_pixelformat *
ipu_isys_video_try_fmt_vid_mplane(struct ipu_isys_video *av,
				  struct v4l2_pix_format_mplane *mpix)
{
	const struct ipu_isys_pixelformat *pfmt =
	    ipu_isys_get_pixelformat(av, mpix->pixelformat);

	if (!pfmt)
		return NULL;

	mpix->pixelformat = pfmt->pixelformat;
	mpix->num_planes = 1;

	mpix->width = clamp(mpix->width, IPU_ISYS_MIN_WIDTH,
			    IPU_ISYS_MAX_WIDTH);
	mpix->height = clamp(mpix->height, IPU_ISYS_MIN_HEIGHT,
			     IPU_ISYS_MAX_HEIGHT);

	if (pfmt->bpp != pfmt->bpp_packed)
		mpix->plane_fmt[0].bytesperline =
			mpix->width * DIV_ROUND_UP(pfmt->bpp, BITS_PER_BYTE);
	else
		mpix->plane_fmt[0].bytesperline =
		    DIV_ROUND_UP(mpix->width * pfmt->bpp, BITS_PER_BYTE);

	mpix->plane_fmt[0].bytesperline = ALIGN(mpix->plane_fmt[0].bytesperline,
						av->isys->line_align);

	/*
	 * (height + 1) * bytesperline due to a hardware issue: the DMA unit
	 * is a power of two, and a line should be transferred as few units
	 * as possible. The result is that up to line length more data than
	 * the image size may be transferred to memory after the image.
	 * Another limition is the GDA allocation unit size. For low
	 * resolution it gives a bigger number. Use larger one to avoid
	 * memory corruption.
	 */
	mpix->plane_fmt[0].sizeimage =
		max(max(mpix->plane_fmt[0].sizeimage,
			mpix->plane_fmt[0].bytesperline * mpix->height +
			max(mpix->plane_fmt[0].bytesperline,
			    av->isys->pdata->ipdata->isys_dma_overshoot)), 1U);

	memset(mpix->plane_fmt[0].reserved, 0,
	       sizeof(mpix->plane_fmt[0].reserved));

	if (mpix->field == V4L2_FIELD_ANY)
		mpix->field = V4L2_FIELD_NONE;
	/* Use defaults */
	mpix->colorspace = V4L2_COLORSPACE_RAW;
	mpix->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	mpix->quantization = V4L2_QUANTIZATION_DEFAULT;
	mpix->xfer_func = V4L2_XFER_FUNC_DEFAULT;

	return pfmt;
}

static int vidioc_s_fmt_vid_cap_mplane(struct file *file, void *fh,
				       struct v4l2_format *f)
{
	struct ipu_isys_video *av = video_drvdata(file);

	if (av->aq.vbq.streaming)
		return -EBUSY;

	av->pfmt = ipu_isys_video_try_fmt_vid_mplane(av, &f->fmt.pix_mp);
	av->mpix = f->fmt.pix_mp;

	return 0;
}

static int vidioc_try_fmt_vid_cap_mplane(struct file *file, void *fh,
					 struct v4l2_format *f)
{
	struct ipu_isys_video *av = video_drvdata(file);

	ipu_isys_video_try_fmt_vid_mplane(av, &f->fmt.pix_mp);

	return 0;
}

static int vidioc_enum_input(struct file *file, void *fh,
			     struct v4l2_input *input)
{
	if (input->index > 0)
		return -EINVAL;
	strscpy(input->name, "camera", sizeof(input->name));
	input->type = V4L2_INPUT_TYPE_CAMERA;

	return 0;
}

static int vidioc_g_input(struct file *file, void *fh, unsigned int *input)
{
	*input = 0;

	return 0;
}

static int vidioc_s_input(struct file *file, void *fh, unsigned int input)
{
	return input == 0 ? 0 : -EINVAL;
}

/*
 * Return true if an entity directly connected to an Iunit entity is
 * an image source for the ISP. This can be any external directly
 * connected entity or any of the test pattern generators in the
 * Iunit.
 */
static bool is_external(struct ipu_isys_video *av, struct media_entity *entity)
{
	struct v4l2_subdev *sd;
#ifdef CONFIG_VIDEO_INTEL_IPU_MGC
	unsigned int i;
#endif

	/* All video nodes are ours. */
	if (!is_media_entity_v4l2_subdev(entity))
		return false;

	sd = media_entity_to_v4l2_subdev(entity);
	if (strncmp(sd->name, IPU_ISYS_ENTITY_PREFIX,
		    strlen(IPU_ISYS_ENTITY_PREFIX)) != 0)
		return true;

#ifdef CONFIG_VIDEO_INTEL_IPU_MGC
	for (i = 0; i < av->isys->pdata->ipdata->tpg.ntpgs &&
	     av->isys->tpg[i].isys; i++)
		if (entity == &av->isys->tpg[i].asd.sd.entity)
			return true;
#endif

	return false;
}

static int link_validate(struct media_link *link)
{
	struct ipu_isys_video *av =
	    container_of(link->sink, struct ipu_isys_video, pad);
	/* All sub-devices connected to a video node are ours. */
	struct ipu_isys_stream *stream = av->stream;
	struct v4l2_subdev *sd;

	if (!stream || !link->source->entity)
		return -EINVAL;

	sd = media_entity_to_v4l2_subdev(link->source->entity);
	if (is_external(av, link->source->entity)) {
		stream->external =
			media_pad_remote_pad_first(av->vdev.entity.pads);
		stream->source = to_ipu_isys_subdev(sd)->source;
	}

	return 0;
}

static void get_stream_opened(struct ipu_isys_video *av)
{
	unsigned long flags;

	spin_lock_irqsave(&av->isys->streams_lock, flags);
	av->isys->stream_opened++;
	spin_unlock_irqrestore(&av->isys->streams_lock, flags);
}

static void put_stream_opened(struct ipu_isys_video *av)
{
	unsigned long flags;

	spin_lock_irqsave(&av->isys->streams_lock, flags);
	av->isys->stream_opened--;
	spin_unlock_irqrestore(&av->isys->streams_lock, flags);
}

static int get_external_facing_format(struct ipu_isys_stream *stream,
				      struct v4l2_subdev_format *format)
{
	struct v4l2_subdev *sd;
	struct media_pad *external_facing;

	if (!stream->external->entity) {
		WARN_ON(1);
		return -ENODEV;
	}
	sd = media_entity_to_v4l2_subdev(stream->external->entity);
	external_facing = (strncmp(sd->name, IPU_ISYS_ENTITY_PREFIX,
			   strlen(IPU_ISYS_ENTITY_PREFIX)) == 0) ?
			   stream->external :
			   media_pad_remote_pad_first(stream->external);
	if (WARN_ON(!external_facing)) {
		dev_warn(&stream->isys->adev->dev,
			 "no external facing pad --- driver bug?\n");
		return -EINVAL;
	}

	format->which = V4L2_SUBDEV_FORMAT_ACTIVE;
	format->pad = 0;
	sd = media_entity_to_v4l2_subdev(external_facing->entity);

	return v4l2_subdev_call(sd, pad, get_fmt, NULL, format);
}

void
ipu_isys_prepare_fw_cfg_default(struct ipu_isys_video *av,
				struct ipu_insys_stream_cfg *cfg)
{
	struct ipu_isys_stream *stream = av->stream;
	struct ipu_isys_queue *aq = &av->aq;
	struct ipu_insys_output_pin *pin_info;
	int pin = cfg->nof_output_pins++;

	aq->fw_output = pin;
	stream->output_pins[pin].pin_ready = ipu_isys_queue_buf_ready;
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
	pin_info->ft = (u16)av->pfmt->css_pixelformat;

	/* stride in bytes */
	pin_info->stride = av->mpix.plane_fmt[0].bytesperline;
	pin_info->send_irq = 1;
	pin_info->early_ack_en = 0;

	/* input pin id */
	pin_info->input_pin_id = 0;

	cfg->vc = 0;
}

/* Create stream and start it using the CSS FW ABI. */
static int start_stream_firmware(struct ipu_isys_video *av,
				 struct ipu_isys_buffer_list *bl)
{
	struct ipu_isys_stream *stream = av->stream;
	struct device *dev = &av->isys->adev->dev;
	struct v4l2_subdev_selection sel_fmt = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
		.target = V4L2_SEL_TGT_CROP,
		.pad = CSI2_BE_PAD_SOURCE,
	};
	struct ipu_insys_stream_cfg *stream_cfg;
	struct isys_fw_msgs *msg = NULL;
	struct ipu_insys_buffset *buf = NULL;
	struct ipu_isys_queue *aq;
	struct v4l2_subdev_format source_fmt = { 0 };
	struct v4l2_subdev *be_sd = NULL;
	struct media_pad *source_pad = media_pad_remote_pad_first(&av->pad);
	enum ipu_insys_send_type send_type;
	int ret, retout, tout;

	ret = get_external_facing_format(stream, &source_fmt);
	if (ret)
		return ret;

	msg = ipu_get_fw_msg_buf(stream);
	if (!msg)
		return -ENOMEM;

	stream_cfg = to_stream_cfg_msg_buf(msg);
	stream_cfg->input_pins[0].input_res.width = source_fmt.format.width;
	stream_cfg->input_pins[0].input_res.height = source_fmt.format.height;
	stream_cfg->input_pins[0].dt =
	    ipu_isys_mbus_code_to_data_type(source_fmt.format.code);
	/* It is similar with mipi_store_mod */
	stream_cfg->input_pins[0].disable_mipi_unpacking = 0;
	if (av->pfmt->bpp == av->pfmt->bpp_packed &&
	    av->pfmt->bpp % BITS_PER_BYTE)
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
	stream_cfg->port_id = stream->source;
	stream_cfg->vc = 0;
	stream_cfg->nof_input_pins = 1;

	/*
	 * Only CSI2-BE SOC has the capability to do crop,
	 * so get the crop info from csi2-be or csi2-be-soc.
	 */
	if (stream->csi2_be_soc) {
		be_sd = &stream->csi2_be_soc->asd.sd;
		if (source_pad)
			sel_fmt.pad = source_pad->index;
	}

	list_for_each_entry(aq, &stream->queues, node) {
		struct ipu_isys_video *__av = ipu_isys_queue_to_video(aq);

		ipu_isys_prepare_fw_cfg_default(__av, stream_cfg);
	}

	ipu_fw_isys_dump_stream_cfg(dev, stream_cfg);

	stream->nr_output_pins = stream_cfg->nof_output_pins;

	reinit_completion(&stream->stream_open_completion);

	ret = ipu_fw_isys_complex_cmd(av->isys, stream->stream_handle,
				      stream_cfg, to_dma_addr(msg),
				      sizeof(*stream_cfg),
				      IPU_INSYS_SEND_TYPE_STREAM_OPEN);
	if (ret < 0) {
		dev_err(dev, "can't open stream (%d)\n", ret);
		ipu_put_fw_msg_buf(av->isys, (uintptr_t)stream_cfg);
		return ret;
	}

	get_stream_opened(av);

	tout = wait_for_completion_timeout(&stream->stream_open_completion,
					   IPU_LIB_CALL_TIMEOUT_JIFFIES);

	ipu_put_fw_msg_buf(av->isys, (uintptr_t)stream_cfg);

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

	msg = ipu_get_fw_msg_buf(stream);
	if (!msg) {
		ret = -ENOMEM;
		goto out_put_stream_opened;
	}
	buf = to_frame_msg_buf(msg);

	ipu_isys_buffer_to_fw_frame_buff(buf, stream, bl, 1);
	ipu_isys_buffer_list_queue(bl, IPU_ISYS_BUFFER_LIST_FL_ACTIVE, 0);

	reinit_completion(&stream->stream_start_completion);

	send_type = IPU_INSYS_SEND_TYPE_STREAM_START_AND_CAPTURE;
	ipu_fw_isys_dump_frame_buff_set(dev, buf,
					stream_cfg->nof_output_pins);
	ret = ipu_fw_isys_complex_cmd(av->isys, stream->stream_handle, buf,
				      to_dma_addr(msg), sizeof(*buf),
				      send_type);
	if (ret < 0) {
		dev_err(dev, "can't start streaming (%d)\n", ret);
		goto out_stream_close;
	}

	tout = wait_for_completion_timeout(&stream->stream_start_completion,
					   IPU_LIB_CALL_TIMEOUT_JIFFIES);
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

	retout = ipu_fw_isys_simple_cmd(av->isys, stream->stream_handle,
					IPU_INSYS_SEND_TYPE_STREAM_CLOSE);
	if (retout < 0) {
		dev_dbg(dev, "can't close stream (%d)\n", retout);
		goto out_put_stream_opened;
	}

	tout = wait_for_completion_timeout(&stream->stream_close_completion,
					   IPU_LIB_CALL_TIMEOUT_JIFFIES);
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

static void stop_streaming_firmware(struct ipu_isys_video *av)
{
	struct ipu_isys_stream *stream = av->stream;
	struct device *dev = &av->isys->adev->dev;
	int ret, tout;
	enum ipu_insys_send_type send_type =
		IPU_INSYS_SEND_TYPE_STREAM_FLUSH;

	reinit_completion(&stream->stream_stop_completion);

	ret = ipu_fw_isys_simple_cmd(av->isys, stream->stream_handle,
				     send_type);

	if (ret < 0) {
		dev_err(dev, "can't stop stream (%d)\n", ret);
		return;
	}

	tout = wait_for_completion_timeout(&stream->stream_stop_completion,
					   IPU_LIB_CALL_TIMEOUT_JIFFIES);
	if (!tout)
		dev_err(dev, "stream stop time out\n");
	else if (stream->error)
		dev_err(dev, "stream stop error: %d\n", stream->error);
	else
		dev_dbg(dev, "stop stream: complete\n");
}

static void close_streaming_firmware(struct ipu_isys_video *av)
{
	struct ipu_isys_stream *stream =  av->stream;
	struct device *dev = &av->isys->adev->dev;
	int ret, tout;

	reinit_completion(&stream->stream_close_completion);

	ret = ipu_fw_isys_simple_cmd(av->isys, stream->stream_handle,
				     IPU_INSYS_SEND_TYPE_STREAM_CLOSE);
	if (ret < 0) {
		dev_err(dev, "can't close stream (%d)\n", ret);
		return;
	}

	tout = wait_for_completion_timeout(&stream->stream_close_completion,
					   IPU_LIB_CALL_TIMEOUT_JIFFIES);
	if (!tout)
		dev_err(dev, "stream close time out\n");
	else if (stream->error)
		dev_err(dev, "stream close error: %d\n", stream->error);
	else
		dev_dbg(dev, "close stream: complete\n");

	put_stream_opened(av);
}

int ipu_isys_video_prepare_streaming(struct ipu_isys_video *av)
{
	struct ipu_isys *isys = av->isys;
	struct device *dev = &isys->adev->dev;
	struct ipu_isys_stream *stream = av->stream;
	int ret;

	WARN_ON(stream->nr_streaming);
	stream->nr_queues = 0;
	stream->external = NULL;
	atomic_set(&stream->sequence, 0);
	atomic_set(&stream->buf_id, 0);

	stream->csi2_be_soc = NULL;
	stream->csi2 = NULL;
#ifdef CONFIG_VIDEO_INTEL_IPU_MGC
	stream->tpg = NULL;
#endif
	stream->seq_index = 0;
	memset(stream->seq, 0, sizeof(stream->seq));

	WARN_ON(!list_empty(&stream->queues));

	ret = media_pipeline_start(av->vdev.entity.pads, &stream->pipe);
	if (ret < 0) {
		dev_dbg(dev, "pipeline start failed\n");
		return ret;
	}

	if (!stream->external) {
		dev_err(dev, "no external entity set! Driver bug?\n");
		media_pipeline_stop(av->vdev.entity.pads);
		return -EINVAL;
	}

	dev_dbg(dev, "prepare stream: external entity %s\n",
		stream->external->entity->name);

	return 0;
}

int ipu_isys_video_set_streaming(struct ipu_isys_video *av,
				 unsigned int state,
				 struct ipu_isys_buffer_list *bl)
{
	struct device *dev = &av->isys->adev->dev;
	struct media_device *mdev = av->vdev.entity.graph_obj.mdev;
	struct media_entity_enum entities;
	struct media_graph graph;

	struct media_entity *entity, *entity2;
	struct ipu_isys_stream *stream  = av->stream;
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
			stream->source, stream->stream_handle);

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

void ipu_isys_put_stream(struct ipu_isys_stream *stream)
{
	unsigned int i;
	unsigned long flags;

	if (!stream) {
		dev_err(&stream->isys->adev->dev, "no available stream\n");
		return;
	}

	spin_lock_irqsave(&stream->isys->streams_lock, flags);
	for (i = 0; i < IPU_ISYS_MAX_STREAMS; i++) {
		if (&stream->isys->streams[i] == stream) {
			if (stream->isys->streams_ref_count[i] > 0) {
				stream->isys->streams_ref_count[i]--;
			} else {
				dev_warn(&stream->isys->adev->dev,
					 "stream %d isn't used\n", i);
			}
			break;
		}
	}
	spin_unlock_irqrestore(&stream->isys->streams_lock, flags);
}

struct ipu_isys_stream *ipu_isys_get_stream(struct ipu_isys *isys)
{
	unsigned int i;
	unsigned long flags;
	struct ipu_isys_stream *stream = NULL;

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

struct ipu_isys_stream *ipu_isys_query_stream_by_handle(struct ipu_isys *isys,
							u8 stream_handle)
{
	unsigned long flags;
	struct ipu_isys_stream *stream = NULL;

	if (!isys)
		return NULL;

	if (stream_handle >= IPU_ISYS_MAX_STREAMS) {
		dev_err(&isys->adev->dev,
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

#ifdef CONFIG_COMPAT
static long ipu_isys_compat_ioctl(struct file *file, unsigned int cmd,
				  unsigned long arg)
{
	long ret = -ENOIOCTLCMD;
	void __user *up = compat_ptr(arg);

	/*
	 * at present, there is not any private IOCTL need to compat handle
	 */
	if (file->f_op->unlocked_ioctl)
		ret = file->f_op->unlocked_ioctl(file, cmd, (unsigned long)up);

	return ret;
}
#endif

static const struct v4l2_ioctl_ops ioctl_ops_mplane = {
	.vidioc_querycap = ipu_isys_vidioc_querycap,
	.vidioc_enum_fmt_vid_cap = ipu_isys_vidioc_enum_fmt,
	.vidioc_g_fmt_vid_cap_mplane = vidioc_g_fmt_vid_cap_mplane,
	.vidioc_s_fmt_vid_cap_mplane = vidioc_s_fmt_vid_cap_mplane,
	.vidioc_try_fmt_vid_cap_mplane = vidioc_try_fmt_vid_cap_mplane,
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_enum_input = vidioc_enum_input,
	.vidioc_g_input = vidioc_g_input,
	.vidioc_s_input = vidioc_s_input,
};

static const struct media_entity_operations entity_ops = {
	.link_validate = link_validate,
};

static const struct v4l2_file_operations isys_fops = {
	.owner = THIS_MODULE,
	.poll = vb2_fop_poll,
	.unlocked_ioctl = video_ioctl2,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = ipu_isys_compat_ioctl,
#endif
	.mmap = vb2_fop_mmap,
	.open = video_open,
	.release = video_release,
};

/*
 * Do everything that's needed to initialise things related to video
 * buffer queue, video node, and the related media entity. The caller
 * is expected to assign isys field and set the name of the video
 * device.
 */
int ipu_isys_video_init(struct ipu_isys_video *av,
			struct media_entity *entity,
			unsigned int pad, unsigned long pad_flags,
			unsigned int flags)
{
	const struct v4l2_ioctl_ops *ioctl_ops = NULL;
	int ret;

	mutex_init(&av->mutex);
	av->vdev.device_caps = V4L2_CAP_STREAMING;
	if (pad_flags & MEDIA_PAD_FL_SINK) {
		av->aq.vbq.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
		ioctl_ops = &ioctl_ops_mplane;
		av->vdev.device_caps |= V4L2_CAP_VIDEO_CAPTURE_MPLANE;
		av->vdev.vfl_dir = VFL_DIR_RX;
	} else {
		av->aq.vbq.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
		av->vdev.vfl_dir = VFL_DIR_TX;
		av->vdev.device_caps |= V4L2_CAP_VIDEO_OUTPUT_MPLANE;
	}
	ret = ipu_isys_queue_init(&av->aq);
	if (ret)
		goto out_mutex_destroy;

	av->pad.flags = pad_flags | MEDIA_PAD_FL_MUST_CONNECT;
	ret = media_entity_pads_init(&av->vdev.entity, 1, &av->pad);
	if (ret)
		goto out_vb2_queue_cleanup;

	av->vdev.entity.ops = &entity_ops;
	av->vdev.release = video_device_release_empty;
	av->vdev.fops = &isys_fops;
	av->vdev.v4l2_dev = &av->isys->v4l2_dev;
	if (!av->vdev.ioctl_ops)
		av->vdev.ioctl_ops = ioctl_ops;
	av->vdev.queue = &av->aq.vbq;
	av->vdev.lock = &av->mutex;
	set_bit(V4L2_FL_USES_V4L2_FH, &av->vdev.flags);
	video_set_drvdata(&av->vdev, av);

	ret = video_register_device(&av->vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto out_media_entity_cleanup;

	if (pad_flags & MEDIA_PAD_FL_SINK)
		ret = media_create_pad_link(entity, pad,
					    &av->vdev.entity, 0, flags);
	else
		ret = media_create_pad_link(&av->vdev.entity, 0, entity,
					    pad, flags);
	if (ret) {
		dev_info(&av->isys->adev->dev, "can't create link\n");
		goto out_media_entity_cleanup;
	}

	av->pfmt = ipu_isys_video_try_fmt_vid_mplane(av, &av->mpix);

	return ret;

out_media_entity_cleanup:
	vb2_video_unregister_device(&av->vdev);
	media_entity_cleanup(&av->vdev.entity);

out_vb2_queue_cleanup:
	vb2_queue_release(&av->aq.vbq);

out_mutex_destroy:
	mutex_destroy(&av->mutex);

	return ret;
}

void ipu_isys_video_cleanup(struct ipu_isys_video *av)
{
	vb2_video_unregister_device(&av->vdev);
	media_entity_cleanup(&av->vdev.entity);
	mutex_destroy(&av->mutex);
}
