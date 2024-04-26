/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2013 - 2024 Intel Corporation */

#ifndef IPU_ISYS_VIDEO_H
#define IPU_ISYS_VIDEO_H

#include <linux/mutex.h>
#include <linux/list.h>
#include <linux/videodev2.h>
#include <media/media-entity.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#include "ipu-isys-queue.h"

#define IPU_INSYS_OUTPUT_PINS 11
#define IPU_ISYS_MAX_PARALLEL_SOF 2

struct ipu_isys;
struct ipu_isys_csi2_be_soc;
struct ipu_insys_stream_cfg;

struct ipu_isys_pixelformat {
	u32 pixelformat;
	u32 bpp;
	u32 bpp_packed;
	u32 code;
	u32 css_pixelformat;
};

struct sequence_info {
	unsigned int sequence;
	u64 timestamp;
};

struct output_pin_data {
	void (*pin_ready)(struct ipu_isys_stream *stream,
			  struct ipu_insys_resp *info);
	struct ipu_isys_queue *aq;
};

struct ipu_isys_stream {
	struct mutex mutex;
	struct media_pipeline pipe;
	struct media_pad *external;
	atomic_t sequence;
	atomic_t buf_id;
	unsigned int seq_index;
	struct sequence_info seq[IPU_ISYS_MAX_PARALLEL_SOF];
	int source;	/* SSI stream source */
	int stream_handle;	/* stream handle for CSS API */
	unsigned int nr_output_pins;	/* How many firmware pins? */
	struct ipu_isys_csi2_be_soc *csi2_be_soc;
	struct ipu_isys_csi2 *csi2;
#ifdef CONFIG_VIDEO_INTEL_IPU_MGC
	struct ipu_isys_tpg *tpg;
#endif

	/*
	 * Number of capture queues, write access serialised using struct
	 * ipu_isys.stream_mutex
	 */
	int nr_queues;
	int nr_streaming;	/* Number of capture queues streaming */
	int streaming;	/* Has streaming been really started? */
	struct list_head queues;
	struct completion stream_open_completion;
	struct completion stream_close_completion;
	struct completion stream_start_completion;
	struct completion stream_stop_completion;
	struct ipu_isys *isys;

	struct output_pin_data output_pins[IPU_INSYS_OUTPUT_PINS];
	int error;
};

#define to_ipu_isys_pipeline(__pipe)				\
	container_of((__pipe), struct ipu_isys_stream, pipe)

struct ipu_isys_video {
	/* Serialise access to other fields in the struct. */
	struct mutex mutex;
	struct media_pad pad;
	struct video_device vdev;
	struct v4l2_pix_format_mplane mpix;
	const struct ipu_isys_pixelformat *pfmt;
	struct ipu_isys_queue aq;
	struct ipu_isys *isys;
	struct ipu_isys_stream *stream;
	unsigned int streaming;
};

#define ipu_isys_queue_to_video(__aq) \
	container_of(__aq, struct ipu_isys_video, aq)

int ipu_isys_vidioc_querycap(struct file *file, void *fh,
			     struct v4l2_capability *cap);

int ipu_isys_vidioc_enum_fmt(struct file *file, void *fh,
			     struct v4l2_fmtdesc *f);
void ipu_isys_prepare_fw_cfg_default(struct ipu_isys_video *av,
				     struct ipu_insys_stream_cfg *cfg);
int ipu_isys_video_prepare_streaming(struct ipu_isys_video *av);
int ipu_isys_video_set_streaming(struct ipu_isys_video *av, unsigned int state,
				 struct ipu_isys_buffer_list *bl);
int ipu_isys_video_init(struct ipu_isys_video *av, struct media_entity *source,
			unsigned int source_pad, unsigned long pad_flags,
			unsigned int flags);
void ipu_isys_video_cleanup(struct ipu_isys_video *av);
void ipu_isys_put_stream(struct ipu_isys_stream *stream);
struct ipu_isys_stream *ipu_isys_get_stream(struct ipu_isys *isys);
struct ipu_isys_stream *ipu_isys_query_stream_by_handle(struct ipu_isys *isys,
							u8 stream_handle);
#endif /* IPU_ISYS_VIDEO_H */
