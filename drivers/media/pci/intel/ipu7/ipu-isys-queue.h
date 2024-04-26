/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2013 - 2024 Intel Corporation */

#ifndef IPU_ISYS_QUEUE_H
#define IPU_ISYS_QUEUE_H

#include <linux/list.h>
#include <linux/spinlock.h>

#include <media/videobuf2-v4l2.h>

#include "ipu-isys-media.h"

struct ipu_isys_video;
struct ipu_isys_stream;
struct ipu_insys_resp;
struct ipu_insys_buffset;

struct ipu_isys_queue {
	struct list_head node;	/* struct ipu_isys_stream.queues */
	struct vb2_queue vbq;
	struct device *dev;
	/*
	 * @lock: serialise access to queued and pre_streamon_queued
	 */
	spinlock_t lock;
	struct list_head active;
	struct list_head incoming;
	unsigned int fw_output;
	void (*fill_frame_buff_set_pin)(struct vb2_buffer *vb,
					struct ipu_insys_buffset *set);
	int (*link_fmt_validate)(struct ipu_isys_queue *aq);
};

struct ipu_isys_buffer {
	struct list_head head;
	atomic_t str2mmio_flag;
};

struct ipu_isys_video_buffer {
	struct vb2_v4l2_buffer vb_v4l2;
	struct ipu_isys_buffer ib;
};

#define IPU_ISYS_BUFFER_LIST_FL_INCOMING	BIT(0)
#define IPU_ISYS_BUFFER_LIST_FL_ACTIVE	BIT(1)
#define IPU_ISYS_BUFFER_LIST_FL_SET_STATE	BIT(2)

struct ipu_isys_buffer_list {
	struct list_head head;
	unsigned int nbufs;
};

#define vb2_queue_to_ipu_isys_queue(__vb2) \
	container_of(__vb2, struct ipu_isys_queue, vbq)

#define ipu_isys_to_isys_video_buffer(__ib) \
	container_of(__ib, struct ipu_isys_video_buffer, ib)

#define vb2_buffer_to_ipu_isys_video_buffer(__vb) \
	container_of(to_vb2_v4l2_buffer(__vb), \
	struct ipu_isys_video_buffer, vb_v4l2)

#define ipu_isys_buffer_to_vb2_buffer(__ib) \
	(&ipu_isys_to_isys_video_buffer(__ib)->vb_v4l2.vb2_buf)

#define vb2_buffer_to_ipu_isys_buffer(__vb) \
	(&vb2_buffer_to_ipu_isys_video_buffer(__vb)->ib)

struct ipu_isys_request {
	struct media_device_request req;
	/* serialise access to buffers */
	spinlock_t lock;
	struct list_head buffers;	/* struct ipu_isys_buffer.head */
	bool dispatched;
	/*
	 * struct ipu_isys.requests;
	 * struct ipu_isys_stream.struct.*
	 */
	struct list_head head;
};

#define to_ipu_isys_request(__req) \
	container_of(__req, struct ipu_isys_request, req)

void ipu_isys_buffer_list_queue(struct ipu_isys_buffer_list *bl,
				unsigned long op_flags,
				enum vb2_buffer_state state);
void
ipu_isys_buffer_to_fw_frame_buff_pin(struct vb2_buffer *vb,
				     struct ipu_insys_buffset *set);
void
ipu_isys_buffer_to_fw_frame_buff(struct ipu_insys_buffset *set,
				 struct ipu_isys_stream *stream,
				 struct ipu_isys_buffer_list *bl,
				 u8 enable_stream_start_ack);
int ipu_isys_link_fmt_validate(struct ipu_isys_queue *aq);

void
ipu_isys_buf_calc_sequence_time(struct ipu_isys_buffer *ib,
				struct ipu_insys_resp *info);
void ipu_isys_queue_buf_done(struct ipu_isys_buffer *ib);
void ipu_isys_queue_buf_ready(struct ipu_isys_stream *stream,
			      struct ipu_insys_resp *info);
int ipu_isys_queue_init(struct ipu_isys_queue *aq);
#endif /* IPU_ISYS_QUEUE_H */
