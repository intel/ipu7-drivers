// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2013 - 2024 Intel Corporation

#include <linux/compat.h>
#include <linux/errno.h>
#include <linux/uaccess.h>

#include <uapi/linux/ipu-psys.h>

#include "ipu-psys.h"

static long native_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret = -ENOTTY;

	if (file->f_op->unlocked_ioctl)
		ret = file->f_op->unlocked_ioctl(file, cmd, arg);

	return ret;
}

struct ipu_psys_buffer32 {
	u64 len;
	union {
		int fd;
		compat_uptr_t userptr;
		u64 reserved;
	} base;
	u32 data_offset;
	u32 bytes_used;
	u32 flags;
	u32 reserved[2];
} __packed;

struct ipu_psys_graph_info32 {
	u8 graph_id;
	u8 num_nodes;
	compat_uptr_t nodes;
	struct graph_link links[MAX_GRAPH_LINKS];
} __packed;

struct ipu_psys_task_request32 {
	u8 graph_id;
	u8 node_ctx_id;
	u8 frame_id;
	u32 payload_reuse_bm[2];
	u8 term_buf_count;
	compat_uptr_t task_buffers;
} __packed;

static int
get_ipu_psys_buffer32(struct ipu_psys_buffer *kp,
		      struct ipu_psys_buffer32 __user *up)
{
	compat_uptr_t ptr;
	bool access_ok;

	access_ok = access_ok(up, sizeof(struct ipu_psys_buffer32));
	if (!access_ok || get_user(kp->len, &up->len) ||
	    get_user(ptr, &up->base.userptr) ||
	    get_user(kp->data_offset, &up->data_offset) ||
	    get_user(kp->bytes_used, &up->bytes_used) ||
	    get_user(kp->flags, &up->flags))
		return -EFAULT;

	kp->base.userptr = compat_ptr(ptr);

	return 0;
}

static int
put_ipu_psys_buffer32(struct ipu_psys_buffer *kp,
		      struct ipu_psys_buffer32 __user *up)
{
	bool access_ok;

	access_ok = access_ok(up, sizeof(struct ipu_psys_buffer32));
	if (!access_ok || put_user(kp->len, &up->len) ||
	    put_user(kp->base.fd, &up->base.fd) ||
	    put_user(kp->data_offset, &up->data_offset) ||
	    put_user(kp->bytes_used, &up->bytes_used) ||
	    put_user(kp->flags, &up->flags))
		return -EFAULT;

	return 0;
}

static int
get_ipu_psys_graph_info(struct ipu_psys_graph_info *kp,
			struct ipu_psys_graph_info32 __user *up)
{
	compat_uptr_t nodes;
	bool access_ok;

	access_ok = access_ok(up, sizeof(struct ipu_psys_graph_info32));
	if (!access_ok || get_user(kp->graph_id, &up->graph_id) ||
	    get_user(kp->num_nodes, &up->num_nodes) ||
	    get_user(nodes, &up->nodes) ||
	    copy_from_user(kp->links, up->links, sizeof(up->links)))
		return -EFAULT;

	kp->nodes = compat_ptr(nodes);

	return 0;
}

static int
put_ipu_psys_graph_info(struct ipu_psys_graph_info *kp,
			struct ipu_psys_graph_info32 __user *up)
{
	bool access_ok;

	access_ok = access_ok(up, sizeof(struct ipu_psys_graph_info32));

	if (!access_ok || put_user(kp->graph_id, &up->graph_id))
		return -EFAULT;

	return 0;
}

static int
get_ipu_psys_task_request(struct ipu_psys_task_request *kp,
			  struct ipu_psys_task_request32 __user *up)
{
	compat_uptr_t task_buffers = {0};
	bool access_ok;

	access_ok = access_ok(up, sizeof(struct ipu_psys_task_request32));
	if (!access_ok || get_user(kp->graph_id, &up->graph_id) ||
	    put_user(kp->node_ctx_id, &up->node_ctx_id) ||
	    put_user(kp->frame_id, &up->frame_id) ||
	    put_user(kp->term_buf_count, &up->term_buf_count) ||
	    copy_from_user(kp->payload_reuse_bm, up->payload_reuse_bm,
			   sizeof(up->payload_reuse_bm)) ||
	    put_user(task_buffers, &up->task_buffers))
		return -EFAULT;

	kp->task_buffers = compat_ptr(task_buffers);

	return 0;
}

#define IPU_IOC_GETBUF32 _IOWR('A', 4, struct ipu_psys_buffer32)
#define IPU_IOC_PUTBUF32 _IOWR('A', 5, struct ipu_psys_buffer32)
#define IPU_IOC_GRAPH_OPEN32 _IOWR('A', 7, struct ipu_psys_graph_info32)
#define IPU_IOC_TASK_REQUEST32 _IOWR('A', 8, struct ipu_psys_task_request32)

long ipu_psys_compat_ioctl32(struct file *file, unsigned int cmd,
			     unsigned long arg)
{
	union {
		struct ipu_psys_buffer buf;
		struct ipu_psys_graph_info info;
		struct ipu_psys_task_request request;
	} karg;
	int compatible_arg = 1;
	int err = 0;
	void __user *up = compat_ptr(arg);

	switch (cmd) {
	case IPU_IOC_GETBUF32:
		cmd = IPU_IOC_GETBUF;
		break;
	case IPU_IOC_PUTBUF32:
		cmd = IPU_IOC_PUTBUF;
		break;
	case IPU_IOC_GRAPH_OPEN32:
		cmd = IPU_IOC_GRAPH_OPEN;
		break;
	case IPU_IOC_TASK_REQUEST32:
		cmd = IPU_IOC_TASK_REQUEST;
		break;
	}

	switch (cmd) {
	case IPU_IOC_GETBUF:
	case IPU_IOC_PUTBUF:
		err = get_ipu_psys_buffer32(&karg.buf, up);
		compatible_arg = 0;
		break;
	case IPU_IOC_GRAPH_OPEN:
		err = get_ipu_psys_graph_info(&karg.info, up);
		compatible_arg = 0;
		break;
	case IPU_IOC_TASK_REQUEST:
		err = get_ipu_psys_task_request(&karg.request, up);
		break;
	}
	if (err)
		return err;

	if (compatible_arg) {
		err = native_ioctl(file, cmd, (unsigned long)up);
	} else {
		err = native_ioctl(file, cmd, (unsigned long)&karg);
	}

	if (err)
		return err;

	switch (cmd) {
	case IPU_IOC_GETBUF:
		err = put_ipu_psys_buffer32(&karg.buf, up);
		break;
	case IPU_IOC_GRAPH_OPEN:
		err = put_ipu_psys_graph_info(&karg.info, up);
		break;
	}

	return err;
}
