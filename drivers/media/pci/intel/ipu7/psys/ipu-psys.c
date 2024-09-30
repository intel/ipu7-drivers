// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2013 - 2024 Intel Corporation

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-buf.h>
#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/highmem.h>
#include <linux/completion.h>
#include <linux/init_task.h>
#include <linux/kthread.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/poll.h>
#include <uapi/linux/sched/types.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/dma-mapping.h>

#include <uapi/linux/ipu7-psys.h>

#include "ipu7.h"
#include "ipu7-mmu.h"
#include "ipu7-bus.h"
#include "ipu7-buttress.h"
#include "ipu7-cpd.h"
#include "ipu7-fw-psys.h"
#include "ipu7-psys.h"
#include "ipu7-platform-regs.h"
#include "ipu7-syscom.h"
#include "ipu7-boot.h"

static bool async_fw_init;
module_param(async_fw_init, bool, 0664);
MODULE_PARM_DESC(async_fw_init, "Enable asynchronous firmware initialization");

#define IPU_PSYS_NUM_DEVICES		4

static int psys_runtime_pm_resume(struct device *dev);
static int psys_runtime_pm_suspend(struct device *dev);

#define IPU_FW_CALL_TIMEOUT_JIFFIES			\
	msecs_to_jiffies(IPU_PSYS_CMD_TIMEOUT_MS)

static dev_t ipu7_psys_dev_t;
static DECLARE_BITMAP(ipu7_psys_devices, IPU_PSYS_NUM_DEVICES);
static DEFINE_MUTEX(ipu7_psys_mutex);

static struct fw_init_task {
	struct delayed_work work;
	struct ipu7_psys *psys;
} fw_init_task;

static void ipu7_psys_remove(struct auxiliary_device *auxdev);

static int ipu7_psys_get_userpages(struct ipu7_dma_buf_attach *attach)
{
	struct vm_area_struct *vma;
	unsigned long start, end;
	int npages, array_size;
	struct page **pages;
	struct sg_table *sgt;
	int ret = -ENOMEM;
	int nr = 0;
	u32 flags;

	start = (unsigned long)attach->userptr;
	end = PAGE_ALIGN(start + attach->len);
	npages = PHYS_PFN(end - (start & PAGE_MASK));
	array_size = npages * sizeof(struct page *);

	sgt = kzalloc(sizeof(*sgt), GFP_KERNEL);
	if (!sgt)
		return -ENOMEM;

	WARN_ON_ONCE(attach->npages);

	pages = kvzalloc(array_size, GFP_KERNEL);
	if (!pages)
		goto free_sgt;

	mmap_read_lock(current->mm);
	vma = vma_lookup(current->mm, start);
	if (unlikely(!vma)) {
		ret = -EFAULT;
		goto error_up_read;
	}
	mmap_read_unlock(current->mm);

	flags = FOLL_WRITE | FOLL_FORCE | FOLL_LONGTERM;
	nr = pin_user_pages_fast(start & PAGE_MASK, npages,
				 flags, pages);
	if (nr < npages)
		goto error;

	attach->pages = pages;
	attach->npages = npages;

	ret = sg_alloc_table_from_pages(sgt, pages, npages,
					start & ~PAGE_MASK, attach->len,
					GFP_KERNEL);
	if (ret < 0)
		goto error;

	attach->sgt = sgt;

	return 0;

error_up_read:
	mmap_read_unlock(current->mm);
error:
	if (nr)
		unpin_user_pages(pages, nr);

	kvfree(pages);
free_sgt:
	kfree(sgt);

	pr_err("failed to get userpages:%d\n", ret);

	return ret;
}

static void ipu7_psys_put_userpages(struct ipu7_dma_buf_attach *attach)
{
	if (!attach || !attach->userptr || !attach->sgt)
		return;

	unpin_user_pages(attach->pages, attach->npages);

	kvfree(attach->pages);

	sg_free_table(attach->sgt);
	kfree(attach->sgt);
	attach->sgt = NULL;
}

static int ipu7_dma_buf_attach(struct dma_buf *dbuf,
			       struct dma_buf_attachment *attach)
{
	struct ipu7_psys_kbuffer *kbuf = dbuf->priv;
	struct ipu7_dma_buf_attach *ipu7_attach;
	int ret;

	ipu7_attach = kzalloc(sizeof(*ipu7_attach), GFP_KERNEL);
	if (!ipu7_attach)
		return -ENOMEM;

	ipu7_attach->len = kbuf->len;
	ipu7_attach->userptr = kbuf->userptr;

	attach->priv = ipu7_attach;

	ret = ipu7_psys_get_userpages(ipu7_attach);
	if (ret) {
		kfree(ipu7_attach);
		return ret;
	}

	return 0;
}

static void ipu7_dma_buf_detach(struct dma_buf *dbuf,
				struct dma_buf_attachment *attach)
{
	struct ipu7_dma_buf_attach *ipu7_attach = attach->priv;

	ipu7_psys_put_userpages(ipu7_attach);
	kfree(ipu7_attach);
	attach->priv = NULL;
}

static struct sg_table *ipu7_dma_buf_map(struct dma_buf_attachment *attach,
					 enum dma_data_direction dir)
{
	struct ipu7_dma_buf_attach *ipu7_attach = attach->priv;
	unsigned long attrs;
	int ret;

	attrs = DMA_ATTR_SKIP_CPU_SYNC;
	ret = dma_map_sgtable(attach->dev, ipu7_attach->sgt, dir, attrs);
	if (ret < 0) {
		dev_err(attach->dev, "buf map failed\n");
		return ERR_PTR(-EIO);
	}

	/*
	 * Initial cache flush to avoid writing dirty pages for buffers which
	 * are later marked as IPU_BUFFER_FLAG_NO_FLUSH.
	 */
	dma_sync_sgtable_for_device(attach->dev, ipu7_attach->sgt,
				    DMA_BIDIRECTIONAL);

	return ipu7_attach->sgt;
}

static void ipu7_dma_buf_unmap(struct dma_buf_attachment *attach,
			       struct sg_table *sg, enum dma_data_direction dir)
{
	dma_unmap_sgtable(attach->dev, sg, dir, DMA_ATTR_SKIP_CPU_SYNC);
}

static int ipu7_dma_buf_mmap(struct dma_buf *dbuf, struct vm_area_struct *vma)
{
	return -ENOTTY;
}

static void ipu7_dma_buf_release(struct dma_buf *buf)
{
	struct ipu7_psys_kbuffer *kbuf = buf->priv;

	if (!kbuf)
		return;

	if (kbuf->db_attach)
		ipu7_psys_put_userpages(kbuf->db_attach->priv);

	kfree(kbuf);
}

static int ipu7_dma_buf_begin_cpu_access(struct dma_buf *dma_buf,
					 enum dma_data_direction dir)
{
	return -ENOTTY;
}

static int ipu7_dma_buf_vmap(struct dma_buf *dmabuf, struct iosys_map *map)
{
	struct dma_buf_attachment *attach;
	struct ipu7_dma_buf_attach *ipu7_attach;

	if (list_empty(&dmabuf->attachments))
		return -EINVAL;

	attach = list_last_entry(&dmabuf->attachments,
				 struct dma_buf_attachment, node);
	ipu7_attach = attach->priv;

	if (!ipu7_attach || !ipu7_attach->pages || !ipu7_attach->npages)
		return -EINVAL;

	map->vaddr = vm_map_ram(ipu7_attach->pages, ipu7_attach->npages, 0);
	map->is_iomem = false;
	if (!map->vaddr)
		return -EINVAL;

	return 0;
}

static void ipu7_dma_buf_vunmap(struct dma_buf *dmabuf, struct iosys_map *map)
{
	struct dma_buf_attachment *attach;
	struct ipu7_dma_buf_attach *ipu7_attach;

	if (WARN_ON(list_empty(&dmabuf->attachments)))
		return;

	attach = list_last_entry(&dmabuf->attachments,
				 struct dma_buf_attachment, node);
	ipu7_attach = attach->priv;

	if (WARN_ON(!ipu7_attach || !ipu7_attach->pages ||
		    !ipu7_attach->npages))
		return;

	vm_unmap_ram(map->vaddr, ipu7_attach->npages);
}

struct dma_buf_ops ipu7_dma_buf_ops = {
	.attach = ipu7_dma_buf_attach,
	.detach = ipu7_dma_buf_detach,
	.map_dma_buf = ipu7_dma_buf_map,
	.unmap_dma_buf = ipu7_dma_buf_unmap,
	.release = ipu7_dma_buf_release,
	.begin_cpu_access = ipu7_dma_buf_begin_cpu_access,
	.mmap = ipu7_dma_buf_mmap,
	.vmap = ipu7_dma_buf_vmap,
	.vunmap = ipu7_dma_buf_vunmap,
};

static int ipu7_psys_get_graph_id(struct ipu7_psys_fh *fh)
{
	u8 graph_id = 0;

	for (graph_id = 0; graph_id < IPU_PSYS_NUM_STREAMS; graph_id++) {
		if (fh->psys->graph_id[graph_id] == INVALID_STREAM_ID)
			break;
	}

	if (graph_id == IPU_PSYS_NUM_STREAMS)
		return -EBUSY;

	fh->psys->graph_id[graph_id] = graph_id;
	return graph_id;
}

static void ipu7_psys_put_graph_id(struct ipu7_psys_fh *fh)
{
	fh->psys->graph_id[fh->ip->graph_id] = INVALID_STREAM_ID;
}

static void ipu7_psys_stream_deinit(struct ipu7_psys_stream *ip,
				    struct ipu7_bus_device *adev)
{
	u8 i;

	mutex_destroy(&ip->event_mutex);
	mutex_destroy(&ip->task_mutex);

	for (i = 0; i < MAX_TASK_REQUEST_QUEUE_SIZE; i++) {
		if (ip->task_queue[i].msg_task) {
			ip->task_queue[i].available = 0;
			dma_free_attrs(&adev->auxdev.dev,
				       sizeof(struct ipu7_msg_task),
				       ip->task_queue[i].msg_task,
				       ip->task_queue[i].task_dma_addr, 0);
		}
	}
}

static int ipu7_psys_stream_init(struct ipu7_psys_stream *ip,
				 struct ipu7_bus_device *adev)
{
	struct device *dev = &adev->auxdev.dev;
	u8 i, j;

	ip->event_read_index = 0;
	ip->event_write_index = 0;

	for (i = 0; i < MAX_TASK_REQUEST_QUEUE_SIZE; i++) {
		ip->task_queue[i].available = 1;
		ip->task_queue[i].msg_task =
			dma_alloc_attrs(dev, sizeof(struct ipu7_msg_task),
					&ip->task_queue[i].task_dma_addr,
					GFP_KERNEL, 0);

		if (!ip->task_queue[i].msg_task) {
			dev_err(dev, "Failed to allocate msg task.\n");
			goto allocate_fail;
		}
	}

	for (i = 0; i < MAX_TASK_EVENT_QUEUE_SIZE; i++)
		ip->event_queue[i].available = 1;

	init_completion(&ip->graph_open);
	init_completion(&ip->graph_close);

	return 0;

allocate_fail:
	for (j = 0; j < i; j++) {
		if (ip->task_queue[j].msg_task) {
			ip->task_queue[j].available = 0;
			dma_free_attrs(dev, sizeof(struct ipu7_msg_task),
				       ip->task_queue[j].msg_task,
				       ip->task_queue[j].task_dma_addr, 0);
		}
	}

	return -ENOMEM;
}

static int ipu7_psys_open(struct inode *inode, struct file *file)
{
	struct ipu7_psys *psys = inode_to_ipu_psys(inode);
	struct device *dev = &psys->adev->auxdev.dev;
	struct ipu7_psys_fh *fh;
	struct ipu7_psys_stream *ip;
	int rval;

	fh = kzalloc(sizeof(*fh), GFP_KERNEL);
	if (!fh)
		return -ENOMEM;

	ip = kzalloc(sizeof(*ip), GFP_KERNEL);
	if (!ip) {
		rval = -ENOMEM;
		goto alloc_failed;
	}

	rval = ipu7_psys_stream_init(ip, psys->adev);
	if (rval)
		goto stream_init_failed;

	fh->ip = ip;
	ip->fh = fh;

	fh->psys = psys;

	file->private_data = fh;

	mutex_init(&fh->mutex);
	INIT_LIST_HEAD(&fh->bufmap);
	init_waitqueue_head(&fh->wait);

	mutex_init(&ip->task_mutex);
	mutex_init(&ip->event_mutex);

	mutex_lock(&psys->mutex);

	rval = ipu7_psys_get_graph_id(fh);
	if (rval < 0)
		goto open_failed;

	fh->ip->graph_id = rval;

	rval = pm_runtime_get_sync(dev);
	if (rval < 0) {
		dev_err(dev, "Runtime PM failed (%d)\n", rval);
		goto rpm_put;
	}

	list_add_tail(&fh->list, &psys->fhs);

	mutex_unlock(&psys->mutex);

	return 0;

rpm_put:
	pm_runtime_put(dev);
	ipu7_psys_put_graph_id(fh);

open_failed:
	ipu7_psys_stream_deinit(ip, psys->adev);

	mutex_destroy(&fh->mutex);

	mutex_unlock(&psys->mutex);

stream_init_failed:
	kfree(ip);

alloc_failed:
	kfree(fh);

	return rval;
}

static inline void ipu7_psys_kbuf_unmap(struct ipu7_psys_kbuffer *kbuf)
{
	if (!kbuf)
		return;

	kbuf->valid = false;
	if (kbuf->kaddr) {
		struct iosys_map dmap;

		iosys_map_set_vaddr(&dmap, kbuf->kaddr);
		dma_buf_vunmap_unlocked(kbuf->dbuf, &dmap);
	}

	if (kbuf->sgt)
		dma_buf_unmap_attachment_unlocked(kbuf->db_attach,
						  kbuf->sgt,
						  DMA_BIDIRECTIONAL);
	if (kbuf->db_attach)
		dma_buf_detach(kbuf->dbuf, kbuf->db_attach);
	dma_buf_put(kbuf->dbuf);

	kbuf->db_attach = NULL;
	kbuf->dbuf = NULL;
	kbuf->sgt = NULL;
}

static int ipu7_psys_release(struct inode *inode, struct file *file)
{
	struct ipu7_psys *psys = inode_to_ipu_psys(inode);
	struct ipu7_psys_fh *fh = file->private_data;
	struct ipu7_psys_kbuffer *kbuf, *kbuf0;
	struct dma_buf_attachment *dba;

	mutex_lock(&fh->mutex);
	/* clean up buffers */
	if (!list_empty(&fh->bufmap)) {
		list_for_each_entry_safe(kbuf, kbuf0, &fh->bufmap, list) {
			list_del(&kbuf->list);
			dba = kbuf->db_attach;

			/* Unmap and release buffers */
			if (kbuf->dbuf && dba) {
				ipu7_psys_kbuf_unmap(kbuf);
			} else {
				if (dba)
					ipu7_psys_put_userpages(dba->priv);
				kfree(kbuf);
			}
		}
	}
	mutex_unlock(&fh->mutex);

	ipu7_psys_stream_deinit(fh->ip, psys->adev);

	mutex_lock(&psys->mutex);
	list_del(&fh->list);

	ipu7_psys_put_graph_id(fh);
	kfree(fh->ip);

	if (list_empty(&psys->fhs))
		psys->power_gating = 0;

	mutex_unlock(&psys->mutex);
	mutex_destroy(&fh->mutex);
	kfree(fh);

	pm_runtime_put(&psys->adev->auxdev.dev);

	return 0;
}

static int ipu7_psys_getbuf(struct ipu_psys_buffer *buf, struct ipu7_psys_fh *fh)
{
	struct device *dev = &fh->psys->adev->auxdev.dev;
	struct ipu7_psys_kbuffer *kbuf;

	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);
	struct dma_buf *dbuf;
	int ret;

	if (!buf->base.userptr) {
		dev_err(dev, "Buffer allocation not supported\n");
		return -EINVAL;
	}

	if (!PAGE_ALIGNED(buf->base.userptr)) {
		dev_err(dev, "Not page-aligned userptr is not supported\n");
		return -EINVAL;
	}

	kbuf = kzalloc(sizeof(*kbuf), GFP_KERNEL);
	if (!kbuf)
		return -ENOMEM;

	kbuf->len = buf->len;
	kbuf->userptr = buf->base.userptr;
	kbuf->flags = buf->flags;

	exp_info.ops = &ipu7_dma_buf_ops;
	exp_info.size = kbuf->len;
	exp_info.flags = O_RDWR;
	exp_info.priv = kbuf;

	dbuf = dma_buf_export(&exp_info);
	if (IS_ERR(dbuf)) {
		kfree(kbuf);
		return PTR_ERR(dbuf);
	}

	ret = dma_buf_fd(dbuf, 0);
	if (ret < 0) {
		dma_buf_put(dbuf);
		return ret;
	}

	kbuf->fd = ret;
	buf->base.fd = ret;
	buf->flags &= ~IPU_BUFFER_FLAG_USERPTR;
	buf->flags |= IPU_BUFFER_FLAG_DMA_HANDLE;
	kbuf->flags = buf->flags;

	mutex_lock(&fh->mutex);
	list_add(&kbuf->list, &fh->bufmap);
	mutex_unlock(&fh->mutex);

	dev_dbg(dev, "IOC_GETBUF: userptr %p size %llu to fd %d",
		buf->base.userptr, buf->len, buf->base.fd);

	return 0;
}

static int ipu7_psys_putbuf(struct ipu_psys_buffer *buf, struct ipu7_psys_fh *fh)
{
	return 0;
}

static struct ipu7_psys_kbuffer *
ipu7_psys_lookup_kbuffer(struct ipu7_psys_fh *fh, int fd)
{
	struct ipu7_psys_kbuffer *kbuf;

	list_for_each_entry(kbuf, &fh->bufmap, list) {
		if (kbuf->fd == fd)
			return kbuf;
	}

	return NULL;
}

static int ipu7_psys_unmapbuf_locked(int fd, struct ipu7_psys_fh *fh,
				     struct ipu7_psys_kbuffer *kbuf)
{
	struct device *dev = &fh->psys->adev->auxdev.dev;

	if (!kbuf || fd != kbuf->fd) {
		dev_err(dev, "invalid kbuffer\n");
		return -EINVAL;
	}

	/* From now on it is not safe to use this kbuffer */
	ipu7_psys_kbuf_unmap(kbuf);

	list_del(&kbuf->list);

	if (!kbuf->userptr)
		kfree(kbuf);

	dev_dbg(dev, "%s fd %d unmapped\n", __func__, fd);

	return 0;
}

static int ipu7_psys_mapbuf_locked(int fd, struct ipu7_psys_fh *fh,
				   struct ipu7_psys_kbuffer *kbuf)
{
	struct device *dev = &fh->psys->adev->auxdev.dev;
	struct dma_buf *dbuf;
	struct iosys_map dmap = {
		.is_iomem = false,
	};
	int ret;

	dbuf = dma_buf_get(fd);
	if (IS_ERR(dbuf))
		return -EINVAL;

	if (!kbuf) {
		/* This fd isn't generated by ipu7_psys_getbuf, it
		 * is a new fd. Create a new kbuf item for this fd, and
		 * add this kbuf to bufmap list.
		 */
		kbuf = kzalloc(sizeof(*kbuf), GFP_KERNEL);
		if (!kbuf) {
			ret = -ENOMEM;
			goto mapbuf_fail;
		}

		list_add(&kbuf->list, &fh->bufmap);
	}

	/* fd valid and found, need remap */
	if (kbuf->dbuf && (kbuf->dbuf != dbuf || kbuf->len != dbuf->size)) {
		dev_dbg(dev, "dmabuf fd %d with kbuf %p changed, need remap.\n",
			fd, kbuf);
		ret = ipu7_psys_unmapbuf_locked(fd, fh, kbuf);
		if (ret)
			goto mapbuf_fail;

		kbuf = ipu7_psys_lookup_kbuffer(fh, fd);
		/* changed external dmabuf */
		if (!kbuf) {
			kbuf = kzalloc(sizeof(*kbuf), GFP_KERNEL);
			if (!kbuf) {
				ret = -ENOMEM;
				goto mapbuf_fail;
			}
			list_add(&kbuf->list, &fh->bufmap);
		}
	}

	if (kbuf->sgt) {
		dev_dbg(dev, "fd %d has been mapped!\n", fd);
		dma_buf_put(dbuf);
		goto mapbuf_end;
	}

	kbuf->dbuf = dbuf;

	if (kbuf->len == 0)
		kbuf->len = kbuf->dbuf->size;

	kbuf->fd = fd;

	kbuf->db_attach = dma_buf_attach(kbuf->dbuf, dev);
	if (IS_ERR(kbuf->db_attach)) {
		ret = PTR_ERR(kbuf->db_attach);
		dev_dbg(dev, "dma buf attach failed\n");
		goto kbuf_map_fail;
	}

	kbuf->sgt = dma_buf_map_attachment_unlocked(kbuf->db_attach,
						    DMA_BIDIRECTIONAL);
	if (IS_ERR_OR_NULL(kbuf->sgt)) {
		ret = -EINVAL;
		kbuf->sgt = NULL;
		dev_dbg(dev, "dma buf map attachment failed\n");
		goto kbuf_map_fail;
	}

	kbuf->dma_addr = sg_dma_address(kbuf->sgt->sgl);

	/* no need vmap for imported dmabufs */
	if (!kbuf->userptr)
		goto mapbuf_end;

	ret = dma_buf_vmap_unlocked(kbuf->dbuf, &dmap);
	if (ret) {
		dev_dbg(dev, "dma buf vmap failed\n");
		goto kbuf_map_fail;
	}
	kbuf->kaddr = dmap.vaddr;

	dev_dbg(dev, "%s kbuf %p fd %d with len %llu mapped\n",
		__func__, kbuf, fd, kbuf->len);
mapbuf_end:

	kbuf->valid = true;

	return 0;

kbuf_map_fail:
	ipu7_psys_kbuf_unmap(kbuf);

	list_del(&kbuf->list);
	if (!kbuf->userptr)
		kfree(kbuf);

mapbuf_fail:
	dma_buf_put(dbuf);

	dev_err(dev, "%s failed for fd %d\n", __func__, fd);
	return ret;
}

static long ipu7_psys_mapbuf(int fd, struct ipu7_psys_fh *fh)
{
	long ret;
	struct ipu7_psys_kbuffer *kbuf;

	mutex_lock(&fh->mutex);
	kbuf = ipu7_psys_lookup_kbuffer(fh, fd);
	ret = ipu7_psys_mapbuf_locked(fd, fh, kbuf);
	mutex_unlock(&fh->mutex);

	dev_dbg(&fh->psys->adev->auxdev.dev, "IOC_MAPBUF ret %ld\n", ret);

	return ret;
}

static long ipu7_psys_unmapbuf(int fd, struct ipu7_psys_fh *fh)
{
	struct device *dev = &fh->psys->adev->auxdev.dev;
	struct ipu7_psys_kbuffer *kbuf;
	long ret;

	mutex_lock(&fh->mutex);
	kbuf = ipu7_psys_lookup_kbuffer(fh, fd);
	if (!kbuf) {
		dev_err(dev,
			"buffer with fd %d not found\n", fd);
		mutex_unlock(&fh->mutex);
		return -EINVAL;
	}
	ret = ipu7_psys_unmapbuf_locked(fd, fh, kbuf);
	mutex_unlock(&fh->mutex);

	dev_dbg(dev, "IOC_UNMAPBUF\n");

	return ret;
}

static long ipu_psys_graph_open(struct ipu_psys_graph_info *graph,
				 struct ipu7_psys_fh *fh)
{
	struct ipu7_psys *psys = fh->psys;
	int ret = 0;

	if (fh->ip->graph_state != IPU_MSG_GRAPH_STATE_CLOSED) {
		dev_err(&psys->dev, "Wrong state %d to open graph %d\n",
			fh->ip->graph_state, fh->ip->graph_id);
		return -EINVAL;
	}

	if (!graph->nodes || graph->num_nodes > MAX_GRAPH_NODES) {
		dev_err(&psys->dev, "nodes is wrong\n");
		return -EINVAL;
	}

	if (copy_from_user(fh->ip->nodes, graph->nodes,
			   graph->num_nodes * sizeof(*graph->nodes))) {
		dev_err(&psys->dev, "Failed to copy nodes\n");
		return -EINVAL;
	}

	reinit_completion(&fh->ip->graph_open);

	ret = ipu7_fw_psys_graph_open(graph, psys, fh->ip);
	if (ret) {
		dev_err(&psys->dev, "Failed to open graph %d\n",
			fh->ip->graph_id);
		return ret;
	}

	fh->ip->graph_state = IPU_MSG_GRAPH_STATE_OPEN_WAIT;

	ret = wait_for_completion_timeout(&fh->ip->graph_open,
					  IPU_FW_CALL_TIMEOUT_JIFFIES);
	if (!ret) {
		dev_err(&psys->dev, "Open graph %d timeout\n",
			fh->ip->graph_id);
		fh->ip->graph_state = IPU_MSG_GRAPH_STATE_CLOSED;
		return -ETIMEDOUT;
	}

	if (fh->ip->graph_state != IPU_MSG_GRAPH_STATE_OPEN) {
		dev_err(&psys->dev, "Failed to set graph\n");
		fh->ip->graph_state = IPU_MSG_GRAPH_STATE_CLOSED;
		return -EINVAL;
	}

	graph->graph_id = fh->ip->graph_id;

	return 0;
}

static long ipu_psys_graph_close(int graph_id, struct ipu7_psys_fh *fh)
{
	struct ipu7_psys *psys = fh->psys;
	int ret = 0;

	if (fh->ip->graph_state != IPU_MSG_GRAPH_STATE_OPEN) {
		dev_err(&psys->dev, "Wrong state %d to open graph %d\n",
			fh->ip->graph_state, fh->ip->graph_id);
		return -EINVAL;
	}

	reinit_completion(&fh->ip->graph_close);

	ret = ipu7_fw_psys_graph_close(fh->ip->graph_id, fh->psys);
	if (ret) {
		dev_err(&psys->dev, "Failed to close graph %d\n",
			fh->ip->graph_id);
		return ret;
	}

	fh->ip->graph_state = IPU_MSG_GRAPH_STATE_CLOSE_WAIT;

	ret = wait_for_completion_timeout(&fh->ip->graph_close,
					  IPU_FW_CALL_TIMEOUT_JIFFIES);
	if (!ret) {
		dev_err(&psys->dev, "Close graph %d timeout\n",
			fh->ip->graph_id);
		return -ETIMEDOUT;
	}

	if (fh->ip->graph_state != IPU_MSG_GRAPH_STATE_CLOSED) {
		dev_err(&psys->dev, "Failed to close graph\n");
		fh->ip->graph_state = IPU_MSG_GRAPH_STATE_CLOSED;
		return -EINVAL;
	}

	return 0;
}

static struct ipu_psys_task_queue *
ipu7_psys_get_task_queue(struct ipu7_psys_stream *ip,
			 struct ipu_psys_task_request *task)
{
	struct ipu_psys_task_queue *tq = NULL;
	struct device *dev = &ip->fh->psys->dev;
	struct device *adev = &ip->fh->psys->adev->auxdev.dev;
	struct ipu7_psys_kbuffer *kbuf = NULL;
	u32 i, j;
	int fd, prevfd = -1;

	if (task->term_buf_count > MAX_GRAPH_TERMINALS) {
		dev_err(dev, "num_teminal_buffer is too large\n");
		return NULL;
	}

	mutex_lock(&ip->task_mutex);
	for (i = 0U; i < MAX_TASK_REQUEST_QUEUE_SIZE; i++) {
		if (ip->task_queue[i].available == 1U) {
			tq = &ip->task_queue[i];

			if (copy_from_user(tq->task_buffers,
					   task->task_buffers,
					   task->term_buf_count *
					   sizeof(*task->task_buffers))) {
				dev_err(dev, "failed to copy task buffers\n");
				goto unlock;
			}

			for (j = 0; j < task->term_buf_count; j++) {
				fd = tq->task_buffers[j].term_buf.base.fd;
				kbuf = ipu7_psys_lookup_kbuffer(ip->fh, fd);
				if (!kbuf) {
					dev_err(dev, "fd %d not found\n", fd);
					goto unlock;
				}
				tq->ipu7_addr[j] = kbuf->dma_addr
					+ tq->task_buffers[j].term_buf.data_offset;

				if ((tq->task_buffers[j].term_buf.flags &
				     IPU_BUFFER_FLAG_NO_FLUSH) ||
				    prevfd == fd)
					continue;

				prevfd = fd;
				dma_sync_sgtable_for_device(adev, kbuf->sgt,
							    DMA_BIDIRECTIONAL);
			}

			ip->task_queue[i].available = 0U;
			dev_dbg(dev, "frame %d to task queue %p\n",
				task->frame_id, tq);

			mutex_unlock(&ip->task_mutex);
			return tq;
		}
	}

	dev_err(dev, "No available take queues for stream %p\n", ip);
unlock:
	mutex_unlock(&ip->task_mutex);
	return NULL;
}

static long ipu_psys_task_request(struct ipu_psys_task_request *task,
				   struct ipu7_psys_fh *fh)
{
	struct ipu7_psys *psys = fh->psys;
	struct ipu_psys_task_queue *tq;
	int ret = 0;

	if (task->term_buf_count == 0 || !task->task_buffers) {
		dev_err(&psys->dev, "task_buffer is NULL\n");
		return -EINVAL;
	}

	tq = ipu7_psys_get_task_queue(fh->ip, task);
	if (!tq) {
		dev_err(&psys->dev, "Failed to get task queue\n");
		return -EINVAL;
	}

	ret = ipu7_fw_psys_task_request(task, fh->ip, tq, psys);
	if (ret) {
		dev_err(&psys->dev, "Failed to request task %d\n",
			fh->ip->graph_id);
		mutex_lock(&fh->ip->task_mutex);
		tq->available = 1;
		mutex_unlock(&fh->ip->task_mutex);
		return ret;
	}

	tq->task_state = IPU_MSG_TASK_STATE_WAIT_DONE;

	return 0;
}

static unsigned int ipu7_psys_poll(struct file *file,
				   struct poll_table_struct *wait)
{
	struct ipu7_psys_fh *fh = file->private_data;
	struct device *dev = &fh->psys->adev->auxdev.dev;
	struct ipu7_psys_stream *ip = fh->ip;
	unsigned int res = 0;

	dev_dbg(dev, "ipu psys poll\n");

	poll_wait(file, &fh->wait, wait);

	mutex_lock(&ip->event_mutex);
	if (!ip->event_queue[ip->event_read_index].available)
		res = POLLIN;
	mutex_unlock(&ip->event_mutex);

	dev_dbg(dev, "ipu psys poll res %u\n", res);

	return res;
}

static long ipu7_psys_ioctl(struct file *file, unsigned int cmd,
			    unsigned long arg)
{
	union {
		struct ipu_psys_graph_info graph;
		struct ipu_psys_task_request task;
		struct ipu_psys_buffer buf;
		struct ipu_psys_event ev;
		struct ipu_psys_capability caps;
	} karg;
	struct ipu7_psys_fh *fh = file->private_data;
	long err = 0;
	void __user *up = (void __user *)arg;
	bool copy = (cmd != IPU_IOC_MAPBUF && cmd != IPU_IOC_UNMAPBUF &&
		     cmd != IPU_IOC_GRAPH_CLOSE);

	if (copy) {
		if (_IOC_SIZE(cmd) > sizeof(karg))
			return -ENOTTY;

		if (_IOC_DIR(cmd) & _IOC_WRITE) {
			err = copy_from_user(&karg, up, _IOC_SIZE(cmd));
			if (err)
				return -EFAULT;
		}
	}

	switch (cmd) {
	case IPU_IOC_MAPBUF:
		err = ipu7_psys_mapbuf(arg, fh);
		break;
	case IPU_IOC_UNMAPBUF:
		err = ipu7_psys_unmapbuf(arg, fh);
		break;
	case IPU_IOC_QUERYCAP:
		karg.caps = fh->psys->caps;
		break;
	case IPU_IOC_GETBUF:
		err = ipu7_psys_getbuf(&karg.buf, fh);
		break;
	case IPU_IOC_PUTBUF:
		err = ipu7_psys_putbuf(&karg.buf, fh);
		break;
	case IPU_IOC_GRAPH_OPEN:
		err = ipu_psys_graph_open(&karg.graph, fh);
		break;
	case IPU_IOC_GRAPH_CLOSE:
		err = ipu_psys_graph_close(arg, fh);
		break;
	case IPU_IOC_TASK_REQUEST:
		err = ipu_psys_task_request(&karg.task, fh);
		break;
	case IPU_IOC_DQEVENT:
		err = ipu7_ioctl_dqevent(&karg.ev, fh, file->f_flags);
		break;
	default:
		err = -ENOTTY;
		break;
	}

	if (err)
		return err;

	if (copy && _IOC_DIR(cmd) & _IOC_READ)
		if (copy_to_user(up, &karg, _IOC_SIZE(cmd)))
			return -EFAULT;

	return 0;
}

static const struct file_operations ipu7_psys_fops = {
	.open = ipu7_psys_open,
	.release = ipu7_psys_release,
	.unlocked_ioctl = ipu7_psys_ioctl,
	.poll = ipu7_psys_poll,
	.owner = THIS_MODULE,
};

static void ipu7_psys_dev_release(struct device *dev)
{
}

static int psys_runtime_pm_resume(struct device *dev)
{
	struct ipu7_bus_device *adev = to_ipu7_bus_device(dev);
	struct ipu7_psys *psys = ipu7_bus_get_drvdata(adev);
	unsigned long flags;
	int rval;

	if (!psys)
		return 0;

	spin_lock_irqsave(&psys->ready_lock, flags);
	if (psys->ready) {
		spin_unlock_irqrestore(&psys->ready_lock, flags);
		return 0;
	}
	spin_unlock_irqrestore(&psys->ready_lock, flags);

	rval = ipu7_mmu_hw_init(adev->mmu);
	if (rval)
		return rval;

	if (async_fw_init && !psys->adev->syscom) {
		dev_err(dev,
			"%s: asynchronous firmware init not finished, skipping\n",
			__func__);
		return 0;
	}

	if (!ipu7_buttress_auth_done(adev->isp)) {
		dev_dbg(dev, "%s: not yet authenticated, skipping\n", __func__);
		return 0;
	}

	ipu7_psys_setup_hw(psys);

	ipu7_psys_subdomains_power(psys, 1);

	rval = ipu7_boot_start_fw(psys->adev);
	if (rval) {
		dev_err(&psys->dev, "failed to start psys fw. ret: %d\n", rval);
		return rval;
	}

	rval = ipu7_fw_psys_open(psys);
	if (rval) {
		dev_err(&psys->adev->auxdev.dev, "Failed to open abi.\n");
		return rval;
	}

	spin_lock_irqsave(&psys->ready_lock, flags);
	psys->ready = 1;
	spin_unlock_irqrestore(&psys->ready_lock, flags);

	return 0;
}

static int psys_runtime_pm_suspend(struct device *dev)
{
	struct ipu7_bus_device *adev = to_ipu7_bus_device(dev);
	struct ipu7_psys *psys = ipu7_bus_get_drvdata(adev);
	unsigned long flags;

	if (!psys)
		return 0;

	if (!psys->ready)
		return 0;

	spin_lock_irqsave(&psys->ready_lock, flags);
	psys->ready = 0;
	spin_unlock_irqrestore(&psys->ready_lock, flags);

	ipu7_fw_psys_close(psys);

	ipu7_boot_stop_fw(psys->adev);

	ipu7_psys_subdomains_power(psys, 0);

	ipu7_mmu_hw_cleanup(adev->mmu);

	return 0;
}

/* The following PM callbacks are needed to enable runtime PM in IPU PCI
 * device resume, otherwise, runtime PM can't work in PCI resume from
 * S3 state.
 */
static int psys_resume(struct device *dev)
{
	return 0;
}

static int psys_suspend(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops psys_pm_ops = {
	.runtime_suspend = psys_runtime_pm_suspend,
	.runtime_resume = psys_runtime_pm_resume,
	.suspend = psys_suspend,
	.resume = psys_resume,
};

#define PSYS_PM_OPS (&psys_pm_ops)

#ifdef CONFIG_DEBUG_FS
static int ipu7_psys_icache_prefetch_sp_get(void *data, u64 *val)
{
	struct ipu7_psys *psys = data;

	*val = psys->icache_prefetch_sp;
	return 0;
}

static int ipu7_psys_icache_prefetch_sp_set(void *data, u64 val)
{
	struct ipu7_psys *psys = data;

	if (val != !!val)
		return -EINVAL;

	psys->icache_prefetch_sp = val;

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(psys_icache_prefetch_sp_fops,
			ipu7_psys_icache_prefetch_sp_get,
			ipu7_psys_icache_prefetch_sp_set, "%llu\n");

static int ipu7_psys_icache_prefetch_isp_get(void *data, u64 *val)
{
	struct ipu7_psys *psys = data;

	*val = psys->icache_prefetch_isp;
	return 0;
}

static int ipu7_psys_icache_prefetch_isp_set(void *data, u64 val)
{
	struct ipu7_psys *psys = data;

	if (val != !!val)
		return -EINVAL;

	psys->icache_prefetch_isp = val;

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(psys_icache_prefetch_isp_fops,
			ipu7_psys_icache_prefetch_isp_get,
			ipu7_psys_icache_prefetch_isp_set, "%llu\n");

static int psys_fw_log_init(struct ipu7_psys *psys)
{
	struct device *dev = &psys->adev->auxdev.dev;
	struct psys_fw_log *fw_log;
	void *log_buf;

	if (psys->fw_log)
		return 0;

	fw_log = devm_kzalloc(dev, sizeof(*fw_log), GFP_KERNEL);
	if (!fw_log)
		return -ENOMEM;

	mutex_init(&fw_log->mutex);

	log_buf = devm_kzalloc(dev, FW_LOG_BUF_SIZE, GFP_KERNEL);
	if (!log_buf)
		return -ENOMEM;

	fw_log->head = log_buf;
	fw_log->addr = log_buf;
	fw_log->count = 0;
	fw_log->size = 0;

	psys->fw_log = fw_log;

	return 0;
}

static ssize_t fwlog_read(struct file *file, char __user *userbuf, size_t size,
			  loff_t *pos)
{
	struct ipu7_psys *psys = file->private_data;
	struct psys_fw_log *fw_log = psys->fw_log;
	struct device *dev = &psys->adev->auxdev.dev;
	u32 log_size;
	void *buf;
	int ret = 0;

	if (!fw_log)
		return 0;

	buf = kvzalloc(FW_LOG_BUF_SIZE, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	mutex_lock(&fw_log->mutex);
	if (!fw_log->size) {
		dev_warn(dev, "no available fw log\n");
		mutex_unlock(&fw_log->mutex);
		goto free_and_return;
	}

	if (fw_log->size > FW_LOG_BUF_SIZE)
		log_size = FW_LOG_BUF_SIZE;
	else
		log_size = fw_log->size;

	memcpy(buf, fw_log->addr, log_size);
	dev_dbg(dev, "copy %d bytes fw log to user\n", log_size);
	mutex_unlock(&fw_log->mutex);

	ret = simple_read_from_buffer(userbuf, size, pos, buf,
				      log_size);
free_and_return:
	kvfree(buf);

	return ret;
}

static const struct file_operations psys_fw_log_fops = {
	.open = simple_open,
	.owner = THIS_MODULE,
	.read = fwlog_read,
	.llseek = default_llseek,
};

static int ipu7_psys_init_debugfs(struct ipu7_psys *psys)
{
	struct dentry *file;
	struct dentry *dir;

	dir = debugfs_create_dir("psys", psys->adev->isp->ipu7_dir);
	if (IS_ERR(dir))
		return -ENOMEM;

	file = debugfs_create_file("icache_prefetch_sp", 0600,
				   dir, psys, &psys_icache_prefetch_sp_fops);
	if (IS_ERR(file))
		goto err;

	file = debugfs_create_file("icache_prefetch_isp", 0600,
				   dir, psys, &psys_icache_prefetch_isp_fops);
	if (IS_ERR(file))
		goto err;

	file = debugfs_create_file("fwlog", 0400,
				   dir, psys, &psys_fw_log_fops);
	if (IS_ERR(file))
		goto err;

	psys->debugfsdir = dir;

	return 0;
err:
	debugfs_remove_recursive(dir);
	return -ENOMEM;
}
#endif

static void run_fw_init_work(struct work_struct *work)
{
	struct fw_init_task *task = (struct fw_init_task *)work;
	struct ipu7_psys *psys = task->psys;
	struct auxiliary_device *auxdev = &psys->adev->auxdev;
	int rval;

	rval = ipu7_fw_psys_init(psys);

	if (rval) {
		dev_err(&auxdev->dev, "FW init failed(%d)\n", rval);
		ipu7_psys_remove(auxdev);
	} else {
		dev_info(&auxdev->dev, "FW init done\n");
	}
}

static const struct bus_type ipu7_psys_bus = {
	.name = "intel-ipu7-psys",
};

static int ipu7_psys_probe(struct auxiliary_device *auxdev,
			   const struct auxiliary_device_id *auxdev_id)
{
	struct ipu7_bus_device *adev = auxdev_to_adev(auxdev);
	struct device *dev = &auxdev->dev;
	struct ipu7_psys *psys;
	unsigned int minor;
	int i, rval = -E2BIG;

	if (!adev->isp->ipu7_bus_ready_to_probe)
		return -EPROBE_DEFER;

	rval = alloc_chrdev_region(&ipu7_psys_dev_t, 0,
				   IPU_PSYS_NUM_DEVICES, IPU_PSYS_NAME);
	if (rval) {
		dev_err(dev, "can't alloc psys chrdev region (%d)\n",
			rval);
		return rval;
	}

	rval = pm_runtime_resume_and_get(&auxdev->dev);
	if (rval < 0)
		return rval;

	rval = ipu7_mmu_hw_init(adev->mmu);
	if (rval)
		goto out_unregister_chr_region;

	mutex_lock(&ipu7_psys_mutex);

	minor = find_next_zero_bit(ipu7_psys_devices, IPU_PSYS_NUM_DEVICES, 0);
	if (minor == IPU_PSYS_NUM_DEVICES) {
		dev_err(dev, "too many devices\n");
		goto out_unlock;
	}

	psys = devm_kzalloc(dev, sizeof(*psys), GFP_KERNEL);
	if (!psys) {
		rval = -ENOMEM;
		goto out_unlock;
	}

	for (i = 0 ; i < IPU_PSYS_NUM_STREAMS; i++)
		psys->graph_id[i] = INVALID_STREAM_ID;

	adev->auxdrv_data =
		(const struct ipu7_auxdrv_data *)auxdev_id->driver_data;
	adev->auxdrv = to_auxiliary_drv(dev->driver);

	psys->adev = adev;
	psys->pdata = adev->pdata;
	psys->icache_prefetch_sp = 0;

	psys->power_gating = 0;

	cdev_init(&psys->cdev, &ipu7_psys_fops);
	psys->cdev.owner = ipu7_psys_fops.owner;

	rval = cdev_add(&psys->cdev, MKDEV(MAJOR(ipu7_psys_dev_t), minor), 1);
	if (rval) {
		dev_err(dev, "cdev_add failed (%d)\n", rval);
		goto out_unlock;
	}

	set_bit(minor, ipu7_psys_devices);

	spin_lock_init(&psys->ready_lock);

	psys->ready = 0;
	psys->timeout = IPU_PSYS_CMD_TIMEOUT_MS;

	mutex_init(&psys->mutex);
	INIT_LIST_HEAD(&psys->fhs);

	if (async_fw_init) {
		INIT_DELAYED_WORK((struct delayed_work *)&fw_init_task,
				  run_fw_init_work);
		fw_init_task.psys = psys;
		schedule_delayed_work((struct delayed_work *)&fw_init_task, 0);
	} else {
		rval = ipu7_fw_psys_init(psys);
	}

	if (rval) {
		dev_err(dev, "FW init failed(%d)\n", rval);
		goto out_mutex_destroy;
	}

	psys->dev.bus = &ipu7_psys_bus;
	psys->dev.parent = dev;
	psys->dev.devt = MKDEV(MAJOR(ipu7_psys_dev_t), minor);
	psys->dev.release = ipu7_psys_dev_release;
	dev_set_name(&psys->dev, "ipu7-psys%d", minor);
	rval = device_register(&psys->dev);
	if (rval < 0) {
		dev_err(&psys->dev, "psys device_register failed\n");
		goto out_fw_release;
	}

#ifdef CONFIG_DEBUG_FS
	psys_fw_log_init(psys);

#endif
	/* Add the hw stepping information to caps */
	strscpy(psys->caps.dev_model, IPU_MEDIA_DEV_MODEL_NAME,
		sizeof(psys->caps.dev_model));

	dev_set_drvdata(dev, psys);

	mutex_unlock(&ipu7_psys_mutex);

#ifdef CONFIG_DEBUG_FS
	/* Debug fs failure is not fatal. */
	ipu7_psys_init_debugfs(psys);
#endif

	dev_info(dev, "psys probe minor: %d\n", minor);

	ipu7_mmu_hw_cleanup(adev->mmu);
	pm_runtime_put(&auxdev->dev);

	return 0;

out_fw_release:
	ipu7_fw_psys_release(psys);
out_mutex_destroy:
	mutex_destroy(&psys->mutex);
	cdev_del(&psys->cdev);
out_unlock:
	/* Safe to call even if the init is not called */
	mutex_unlock(&ipu7_psys_mutex);

	ipu7_mmu_hw_cleanup(adev->mmu);

out_unregister_chr_region:
	unregister_chrdev_region(ipu7_psys_dev_t, IPU_PSYS_NUM_DEVICES);
	pm_runtime_put(&auxdev->dev);

	return rval;
}

static void ipu7_psys_remove(struct auxiliary_device *auxdev)
{
	struct ipu7_psys *psys = dev_get_drvdata(&auxdev->dev);
	struct device *dev = &auxdev->dev;
#ifdef CONFIG_DEBUG_FS
	struct ipu7_device *isp = psys->adev->isp;

	if (isp->ipu7_dir)
		debugfs_remove_recursive(psys->debugfsdir);
#endif

	mutex_lock(&ipu7_psys_mutex);
	ipu7_fw_psys_release(psys);
	device_unregister(&psys->dev);
	clear_bit(MINOR(psys->cdev.dev), ipu7_psys_devices);
	cdev_del(&psys->cdev);
	mutex_unlock(&ipu7_psys_mutex);

	mutex_destroy(&psys->mutex);

	unregister_chrdev_region(ipu7_psys_dev_t, IPU_PSYS_NUM_DEVICES);

	dev_info(dev, "removed\n");
}

static irqreturn_t psys_isr_threaded(struct ipu7_bus_device *adev)
{
	struct ipu7_psys *psys = ipu7_bus_get_drvdata(adev);
	struct device *dev = &psys->adev->auxdev.dev;
	void __iomem *base = psys->pdata->base;
	u32 status, state;
	int r;

	mutex_lock(&psys->mutex);
	r = pm_runtime_get_if_in_use(dev);
	if (!r || WARN_ON_ONCE(r < 0)) {
		mutex_unlock(&psys->mutex);
		return IRQ_NONE;
	}

	state = ipu7_boot_get_boot_state(adev);
	if (IA_GOFO_FW_BOOT_STATE_IS_CRITICAL(state)) {
		//TODO: Add log parser
		dev_warn(&psys->dev, "error state %u\n", state);
	} else {
		status = readl(base + IPU_REG_PSYS_TO_SW_IRQ_CNTL_STATUS);
		writel(status, base + IPU_REG_PSYS_TO_SW_IRQ_CNTL_CLEAR);

		if (status & IRQ_FROM_LOCAL_FW)
			ipu7_psys_handle_events(psys);
	}

	pm_runtime_put(dev);
	mutex_unlock(&psys->mutex);

	return IRQ_HANDLED;
}

static const struct ipu7_auxdrv_data ipu7_psys_auxdrv_data = {
	.isr_threaded = psys_isr_threaded,
	.wake_isr_thread = true,
};

static const struct auxiliary_device_id ipu7_psys_id_table[] = {
	{
		.name = "intel_ipu7.psys",
		.driver_data = (kernel_ulong_t)&ipu7_psys_auxdrv_data,
	},
	{ }
};

MODULE_DEVICE_TABLE(auxiliary, ipu7_psys_id_table);

static struct auxiliary_driver ipu7_psys_driver = {
	.name = IPU_PSYS_NAME,
	.probe = ipu7_psys_probe,
	.remove = ipu7_psys_remove,
	.id_table = ipu7_psys_id_table,
	.driver = {
		.pm = PSYS_PM_OPS,
	},
};

module_auxiliary_driver(ipu7_psys_driver);

MODULE_AUTHOR("Bingbu Cao <bingbu.cao@intel.com>");
MODULE_AUTHOR("Qingwu Zhang <qingwu.zhang@intel.com>");
MODULE_AUTHOR("Tianshu Qiu <tian.shu.qiu@intel.com>");

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Intel ipu7 processing system driver");
MODULE_IMPORT_NS(INTEL_IPU7);
MODULE_IMPORT_NS(DMA_BUF);
