// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2013 - 2024 Intel Corporation

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/string.h>
#include <linux/sched.h>
#include <linux/version.h>

#include <media/ipu-isys.h>
#include <media/v4l2-mc.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-async.h>

#include "ipu.h"
#include "ipu-bus.h"
#include "ipu-cpd.h"
#include "ipu-mmu.h"
#include "ipu-dma.h"
#include "ipu-isys.h"
#include "ipu-isys-csi2.h"
#include "ipu-isys-csi2-regs.h"
#ifdef CONFIG_VIDEO_INTEL_IPU_MGC
#include "ipu-isys-tpg.h"
#endif
#include "ipu-isys-video.h"
#include "ipu-platform-regs.h"
#include "ipu-buttress.h"
#include "ipu-platform.h"
#include "ipu-buttress-regs.h"

#define ISYS_PM_QOS_VALUE	300

static int
isys_complete_ext_device_registration(struct ipu_isys *isys,
				      struct v4l2_subdev *sd,
				      struct ipu_isys_csi2_config *csi2)
{
	unsigned int i;
	int ret;

	v4l2_set_subdev_hostdata(sd, csi2);

	for (i = 0; i < sd->entity.num_pads; i++) {
		if (sd->entity.pads[i].flags & MEDIA_PAD_FL_SOURCE)
			break;
	}

	if (i == sd->entity.num_pads) {
		dev_warn(&isys->adev->dev,
			 "no source pad in external entity\n");
		ret = -ENOENT;
		goto skip_unregister_subdev;
	}

	ret = media_create_pad_link(&sd->entity, i,
				    &isys->csi2[csi2->port].asd.sd.entity,
				    0, 0);
	if (ret) {
		dev_warn(&isys->adev->dev, "can't create link\n");
		goto skip_unregister_subdev;
	}

	isys->csi2[csi2->port].nlanes = csi2->nlanes;
	return 0;

skip_unregister_subdev:
	v4l2_device_unregister_subdev(sd);
	return ret;
}

static void isys_stream_init(struct ipu_isys *isys)
{
	for (int i = 0; i < IPU_ISYS_MAX_STREAMS; i++) {
		mutex_init(&isys->streams[i].mutex);
		init_completion(&isys->streams[i].stream_open_completion);
		init_completion(&isys->streams[i].stream_close_completion);
		init_completion(&isys->streams[i].stream_start_completion);
		init_completion(&isys->streams[i].stream_stop_completion);
		INIT_LIST_HEAD(&isys->streams[i].queues);
		isys->streams[i].isys = isys;
		isys->streams[i].stream_handle = i;
	}
}

static void isys_unregister_subdevices(struct ipu_isys *isys)
{
#ifdef CONFIG_VIDEO_INTEL_IPU_MGC
	const struct ipu_isys_internal_tpg_pdata *tpg =
	    &isys->pdata->ipdata->tpg;
#endif
	const struct ipu_isys_internal_csi2_pdata *csi2 =
	    &isys->pdata->ipdata->csi2;
	unsigned int i;

	for (i = 0; i < NR_OF_CSI2_BE_SOC_DEV; i++)
		ipu_isys_csi2_be_soc_cleanup(&isys->csi2_be_soc[i]);

#ifdef CONFIG_VIDEO_INTEL_IPU_MGC
	for (i = 0; i < tpg->ntpgs; i++)
		ipu_isys_tpg_cleanup(&isys->tpg[i]);
#endif

	for (i = 0; i < csi2->nports; i++)
		ipu_isys_csi2_cleanup(&isys->csi2[i]);
}

static int isys_register_subdevices(struct ipu_isys *isys)
{
#ifdef CONFIG_VIDEO_INTEL_IPU_MGC
	const struct ipu_isys_internal_tpg_pdata *tpg =
	    &isys->pdata->ipdata->tpg;
#endif
	const struct ipu_isys_internal_csi2_pdata *csi2 =
	    &isys->pdata->ipdata->csi2;
	struct ipu_isys_csi2_be_soc *csi2_be_soc;
	unsigned int i, k;
	int ret;

	isys->csi2 = devm_kcalloc(&isys->adev->dev, csi2->nports,
				  sizeof(*isys->csi2), GFP_KERNEL);
	if (!isys->csi2) {
		ret = -ENOMEM;
		goto fail;
	}

	for (i = 0; i < csi2->nports; i++) {
		ret = ipu_isys_csi2_init(&isys->csi2[i], isys,
					 isys->pdata->base +
					 csi2->offsets[i], i);
		if (ret)
			goto fail;
	}

	isys->isr_csi2_mask = IPU7_CSI_RX_LEGACY_IRQ_MASK;

#ifdef CONFIG_VIDEO_INTEL_IPU_MGC
	isys->tpg = devm_kcalloc(&isys->adev->dev, tpg->ntpgs,
				 sizeof(*isys->tpg), GFP_KERNEL);
	if (!isys->tpg) {
		ret = -ENOMEM;
		goto fail;
	}

	for (i = 0; i < tpg->ntpgs; i++) {
		ret = ipu_isys_tpg_init(&isys->tpg[i], isys,
					isys->pdata->base +
					tpg->offsets[i],
					tpg->sels ? (isys->pdata->base +
						     tpg->sels[i]) : NULL, i);
		if (ret)
			goto fail;
	}
#endif

	for (k = 0; k < NR_OF_CSI2_BE_SOC_DEV; k++) {
		ret = ipu_isys_csi2_be_soc_init(&isys->csi2_be_soc[k],
						isys, k);
		if (ret) {
			dev_info(&isys->adev->dev,
				 "can't register csi2 soc be device %d\n", k);
			goto fail;
		}
	}

	for (i = 0; i < csi2->nports; i++) {
		for (k = 0; k < NR_OF_CSI2_BE_SOC_DEV; k++) {
			csi2_be_soc = &isys->csi2_be_soc[k];
			ret =
			    media_create_pad_link(&isys->csi2[i].asd.sd.entity,
						  CSI2_PAD_SOURCE,
						  &csi2_be_soc->asd.sd.entity,
						  CSI2_BE_SOC_PAD_SINK, 0);
			if (ret) {
				dev_info(&isys->adev->dev,
					 "can't create link csi2->be_soc\n");
				goto fail;
			}
		}
	}

#ifdef CONFIG_VIDEO_INTEL_IPU_MGC
	for (i = 0; i < tpg->ntpgs; i++) {
		for (k = 0; k < NR_OF_CSI2_BE_SOC_DEV; k++) {
			csi2_be_soc = &isys->csi2_be_soc[k];
			ret =
			    media_create_pad_link(&isys->tpg[i].asd.sd.entity,
						  TPG_PAD_SOURCE,
						  &csi2_be_soc->asd.sd.entity,
						  CSI2_BE_SOC_PAD_SINK, 0);
			if (ret) {
				dev_info(&isys->adev->dev,
					 "can't create link tpg->be_soc\n");
				goto fail;
			}
		}
	}
#endif

	return 0;

fail:
	isys_unregister_subdevices(isys);
	return ret;
}

#define FW_LOG_BUF_SIZE  (2 * 1024 * 1024)
static int isys_fw_log_init(struct ipu_isys *isys)
{
	struct isys_fw_log *fw_log;
	void *log_buf;

	if (isys->fw_log)
		return 0;

	fw_log = devm_kzalloc(&isys->adev->dev, sizeof(*fw_log), GFP_KERNEL);
	if (!fw_log)
		return -ENOMEM;

	mutex_init(&fw_log->mutex);

	log_buf = devm_kzalloc(&isys->adev->dev, FW_LOG_BUF_SIZE, GFP_KERNEL);
	if (!log_buf)
		return -ENOMEM;

	fw_log->head = log_buf;
	fw_log->addr = log_buf;
	fw_log->count = 0;
	fw_log->size = 0;

	isys->fw_log = fw_log;

	return 0;
}

/* The .bound() notifier callback when a match is found */
static int isys_notifier_bound(struct v4l2_async_notifier *notifier,
			       struct v4l2_subdev *sd,
			       struct v4l2_async_subdev *asd)
{
	struct ipu_isys *isys = container_of(notifier,
					struct ipu_isys, notifier);
	struct sensor_async_subdev *s_asd =
		container_of(asd, struct sensor_async_subdev, asd);

	dev_info(&isys->adev->dev, "bind %s nlanes is %d port is %d\n",
		 sd->name, s_asd->csi2.nlanes, s_asd->csi2.port);
	isys_complete_ext_device_registration(isys, sd, &s_asd->csi2);

	return v4l2_device_register_subdev_nodes(&isys->v4l2_dev);
}

static int isys_notifier_complete(struct v4l2_async_notifier *notifier)
{
	struct ipu_isys *isys = container_of(notifier,
					struct ipu_isys, notifier);

	dev_info(&isys->adev->dev, "All sensor registration completed.\n");

	return v4l2_device_register_subdev_nodes(&isys->v4l2_dev);
}

static const struct v4l2_async_notifier_operations isys_async_ops = {
	.bound = isys_notifier_bound,
	.complete = isys_notifier_complete,
};

static int isys_fwnode_parse(struct device *dev,
			     struct v4l2_fwnode_endpoint *vep,
			     struct v4l2_async_subdev *asd)
{
	struct sensor_async_subdev *s_asd =
			container_of(asd, struct sensor_async_subdev, asd);

	s_asd->csi2.port = vep->base.port;
	s_asd->csi2.nlanes = vep->bus.mipi_csi2.num_data_lanes;

	return 0;
}

static int isys_notifier_init(struct ipu_isys *isys)
{
	struct ipu_device *isp = isys->adev->isp;
	size_t asd_struct_size = sizeof(struct sensor_async_subdev);
	int ret;

	v4l2_async_nf_init(&isys->notifier);
	ret = v4l2_async_nf_parse_fwnode_endpoints(&isp->pdev->dev,
						   &isys->notifier,
						   asd_struct_size,
						   isys_fwnode_parse);
	if (ret < 0) {
		dev_err(&isys->adev->dev,
			"v4l2 parse_fwnode_endpoints() failed: %d\n", ret);
		return ret;
	}
	if (list_empty(&isys->notifier.asd_list)) {
		/* isys probe could continue with async subdevs missing */
		dev_warn(&isys->adev->dev, "no subdev found in graph\n");
		return 0;
	}

	isys->notifier.ops = &isys_async_ops;
	ret = v4l2_async_nf_register(&isys->v4l2_dev, &isys->notifier);
	if (ret) {
		dev_err(&isys->adev->dev,
			"failed to register async notifier : %d\n", ret);
		v4l2_async_nf_cleanup(&isys->notifier);
	}

	return ret;
}

static void isys_notifier_cleanup(struct ipu_isys *isys)
{
	v4l2_async_nf_unregister(&isys->notifier);
	v4l2_async_nf_cleanup(&isys->notifier);
}

static struct media_device_ops isys_mdev_ops = {
	.link_notify = v4l2_pipeline_link_notify,
};

static int isys_register_devices(struct ipu_isys *isys)
{
	int ret;

	isys->media_dev.dev = &isys->adev->dev;
	isys->media_dev.ops = &isys_mdev_ops;
	strscpy(isys->media_dev.model,
		IPU_MEDIA_DEV_MODEL_NAME, sizeof(isys->media_dev.model));
	snprintf(isys->media_dev.bus_info, sizeof(isys->media_dev.bus_info),
		 "pci:%s", dev_name(isys->adev->dev.parent->parent));
	strscpy(isys->v4l2_dev.name, isys->media_dev.model,
		sizeof(isys->v4l2_dev.name));

	media_device_init(&isys->media_dev);

	ret = media_device_register(&isys->media_dev);
	if (ret < 0) {
		dev_info(&isys->adev->dev, "can't register media device\n");
		goto out_media_device_unregister;
	}

	isys->v4l2_dev.mdev = &isys->media_dev;

	ret = v4l2_device_register(&isys->adev->dev, &isys->v4l2_dev);
	if (ret < 0) {
		dev_info(&isys->adev->dev, "can't register v4l2 device\n");
		goto out_media_device_unregister;
	}

	ret = isys_register_subdevices(isys);
	if (ret)
		goto out_v4l2_device_unregister;

	ret = isys_notifier_init(isys);
	if (ret)
		goto out_isys_unregister_subdevices;

	ret = v4l2_device_register_subdev_nodes(&isys->v4l2_dev);
	if (ret)
		goto out_isys_notifier_cleanup;

	return 0;

out_isys_notifier_cleanup:
	isys_notifier_cleanup(isys);

out_isys_unregister_subdevices:
	isys_unregister_subdevices(isys);

out_v4l2_device_unregister:
	v4l2_device_unregister(&isys->v4l2_dev);

out_media_device_unregister:
	media_device_unregister(&isys->media_dev);
	media_device_cleanup(&isys->media_dev);

	return ret;
}

static void isys_unregister_devices(struct ipu_isys *isys)
{
	isys_unregister_subdevices(isys);
	v4l2_device_unregister(&isys->v4l2_dev);
	media_device_unregister(&isys->media_dev);
	media_device_cleanup(&isys->media_dev);
}

#ifdef CONFIG_PM
static void enable_csi2_legacy_irq(struct ipu_isys *isys, bool enable)
{
	u32 offset, mask;
	void __iomem *base = isys->pdata->base;

	offset = IS_IO_CSI2_LEGACY_IRQ_CTRL_BASE;
	mask = isys->isr_csi2_mask;

	if (!enable) {
		writel(0, base + offset + IRQ_CTL_ENABLE);
		return;
	}

	writel(mask, base + offset + IRQ_CTL_EDGE);
	writel(mask, base + offset + IRQ_CTL_CLEAR);
	writel(mask, base + offset + IRQ_CTL_MASK);
	writel(mask, base + offset + IRQ_CTL_ENABLE);
}

static void enable_to_sw_irq(struct ipu_isys *isys, bool enable)
{
	u32 offset, mask;
	void __iomem *base = isys->pdata->base;

	offset = IS_UC_CTRL_BASE;
	mask = IS_UC_TO_SW_IRQ_MASK;

	if (!enable) {
		writel(0, base + offset + TO_SW_IRQ_CNTL_ENABLE);
		return;
	}

	writel(mask, base + offset + TO_SW_IRQ_CNTL_CLEAR);
	writel(mask, base + offset + TO_SW_IRQ_CNTL_MASK_N);
	writel(mask, base + offset + TO_SW_IRQ_CNTL_ENABLE);
}

static void isys_setup_hw(struct ipu_isys *isys)
{
	u32 offset;
	void __iomem *base = isys->pdata->base;

	/* soft reset */
	offset = IS_IO_GPREGS_BASE;

	writel(0x0, base + offset + CLK_EN_TXCLKESC);
	/* Update if ISYS freq updated (0: 400/1, 1:400/2, 63:400/64) */
	writel(0x0, base + offset + CLK_DIV_FACTOR_IS_CLK);
	/* correct the initial printf configuration */
	writel(0x200, base + IS_UC_CTRL_BASE + PRINTF_AXI_CNTL);

	enable_to_sw_irq(isys, 1);
	enable_csi2_legacy_irq(isys, 1);
}

static void isys_cleanup_hw(struct ipu_isys *isys)
{
	enable_csi2_legacy_irq(isys, 0);
	enable_to_sw_irq(isys, 0);
}

static int isys_runtime_pm_resume(struct device *dev)
{
	struct ipu_bus_device *adev = to_ipu_bus_device(dev);
	struct ipu_device *isp = adev->isp;
	struct ipu_isys *isys = ipu_bus_get_drvdata(adev);
	unsigned long flags;
	int ret;

	if (!isys)
		return 0;

	ret = ipu_mmu_hw_init(adev->mmu);
	if (ret)
		return ret;

	cpu_latency_qos_update_request(&isys->pm_qos, ISYS_PM_QOS_VALUE);

	ret = ipu_buttress_start_tsc_sync(isp);
	if (ret)
		return ret;

	spin_lock_irqsave(&isys->power_lock, flags);
	isys->power = 1;
	spin_unlock_irqrestore(&isys->power_lock, flags);

	isys_setup_hw(isys);

	return 0;
}

static int isys_runtime_pm_suspend(struct device *dev)
{
	struct ipu_bus_device *adev = to_ipu_bus_device(dev);
	struct ipu_isys *isys = ipu_bus_get_drvdata(adev);
	unsigned long flags;

	if (!isys)
		return 0;

	isys_cleanup_hw(isys);

	spin_lock_irqsave(&isys->power_lock, flags);
	isys->power = 0;
	spin_unlock_irqrestore(&isys->power_lock, flags);

	mutex_lock(&isys->mutex);
	isys->reset_needed = false;
	mutex_unlock(&isys->mutex);

	cpu_latency_qos_update_request(&isys->pm_qos, PM_QOS_DEFAULT_VALUE);

	ipu_mmu_hw_cleanup(adev->mmu);

	return 0;
}

static int isys_suspend(struct device *dev)
{
	struct ipu_bus_device *adev = to_ipu_bus_device(dev);
	struct ipu_isys *isys = ipu_bus_get_drvdata(adev);

	/* If stream is open, refuse to suspend */
	if (isys->stream_opened)
		return -EBUSY;

	return 0;
}

static int isys_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops isys_pm_ops = {
	.runtime_suspend = isys_runtime_pm_suspend,
	.runtime_resume = isys_runtime_pm_resume,
	.suspend = isys_suspend,
	.resume = isys_resume,
};

#define ISYS_PM_OPS (&isys_pm_ops)
#else
#define ISYS_PM_OPS NULL
#endif

static void isys_remove(struct ipu_bus_device *adev)
{
	struct ipu_isys *isys = ipu_bus_get_drvdata(adev);
	struct isys_fw_msgs *fwmsg, *safe;

	dev_info(&adev->dev, "removed\n");
#ifdef CONFIG_DEBUG_FS
	if (adev->isp->ipu_dir)
		debugfs_remove_recursive(isys->debugfsdir);
#endif

	for (int i = 0; i < IPU_ISYS_MAX_STREAMS; i++)
		mutex_destroy(&isys->streams[i].mutex);

	list_for_each_entry_safe(fwmsg, safe, &isys->framebuflist, head) {
		dma_free_attrs(&adev->dev, sizeof(struct isys_fw_msgs),
			       fwmsg, fwmsg->dma_addr,
			       0);
	}

	list_for_each_entry_safe(fwmsg, safe, &isys->framebuflist_fw, head) {
		dma_free_attrs(&adev->dev, sizeof(struct isys_fw_msgs),
			       fwmsg, fwmsg->dma_addr,
			       0
		    );
	}

	isys_notifier_cleanup(isys);
	isys_unregister_devices(isys);

	cpu_latency_qos_remove_request(&isys->pm_qos);

	mutex_destroy(&isys->stream_mutex);
	mutex_destroy(&isys->mutex);
}

#ifdef CONFIG_DEBUG_FS
static int ipu_isys_icache_prefetch_get(void *data, u64 *val)
{
	struct ipu_isys *isys = data;

	*val = isys->icache_prefetch;
	return 0;
}

static int ipu_isys_icache_prefetch_set(void *data, u64 val)
{
	struct ipu_isys *isys = data;

	if (val != !!val)
		return -EINVAL;

	isys->icache_prefetch = val;

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(isys_icache_prefetch_fops,
			ipu_isys_icache_prefetch_get,
			ipu_isys_icache_prefetch_set, "%llu\n");

static ssize_t fwlog_read(struct file *file, char __user *userbuf, size_t size,
			  loff_t *pos)
{
	struct ipu_isys *isys = file->private_data;
	struct isys_fw_log *fw_log = isys->fw_log;
	struct device *dev = &isys->adev->dev;
	void *buf;
	int ret = 0;

	if (!fw_log)
		return 0;

	buf = kvzalloc(FW_LOG_BUF_SIZE, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	mutex_lock(&fw_log->mutex);
	if (!isys->fw_log->size) {
		dev_warn(dev, "no available fw log");
		mutex_unlock(&fw_log->mutex);
		goto free_and_return;
	}

	memcpy(buf, isys->fw_log->addr, isys->fw_log->size);
	dev_info(dev, "copy %d bytes fw log to user...", isys->fw_log->size);
	mutex_unlock(&fw_log->mutex);

	ret = simple_read_from_buffer(userbuf, size, pos, buf,
				      isys->fw_log->size);
free_and_return:
	kvfree(buf);

	return ret;
}

static const struct file_operations isys_fw_log_fops = {
	.open = simple_open,
	.owner = THIS_MODULE,
	.read = fwlog_read,
	.llseek = default_llseek,
};

static int ipu_isys_init_debugfs(struct ipu_isys *isys)
{
	struct dentry *file;
	struct dentry *dir;

	dir = debugfs_create_dir("isys", isys->adev->isp->ipu_dir);
	if (IS_ERR(dir))
		return -ENOMEM;

	file = debugfs_create_file("icache_prefetch", 0600,
				   dir, isys, &isys_icache_prefetch_fops);
	if (IS_ERR(file))
		goto err;

	file = debugfs_create_file("fwlog", 0400,
				   dir, isys, &isys_fw_log_fops);
	if (IS_ERR(file))
		goto err;

	isys->debugfsdir = dir;

	return 0;
err:
	debugfs_remove_recursive(dir);
	return -ENOMEM;
}
#endif

static int alloc_fw_msg_bufs(struct ipu_isys *isys, int amount)
{
	dma_addr_t dma_addr;
	struct isys_fw_msgs *addr;
	unsigned int i;
	unsigned long flags;

	for (i = 0; i < amount; i++) {
		addr = dma_alloc_attrs(&isys->adev->dev,
				       sizeof(struct isys_fw_msgs),
				       &dma_addr, GFP_KERNEL,
				       0);
		if (!addr)
			break;
		addr->dma_addr = dma_addr;

		spin_lock_irqsave(&isys->listlock, flags);
		list_add(&addr->head, &isys->framebuflist);
		spin_unlock_irqrestore(&isys->listlock, flags);
	}
	if (i == amount)
		return 0;
	spin_lock_irqsave(&isys->listlock, flags);
	while (!list_empty(&isys->framebuflist)) {
		addr = list_first_entry(&isys->framebuflist,
					struct isys_fw_msgs, head);
		list_del(&addr->head);
		spin_unlock_irqrestore(&isys->listlock, flags);
		dma_free_attrs(&isys->adev->dev,
			       sizeof(struct isys_fw_msgs),
			       addr, addr->dma_addr,
			       0);
		spin_lock_irqsave(&isys->listlock, flags);
	}
	spin_unlock_irqrestore(&isys->listlock, flags);
	return -ENOMEM;
}

struct isys_fw_msgs *ipu_get_fw_msg_buf(struct ipu_isys_stream *stream)
{
	struct ipu_isys *isys = stream->isys;
	struct isys_fw_msgs *msg;
	unsigned long flags;

	spin_lock_irqsave(&isys->listlock, flags);
	if (list_empty(&isys->framebuflist)) {
		spin_unlock_irqrestore(&isys->listlock, flags);
		dev_warn(&isys->adev->dev, "Frame list empty - Allocate more");

		alloc_fw_msg_bufs(isys, 5);

		spin_lock_irqsave(&isys->listlock, flags);
		if (list_empty(&isys->framebuflist)) {
			spin_unlock_irqrestore(&isys->listlock, flags);
			dev_err(&isys->adev->dev, "Frame list empty");
			return NULL;
		}
	}
	msg = list_last_entry(&isys->framebuflist, struct isys_fw_msgs, head);
	list_move(&msg->head, &isys->framebuflist_fw);
	spin_unlock_irqrestore(&isys->listlock, flags);
	memset(&msg->fw_msg, 0, sizeof(msg->fw_msg));

	return msg;
}

void ipu_cleanup_fw_msg_bufs(struct ipu_isys *isys)
{
	struct isys_fw_msgs *fwmsg, *fwmsg0;
	unsigned long flags;

	spin_lock_irqsave(&isys->listlock, flags);
	list_for_each_entry_safe(fwmsg, fwmsg0, &isys->framebuflist_fw, head)
		list_move(&fwmsg->head, &isys->framebuflist);
	spin_unlock_irqrestore(&isys->listlock, flags);
}

void ipu_put_fw_msg_buf(struct ipu_isys *isys, u64 data)
{
	struct isys_fw_msgs *msg;
	unsigned long flags;
	u64 *ptr = (u64 *)(unsigned long)data;

	if (WARN_ON_ONCE(!ptr))
		return;

	spin_lock_irqsave(&isys->listlock, flags);
	msg = container_of(ptr, struct isys_fw_msgs, fw_msg.dummy);
	list_move(&msg->head, &isys->framebuflist);
	spin_unlock_irqrestore(&isys->listlock, flags);
}

static int isys_probe(struct ipu_bus_device *adev)
{
	struct ipu_isys *isys;
	int ret = 0;

	isys = devm_kzalloc(&adev->dev, sizeof(*isys), GFP_KERNEL);
	if (!isys)
		return -ENOMEM;

	ret = ipu_mmu_hw_init(adev->mmu);
	if (ret)
		return ret;

	/* By default, short packet is captured from T-Unit. */
	isys->adev = adev;
	isys->pdata = adev->pdata;

	/* initial streamID for different sensor types */
	isys->sensor_info.vc1_data_start = IPU_FW_ISYS_VC1_SENSOR_DATA_START;
	isys->sensor_info.vc1_data_end = IPU_FW_ISYS_VC1_SENSOR_DATA_END;
	isys->sensor_info.vc0_data_start = IPU_FW_ISYS_VC0_SENSOR_DATA_START;
	isys->sensor_info.vc0_data_end = IPU_FW_ISYS_VC0_SENSOR_DATA_END;
	isys->sensor_info.vc1_pdaf_start = IPU_FW_ISYS_VC1_SENSOR_PDAF_START;
	isys->sensor_info.vc1_pdaf_end = IPU_FW_ISYS_VC1_SENSOR_PDAF_END;
	isys->sensor_info.sensor_metadata = IPU_FW_ISYS_SENSOR_METADATA;

	isys->sensor_types[IPU_ISYS_VC1_SENSOR_DATA] =
		IPU_FW_ISYS_VC1_SENSOR_DATA_START;
	isys->sensor_types[IPU_ISYS_VC1_SENSOR_PDAF] =
		IPU_FW_ISYS_VC1_SENSOR_PDAF_START;
	isys->sensor_types[IPU_ISYS_VC0_SENSOR_DATA] =
		IPU_FW_ISYS_VC0_SENSOR_DATA_START;

	INIT_LIST_HEAD(&isys->requests);

	spin_lock_init(&isys->streams_lock);
	spin_lock_init(&isys->power_lock);
	isys->power = 0;

	mutex_init(&isys->mutex);
	mutex_init(&isys->stream_mutex);

	spin_lock_init(&isys->listlock);
	INIT_LIST_HEAD(&isys->framebuflist);
	INIT_LIST_HEAD(&isys->framebuflist_fw);

	ipu_bus_set_drvdata(adev, isys);

	isys->line_align = IPU_ISYS_2600_MEM_LINE_ALIGN;
	isys->icache_prefetch = 0;
	isys->phy_rext_cal = 0;

	isys_stream_init(isys);
#ifndef CONFIG_PM
	isys_setup_hw(isys);
#endif

#ifdef CONFIG_DEBUG_FS
	/* Debug fs failure is not fatal. */
	ipu_isys_init_debugfs(isys);
#endif

	cpu_latency_qos_add_request(&isys->pm_qos, PM_QOS_DEFAULT_VALUE);
	alloc_fw_msg_bufs(isys, 20);

	ret = ipu_fw_isys_init(isys);
	if (ret)
		goto out_cleanup;

	ret = isys_register_devices(isys);
	if (ret)
		goto out_cleanup;

	ret = isys_fw_log_init(isys);
	if (ret)
		goto out_cleanup;

	ipu_mmu_hw_cleanup(adev->mmu);

	return 0;

out_cleanup:
	isys_unregister_devices(isys);
	ipu_fw_isys_release(isys);

	for (int i = 0; i < IPU_ISYS_MAX_STREAMS; i++)
		mutex_destroy(&isys->streams[i].mutex);

	mutex_destroy(&isys->mutex);
	mutex_destroy(&isys->stream_mutex);

	ipu_mmu_hw_cleanup(adev->mmu);

	return ret;
}

struct fwmsg {
	int type;
	char *msg;
	bool valid_ts;
};

static const struct fwmsg fw_msg[] = {
	{IPU_INSYS_RESP_TYPE_STREAM_OPEN_DONE, "STREAM_OPEN_DONE", 0},
	{IPU_INSYS_RESP_TYPE_STREAM_CLOSE_ACK, "STREAM_CLOSE_ACK", 0},
	{IPU_INSYS_RESP_TYPE_STREAM_START_AND_CAPTURE_ACK,
	 "STREAM_START_AND_CAPTURE_ACK", 0},
	{IPU_INSYS_RESP_TYPE_STREAM_ABORT_ACK, "STREAM_ABORT_ACK", 0},
	{IPU_INSYS_RESP_TYPE_STREAM_FLUSH_ACK, "STREAM_FLUSH_ACK", 0},
	{IPU_INSYS_RESP_TYPE_PIN_DATA_READY, "PIN_DATA_READY", 1},
	{IPU_INSYS_RESP_TYPE_STREAM_CAPTURE_ACK, "STREAM_CAPTURE_ACK", 0},
	{IPU_INSYS_RESP_TYPE_STREAM_START_AND_CAPTURE_DONE,
	 "STREAM_START_AND_CAPTURE_DONE", 1},
	{IPU_INSYS_RESP_TYPE_STREAM_CAPTURE_DONE, "STREAM_CAPTURE_DONE", 1},
	{IPU_INSYS_RESP_TYPE_FRAME_SOF, "FRAME_SOF", 1},
	{IPU_INSYS_RESP_TYPE_FRAME_EOF, "FRAME_EOF", 1},
	{-1, "UNKNOWN MESSAGE", 0},
};

struct ipu7_csi2_error {
	const char *error_string;
	bool is_info_only;
};

/*
 * Strings corresponding to CSI-2 receiver errors are here.
 * Corresponding macros are defined in the header file.
 */
static struct ipu7_csi2_error dphy_rx_errors[] = {
	{ "Error handler FIFO full", false },
	{ "Reserved Short Packet encoding detected", true },
	{ "Reserved Long Packet encoding detected", true },
	{ "Received packet is too short", false},
	{ "Received packet is too long", false},
	{ "Short packet discarded due to errors", false },
	{ "Long packet discarded due to errors", false },
	{ "CSI Combo Rx interrupt", false },
	{ "IDI CDC FIFO overflow(remaining bits are reserved as 0)", false },
	{ "Received NULL packet", true },
	{ "Received blanking packet", true },
	{ "Tie to 0", true },
	{ }
};

static void ipu7_isys_register_errors(struct ipu_isys_csi2 *csi2)
{
	int mask = IPU7_CSI_RX_ERROR_IRQ_MASK;
	u32 offset = IS_IO_CSI2_ERR_LEGACY_IRQ_CTL_BASE(csi2->index);
	u32 status = readl(csi2->base + offset + IRQ_CTL_STATUS);

	if (!status)
		return;

	dev_dbg(&csi2->isys->adev->dev, "csi2-%u error status 0x%08x",
		csi2->index, status);

	writel(status & mask, csi2->base + offset + IRQ_CTL_CLEAR);
	csi2->receiver_errors |= status & mask;
}

static void ipu_isys_csi2_error(struct ipu_isys_csi2 *csi2)
{
	struct ipu7_csi2_error *errors;
	u32 status;
	unsigned int i;

	/* Register errors once more in case of error interrupts are disabled */
	ipu7_isys_register_errors(csi2);
	status = csi2->receiver_errors;
	csi2->receiver_errors = 0;
	errors = dphy_rx_errors;

	for (i = 0; i < CSI_RX_NUM_ERRORS_IN_IRQ; i++) {
		if (status & BIT(i))
			dev_err_ratelimited(&csi2->isys->adev->dev,
					    "csi2-%i error: %s\n",
					    csi2->index,
					    errors[i].error_string);
	}
}

int isys_isr_one(struct ipu_bus_device *adev)
{
	struct ipu_isys *isys = ipu_bus_get_drvdata(adev);
	struct ipu_insys_resp resp_data;
	struct ipu_insys_resp *resp;
	struct ipu_isys_stream *stream = NULL;
	struct ia_gofo_msg_err err_info;
	struct device *dev = &adev->dev;
	u64 ts;

	if (!isys->adev->syscom)
		return 1;

#ifdef ENABLE_FW_OFFLINE_LOGGER
	ipu_fw_isys_get_log(isys);
#endif

	resp = ipu_fw_isys_get_resp(isys, &resp_data);
	if (!resp)
		return 1;

	err_info = resp->error_info;
	ts = ((u64)resp->timestamp[1] << 32) | resp->timestamp[0];
	if (err_info.err_group == INSYS_MSG_ERR_GROUP_CAPTURE &&
	    err_info.err_code == INSYS_MSG_ERR_CAPTURE_SYNC_FRAME_DROP) {
		/* receive a sp w/o command, firmware drop it */
		dev_dbg(dev, "FRAME DROP: %02u %s stream %u\n",
			resp->type, is_fw_msg[resp->type].msg,
			resp->stream_id);
		dev_dbg(dev, "\tpin %u buf_id %llx frame %u\n",
			resp->pin_id, resp->buf_id, resp->frame_id);
		dev_dbg(dev, "\terror group %u code %u details [%u %u]\n",
			err_info.err_group, err_info.err_code,
			err_info.err_detail[0], err_info.err_detail[1]);
	} else if (!IA_GOFO_MSG_ERR_IS_OK(err_info)) {
		dev_err(dev, "%02u %s stream %u pin %u buf_id %llx frame %u\n",
			resp->type, is_fw_msg[resp->type].msg, resp->stream_id,
			resp->pin_id, resp->buf_id, resp->frame_id);
		dev_err(dev, "\terror group %u code %u details [%u %u]\n",
			err_info.err_group, err_info.err_code,
			err_info.err_detail[0], err_info.err_detail[1]);
	} else {
		dev_dbg(dev, "%02u %s stream %u pin %u buf_id %llx frame %u\n",
			resp->type, is_fw_msg[resp->type].msg, resp->stream_id,
			resp->pin_id, resp->buf_id, resp->frame_id);
		dev_dbg(dev, "\tts %llu\n", ts);
	}

	if (resp->stream_id >= IPU_ISYS_MAX_STREAMS) {
		dev_err(dev, "bad stream handle %u\n",
			resp->stream_id);
		goto leave;
	}

	stream = ipu_isys_query_stream_by_handle(isys, resp->stream_id);
	if (!stream) {
		dev_err(&adev->dev, "stream of stream_handle %u is unused\n",
			resp->stream_id);
		goto leave;
	}

	/* TODO: stream->error should be modified */
	stream->error = err_info.err_code;

	switch (resp->type) {
	case IPU_INSYS_RESP_TYPE_STREAM_OPEN_DONE:
		complete(&stream->stream_open_completion);
		break;
	case IPU_INSYS_RESP_TYPE_STREAM_CLOSE_ACK:
		complete(&stream->stream_close_completion);
		break;
	case IPU_INSYS_RESP_TYPE_STREAM_START_AND_CAPTURE_ACK:
		complete(&stream->stream_start_completion);
		break;
	case IPU_INSYS_RESP_TYPE_STREAM_ABORT_ACK:
		complete(&stream->stream_stop_completion);
		break;
	case IPU_INSYS_RESP_TYPE_STREAM_FLUSH_ACK:
		complete(&stream->stream_stop_completion);
		break;
	case IPU_INSYS_RESP_TYPE_PIN_DATA_READY:
		/*
		 * firmware only release the capture msg until software
		 * get pin_data_ready event
		 */
		dev_dbg(dev, "pin user_token = %llx\n", resp->pin.user_token);
		ipu_put_fw_msg_buf(ipu_bus_get_drvdata(adev),
				   resp->pin.user_token);
		if (resp->pin_id < IPU_INSYS_OUTPUT_PINS &&
		    stream->output_pins[resp->pin_id].pin_ready)
			stream->output_pins[resp->pin_id].pin_ready(stream,
								    resp);
		else
			dev_err(dev, "No handler for pin %u ready\n",
				resp->pin_id);
		if (stream->csi2)
			ipu_isys_csi2_error(stream->csi2);

		break;
	case IPU_INSYS_RESP_TYPE_STREAM_CAPTURE_ACK:
		break;
	case IPU_INSYS_RESP_TYPE_STREAM_START_AND_CAPTURE_DONE:
	case IPU_INSYS_RESP_TYPE_STREAM_CAPTURE_DONE:
		break;
	case IPU_INSYS_RESP_TYPE_FRAME_SOF:
		if (stream->csi2)
			ipu_isys_csi2_sof_event(stream->csi2);

#ifdef CONFIG_VIDEO_INTEL_IPU_MGC
#ifdef IPU_TPG_FRAME_SYNC
		if (stream->tpg)
			ipu_isys_tpg_sof_event(stream->tpg);
#endif
#endif
		stream->seq[stream->seq_index].sequence =
		    atomic_read(&stream->sequence) - 1;
		stream->seq[stream->seq_index].timestamp = ts;
		dev_dbg(dev,
			"SOF: stream %u frame %u (index %u), ts 0x%16.16llx\n",
			resp->stream_id, resp->frame_id,
			stream->seq[stream->seq_index].sequence, ts);
			stream->seq_index = (stream->seq_index + 1)
		    % IPU_ISYS_MAX_PARALLEL_SOF;
		break;
	case IPU_INSYS_RESP_TYPE_FRAME_EOF:
		if (stream->csi2)
			ipu_isys_csi2_eof_event(stream->csi2);

#ifdef CONFIG_VIDEO_INTEL_IPU_MGC
#ifdef IPU_TPG_FRAME_SYNC
		if (stream->tpg)
			ipu_isys_tpg_eof_event(stream->tpg);
#endif
#endif

		dev_dbg(dev, "eof: stream %d(index %u) ts 0x%16.16llx\n",
			resp->stream_id,
			stream->seq[stream->seq_index].sequence, ts);
		break;
	default:
		dev_err(dev, "Unknown response type %u stream %u\n",
			resp->type, resp->stream_id);
		break;
	}

	ipu_isys_put_stream(stream);
leave:
	ipu_fw_isys_put_resp(isys->adev->syscom, IPU_INSYS_OUTPUT_MSG_QUEUE);
	return 0;
}

static void ipu_isys_csi2_isr(struct ipu_isys_csi2 *csi2)
{
	u32 status, offset;
	struct ipu_device *isp = csi2->isys->adev->isp;

	ipu7_isys_register_errors(csi2);

	offset = IS_IO_CSI2_SYNC_LEGACY_IRQ_CTL_BASE(csi2->index);
	status = readl(csi2->base + offset + IRQ_CTL_STATUS);
	dev_dbg(&csi2->isys->adev->dev, "csi2-%u sync status 0x%08x",
		csi2->index, status);

	writel(status, csi2->base + offset + IRQ_CTL_CLEAR);
	if (status & IPU_CSI_RX_SYNC_FS_VC)
		ipu_isys_csi2_sof_event(csi2);

	if (is_ipu7p5(isp->hw_ver)) {
		status = readl(csi2->base + offset + IRQ1_CTL_STATUS);
		writel(status, csi2->base + offset + IRQ1_CTL_CLEAR);
		if (status & IPU7P5_CSI_RX_SYNC_FE_VC)
			ipu_isys_csi2_eof_event(csi2);

		return;
	}

	if (status & IPU_CSI_RX_SYNC_FE_VC)
		ipu_isys_csi2_eof_event(csi2);
}

irqreturn_t isys_isr(struct ipu_bus_device *adev)
{
	struct ipu_isys *isys = ipu_bus_get_drvdata(adev);
	void __iomem *base = isys->pdata->base;
	u32 status_csi, status_sw, csi_offset, sw_offset;

	spin_lock(&isys->power_lock);
	if (!isys->power) {
		spin_unlock(&isys->power_lock);
		return IRQ_NONE;
	}

	csi_offset = IS_IO_CSI2_LEGACY_IRQ_CTRL_BASE;
	sw_offset = IS_BASE;

	status_csi = readl(base + csi_offset + IRQ_CTL_STATUS);
	status_sw = readl(base + sw_offset + TO_SW_IRQ_CNTL_STATUS);
	if (status_csi)
		dev_dbg(&isys->adev->dev, "status csi 0x%08x\n", status_csi);
	if (status_sw)
		dev_dbg(&isys->adev->dev, "status to_sw 0x%08x\n", status_sw);

	do {
		writel(status_sw, base + sw_offset + TO_SW_IRQ_CNTL_CLEAR);
		writel(status_csi, base + csi_offset + IRQ_CTL_CLEAR);

		if (isys->isr_csi2_mask & status_csi) {
			unsigned int i;

			for (i = 0; i < isys->pdata->ipdata->csi2.nports; i++) {
				/* irq from not enabled port */
				if (!isys->csi2[i].base)
					continue;
				if (status_csi & isys->csi2[i].legacy_irq_mask)
					ipu_isys_csi2_isr(&isys->csi2[i]);
			}
		}

		if (!isys_isr_one(adev))
			status_sw = TO_SW_IRQ_FW;
		else
			status_sw = 0;

		status_csi = readl(base + csi_offset + IRQ_CTL_STATUS);
		status_sw |= readl(base + sw_offset + TO_SW_IRQ_CNTL_STATUS);
	} while ((status_csi & isys->isr_csi2_mask) ||
		 (status_sw & TO_SW_IRQ_FW));

	writel(TO_SW_IRQ_MASK, base + sw_offset + TO_SW_IRQ_CNTL_MASK_N);

	spin_unlock(&isys->power_lock);

	return IRQ_HANDLED;
}

static struct ipu_bus_driver isys_driver = {
	.probe = isys_probe,
	.remove = isys_remove,
	.isr = isys_isr,
	.wanted = IPU_ISYS_NAME,
	.drv = {
		.name = IPU_ISYS_NAME,
		.owner = THIS_MODULE,
		.pm = ISYS_PM_OPS,
	},
};

module_ipu_bus_driver(isys_driver);

static const struct pci_device_id ipu_pci_tbl[] = {
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, IPU7_PCI_ID)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, IPU7P5_PCI_ID)},
	{0,}
};
MODULE_DEVICE_TABLE(pci, ipu_pci_tbl);

MODULE_AUTHOR("Bingbu Cao <bingbu.cao@intel.com>");
MODULE_AUTHOR("Tianshu Qiu <tian.shu.qiu@intel.com>");
MODULE_AUTHOR("Qingwu Zhang <qingwu.zhang@intel.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Intel ipu input system driver");
MODULE_IMPORT_NS(INTEL_IPU_BRIDGE);
