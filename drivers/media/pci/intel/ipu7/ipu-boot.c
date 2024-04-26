// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2022 - 2024 Intel Corporation

#include <linux/delay.h>
#include <linux/iopoll.h>

#include "ipu.h"
#include "ipu-boot.h"
#include "ipu-buttress-regs.h"
#include "ipu-platform-regs.h"
#include "ia_gofo_boot_state_abi.h"
#include "ia_gofo_boot_config_abi.h"

#define IPU_FW_START_STOP_TIMEOUT		2000
#define IPU_BOOT_CELL_RESET_TIMEOUT		(2 * USEC_PER_SEC)
#define BOOT_STATE_IS_CRITICAL(s)	IA_GOFO_FW_BOOT_STATE_IS_CRITICAL(s)
#define BOOT_STATE_IS_READY(s)		((s) == IA_GOFO_FW_BOOT_STATE_READY)
#define BOOT_STATE_IS_INACTIVE(s)	((s) == IA_GOFO_FW_BOOT_STATE_INACTIVE)

struct ipu_boot_context {
	u32 base;
	u32 dmem_address;
	u32 status_ctrl_reg;
	u32 fw_start_address_reg;
	u32 fw_code_base_reg;
};

static const struct ipu_boot_context contexts[IPU_SUBSYS_NUM] = {
	{
		/* ISYS */
		.dmem_address = IPU_ISYS_DMEM_OFFSET,
		.status_ctrl_reg = BUTTRESS_REG_DRV_IS_UCX_CONTROL_STATUS,
		.fw_start_address_reg = BUTTRESS_REG_DRV_IS_UCX_START_ADDR,
		.fw_code_base_reg = IS_UC_CTRL_BASE
	},
	{
		/* PSYS */
		.dmem_address = IPU_PSYS_DMEM_OFFSET,
		.status_ctrl_reg = BUTTRESS_REG_DRV_PS_UCX_CONTROL_STATUS,
		.fw_start_address_reg = BUTTRESS_REG_DRV_PS_UCX_START_ADDR,
		.fw_code_base_reg = PS_UC_CTRL_BASE
	}
};

static u32 get_fw_boot_reg_addr(const struct ipu_bus_device *adev,
				enum ia_gofo_buttress_reg_id reg)
{
	u32 base = (adev->subsys == IPU_IS) ? 0 : IA_GOFO_FW_BOOT_ID_MAX;

	return BUTTRESS_FW_BOOT_PARAMS_ENTRY(base + reg);
}

static void write_fw_boot_param(const struct ipu_bus_device *adev,
				enum ia_gofo_buttress_reg_id reg,
				u32 val)
{
	void __iomem *base = adev->isp->base;

	dev_dbg(&adev->dev, "write boot param reg: %d addr: %x val: 0x%x\n",
		reg, get_fw_boot_reg_addr(adev, reg), val);
	writel(val, base + get_fw_boot_reg_addr(adev, reg));
}

static u32 read_fw_boot_param(const struct ipu_bus_device *adev,
			      enum ia_gofo_buttress_reg_id reg)
{
	void __iomem *base = adev->isp->base;

	return readl(base + get_fw_boot_reg_addr(adev, reg));
}

static int ipu_boot_cell_reset(const struct ipu_bus_device *adev)
{
	const struct ipu_boot_context *ctx = &contexts[adev->subsys];
	void __iomem *base = adev->isp->base;
	u32 ucx_ctrl_status = ctx->status_ctrl_reg;
	u32 timeout = IPU_BOOT_CELL_RESET_TIMEOUT;
	u32 val, val2;
	int ret;

	dev_dbg(&adev->dev, "cell enter reset...\n");
	val = readl(base + ucx_ctrl_status);
	dev_dbg(&adev->dev, "cell_ctrl_reg addr = 0x%x, val = 0x%x\n",
		ucx_ctrl_status, val);

	dev_dbg(&adev->dev, "force cell reset...\n");
	val |= UCX_CTL_RESET;
	val &= ~UCX_CTL_RUN;

	dev_dbg(&adev->dev, "write status_ctrl_reg(0x%x) to 0x%x\n",
		ucx_ctrl_status, val);
	writel(val, base + ucx_ctrl_status);

	ret = readl_poll_timeout(base + ucx_ctrl_status, val2,
				 (val2 & 0x3) == (val & 0x3), 100, timeout);
	if (ret) {
		dev_err(&adev->dev, "cell enter reset timeout. status: 0x%x\n",
			val2);
		return -ETIMEDOUT;
	}

	dev_dbg(&adev->dev, "cell exit reset...\n");
	val = readl(base + ucx_ctrl_status);
	WARN((!(val & UCX_CTL_RESET) || val & UCX_CTL_RUN),
	     "cell status 0x%x", val);

	val &= ~(UCX_CTL_RESET | UCX_CTL_RUN);
	dev_dbg(&adev->dev, "write status_ctrl_reg(0x%x) to 0x%x\n",
		ucx_ctrl_status, val);
	writel(val, base + ucx_ctrl_status);

	ret = readl_poll_timeout(base + ucx_ctrl_status, val2,
				 (val2 & 0x3) == (val & 0x3), 100, timeout);
	if (ret) {
		dev_err(&adev->dev, "cell exit reset timeout. status: 0x%x\n",
			val2);
		return -ETIMEDOUT;
	}

	return 0;
}

static void ipu_boot_cell_start(const struct ipu_bus_device *adev)
{
	const struct ipu_boot_context *ctx = &contexts[adev->subsys];
	void __iomem *base = adev->isp->base;
	u32 val;

	dev_dbg(&adev->dev, "starting cell...\n");
	val = readl(base + ctx->status_ctrl_reg);
	WARN_ON(val & (UCX_CTL_RESET | UCX_CTL_RUN));

	val &= ~UCX_CTL_RESET;
	val |= UCX_CTL_RUN;
	dev_dbg(&adev->dev, "write status_ctrl_reg(0x%x) to 0x%x\n",
		ctx->status_ctrl_reg, val);
	writel(val, base + ctx->status_ctrl_reg);
}

static void ipu_boot_cell_stop(const struct ipu_bus_device *adev)
{
	const struct ipu_boot_context *ctx = &contexts[adev->subsys];
	void __iomem *base = adev->isp->base;
	u32 val;

	dev_dbg(&adev->dev, "stopping cell...\n");

	val = readl(base + ctx->status_ctrl_reg);
	val &= ~UCX_CTL_RUN;
	dev_dbg(&adev->dev, "write status_ctrl_reg(0x%x) to 0x%x\n",
		ctx->status_ctrl_reg, val);
	writel(val, base + ctx->status_ctrl_reg);

	/* Wait for uC transactions complete */
	usleep_range(10, 20);

	val = readl(base + ctx->status_ctrl_reg);
	val |= UCX_CTL_RESET;
	dev_dbg(&adev->dev, "write status_ctrl_reg(0x%x) to 0x%x\n",
		ctx->status_ctrl_reg, val);
	writel(val, base + ctx->status_ctrl_reg);
}

static int ipu_boot_cell_init(const struct ipu_bus_device *adev)
{
	const struct ipu_boot_context *ctx = &contexts[adev->subsys];
	void __iomem *base = adev->isp->base;

	dev_dbg(&adev->dev, "write fw_start_address_reg(0x%x) to 0x%x\n",
		ctx->fw_start_address_reg, adev->fw_entry);
	writel(adev->fw_entry, base + ctx->fw_start_address_reg);

	return ipu_boot_cell_reset(adev);
}

static void init_boot_config(struct ia_gofo_boot_config *boot_config,
			     u32 length)
{
	/* syscom version, new syscom2 version */
	boot_config->length = length;
	boot_config->config_version.major = 1U;
	boot_config->config_version.minor = 0U;
	boot_config->config_version.subminor = 0U;
	boot_config->config_version.patch = 0U;

	/* msg version for task interface */
	boot_config->client_version_support.num_versions = 1U;
	boot_config->client_version_support.versions[0].major = 1U;
	boot_config->client_version_support.versions[0].minor = 0U;
	boot_config->client_version_support.versions[0].subminor = 0U;
	boot_config->client_version_support.versions[0].patch = 0U;
}

int ipu_boot_init_boot_config(struct ipu_bus_device *adev,
			      struct syscom_queue_config *qconfigs,
			      int num_queues, u32 uc_freq,
			      dma_addr_t subsys_config)
{
	struct ipu_syscom_context *syscom = adev->syscom;
	struct syscom_config_s *syscfg;
	struct ia_gofo_boot_config *boot_config;
	struct syscom_queue_params_config *cfgs;
	dma_addr_t queue_mem_dma_ptr;
	void *queue_mem_ptr;
	u32 total_queue_size = 0, total_queue_size_aligned = 0;
	unsigned int i;

	dev_dbg(&adev->dev,
		"init boot config. queues_nr: %d freq: %u sys_conf: 0x%llx\n",
		num_queues, uc_freq, subsys_config);
	/* Allocate boot config. */
	adev->boot_config_size = FW_BOOT_CONFIG_ALLOC_SIZE(num_queues);
	adev->boot_config = dma_alloc_attrs(&adev->dev, adev->boot_config_size,
					    &adev->boot_config_dma_addr,
					    GFP_KERNEL, 0);
	if (!adev->boot_config) {
		dev_err(&adev->dev, "Failed to allocate boot config.\n");
		return -ENOMEM;
	}
	dev_dbg(&adev->dev, "boot config dma addr: 0x%llx\n",
		adev->boot_config_dma_addr);
	boot_config = adev->boot_config;
	memset(boot_config, 0, sizeof(struct ia_gofo_boot_config));
	init_boot_config(boot_config, adev->boot_config_size);
	boot_config->subsys_config = subsys_config;

	boot_config->uc_tile_frequency_mhz = uc_freq;
	boot_config->syscom_context_config[0].max_output_queues =
		syscom->num_output_queues;
	boot_config->syscom_context_config[0].max_input_queues =
		syscom->num_input_queues;

	dma_sync_single_for_device(&adev->dev, adev->boot_config_dma_addr,
				   adev->boot_config_size, DMA_TO_DEVICE);

	for (i = 0; i < num_queues; i++) {
		u32 queue_size = qconfigs[i].max_capacity *
			qconfigs[i].token_size_in_bytes;

		total_queue_size += queue_size;
		queue_size = ALIGN(queue_size, IA_GOFO_CL_SIZE);
		total_queue_size_aligned += queue_size;
		qconfigs[i].queue_size = queue_size;
		dev_dbg(&adev->dev,
			"queue_configs[%d]: capacity = %d, token_size = %d\n",
			i, qconfigs[i].max_capacity,
			qconfigs[i].token_size_in_bytes);
	}
	dev_dbg(&adev->dev, "isys syscom queue size %d, aligned size %d\n",
		total_queue_size, total_queue_size_aligned);

	/* Allocate queue memory */
	syscom->queue_mem = dma_alloc_attrs(&adev->dev,
					    total_queue_size_aligned,
					    &syscom->queue_mem_dma_addr,
					    GFP_KERNEL, 0);
	if (!syscom->queue_mem) {
		dev_err(&adev->dev, "Failed to allocate queue memory.\n");
		return -ENOMEM;
	}
	dev_dbg(&adev->dev,
		"syscom queue memory. dma_addr: 0x%llx size: %d\n",
		syscom->queue_mem_dma_addr, total_queue_size_aligned);
	syscom->queue_mem_size = total_queue_size_aligned;

	syscfg = &boot_config->syscom_context_config[0];
	cfgs = ipu_syscom_get_queue_config(syscfg);
	queue_mem_ptr = syscom->queue_mem;
	queue_mem_dma_ptr = syscom->queue_mem_dma_addr;
	for (i = 0; i < num_queues; i++) {
		cfgs[i].token_array_mem = queue_mem_dma_ptr;
		cfgs[i].max_capacity = qconfigs[i].max_capacity;
		cfgs[i].token_size_in_bytes = qconfigs[i].token_size_in_bytes;
		qconfigs[i].token_array_mem = queue_mem_ptr;
		queue_mem_dma_ptr += qconfigs[i].queue_size;
		queue_mem_ptr += qconfigs[i].queue_size;
	}

	dma_sync_single_for_device(&adev->dev, syscom->queue_mem_dma_addr,
				   total_queue_size_aligned, DMA_TO_DEVICE);

	return 0;
}
EXPORT_SYMBOL_GPL(ipu_boot_init_boot_config);

void ipu_boot_release_boot_config(struct ipu_bus_device *adev)
{
	struct ipu_syscom_context *syscom = adev->syscom;

	if (syscom->queue_mem) {
		dma_free_attrs(&adev->dev,
			       syscom->queue_mem_size,
			       syscom->queue_mem,
			       syscom->queue_mem_dma_addr, 0);
		syscom->queue_mem = NULL;
		syscom->queue_mem_dma_addr = 0;
	}

	if (adev->boot_config) {
		dma_free_attrs(&adev->dev,
			       adev->boot_config_size,
			       adev->boot_config,
			       adev->boot_config_dma_addr, 0);
		adev->boot_config = NULL;
		adev->boot_config_dma_addr = 0;
	}
}
EXPORT_SYMBOL_GPL(ipu_boot_release_boot_config);

int ipu_boot_start_fw(const struct ipu_bus_device *adev)
{
	u32 boot_state, last_boot_state;
	u32 timeout = IPU_FW_START_STOP_TIMEOUT;
	void __iomem *base = adev->isp->base;
	u32 indices_addr, msg_ver;
	struct ia_gofo_version_s ver;
	int ret;

	ret = ipu_boot_cell_init(adev);
	if (ret)
		return ret;

	dev_dbg(&adev->dev, "start booting fw...\n");
	/* store "uninit" state to syscom/boot state reg */
	write_fw_boot_param(adev, IA_GOFO_FW_BOOT_STATE_ID,
			    IA_GOFO_FW_BOOT_STATE_UNINIT);
	/*
	 * Set registers to zero
	 * (not strictly required, but recommended for diagnostics)
	 */
	write_fw_boot_param(adev,
			    IA_GOFO_FW_BOOT_SYSCOM_QUEUE_INDICES_BASE_ID, 0);
	write_fw_boot_param(adev, IA_GOFO_FW_BOOT_MESSAGING_VERSION_ID, 0);
	/* store firmware configuration address */
	write_fw_boot_param(adev, IA_GOFO_FW_BOOT_CONFIG_ID,
			    adev->boot_config_dma_addr);

	/* Kick uC, then wait for boot complete */
	ipu_boot_cell_start(adev);

	last_boot_state = IA_GOFO_FW_BOOT_STATE_UNINIT;
	while (timeout--) {
		boot_state = read_fw_boot_param(adev,
						IA_GOFO_FW_BOOT_STATE_ID);
		if (boot_state != last_boot_state) {
			dev_dbg(&adev->dev,
				"boot state changed from 0x%x to 0x%x\n",
				last_boot_state, boot_state);
			last_boot_state = boot_state;
		}
		if (BOOT_STATE_IS_CRITICAL(boot_state) ||
		    BOOT_STATE_IS_READY(boot_state))
			break;
		usleep_range(1000, 1200);
	}

	if (BOOT_STATE_IS_CRITICAL(boot_state)) {
		ipu_dump_fw_error_log(adev);
		dev_err(&adev->dev, "critical boot state error 0x%x\n",
			boot_state);
		return -EINVAL;
	} else if (!BOOT_STATE_IS_READY(boot_state)) {
		dev_err(&adev->dev, "fw boot timeout. state: 0x%x\n",
			boot_state);
		return -ETIMEDOUT;
	}
	dev_dbg(&adev->dev, "fw boot done.\n");

	/* Get FW syscom queue indices addr */
	indices_addr = read_fw_boot_param(adev,
				IA_GOFO_FW_BOOT_SYSCOM_QUEUE_INDICES_BASE_ID);
	adev->syscom->queue_indices = base + indices_addr;
	dev_dbg(&adev->dev, "fw queue indices offset is 0x%x\n", indices_addr);

	/* Get message version. */
	msg_ver = read_fw_boot_param(adev,
				     IA_GOFO_FW_BOOT_MESSAGING_VERSION_ID);
	memcpy(&ver, &msg_ver, sizeof(msg_ver));
	dev_dbg(&adev->dev, "ipu message version is %d.%d.%d.%d\n",
		ver.major, ver.minor, ver.subminor, ver.patch);

	return 0;
}
EXPORT_SYMBOL_GPL(ipu_boot_start_fw);

int ipu_boot_stop_fw(const struct ipu_bus_device *adev)
{
	u32 boot_state;
	u32 timeout = IPU_FW_START_STOP_TIMEOUT;

	boot_state = read_fw_boot_param(adev, IA_GOFO_FW_BOOT_STATE_ID);
	if (BOOT_STATE_IS_CRITICAL(boot_state) ||
	    !BOOT_STATE_IS_READY(boot_state)) {
		dev_err(&adev->dev, "fw not ready for shutdown, state 0x%x\n",
			boot_state);
		return -EBUSY;
	}

	/* Issue shutdown to start shutdown process */
	dev_dbg(&adev->dev, "stopping fw...\n");
	write_fw_boot_param(adev, IA_GOFO_FW_BOOT_STATE_ID,
			    IA_GOFO_FW_BOOT_STATE_SHUTDOWN_CMD);
	while (timeout--) {
		boot_state = read_fw_boot_param(adev,
						IA_GOFO_FW_BOOT_STATE_ID);
		if (BOOT_STATE_IS_CRITICAL(boot_state) ||
		    BOOT_STATE_IS_INACTIVE(boot_state))
			break;
		usleep_range(1000, 1200);
	}

	if (BOOT_STATE_IS_CRITICAL(boot_state)) {
		ipu_dump_fw_error_log(adev);
		dev_err(&adev->dev, "critical boot state error 0x%x\n",
			boot_state);
		return -EINVAL;
	} else if (!BOOT_STATE_IS_INACTIVE(boot_state)) {
		dev_err(&adev->dev, "stop fw timeout. state: 0x%x\n",
			boot_state);
		return -ETIMEDOUT;
	}

	ipu_boot_cell_stop(adev);
	dev_dbg(&adev->dev, "stop fw done.\n");

	return 0;
}
EXPORT_SYMBOL_GPL(ipu_boot_stop_fw);

u32 ipu_boot_get_boot_state(const struct ipu_bus_device *adev)
{
	return read_fw_boot_param(adev, IA_GOFO_FW_BOOT_STATE_ID);
}
EXPORT_SYMBOL_GPL(ipu_boot_get_boot_state);
