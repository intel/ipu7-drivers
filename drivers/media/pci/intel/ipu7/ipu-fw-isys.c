// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2013 - 2024 Intel Corporation

#include <asm/cacheflush.h>

#include <linux/kernel.h>
#include <linux/delay.h>

#include "ipu.h"
#include "ipu-platform-regs.h"
#include "ipu-platform.h"
#include "ipu-fw-isys.h"
#include "ipu-syscom.h"
#include "ipu-isys.h"
#include "ipu_insys_q_defs.h"
#include "ipu_insys_config_abi.h"
#include "ipu_insys_abi.h"
#include "ipu-boot.h"

#define IPU_FW_UNSUPPORTED_DATA_TYPE	0
static const u32
extracted_bits_per_pixel_per_mipi_data_type[N_IPU_INSYS_MIPI_DATA_TYPE] = {
	64,	/* [0x00]   IPU_INSYS_MIPI_DATA_TYPE_FRAME_START_CODE */
	64,	/* [0x01]   IPU_INSYS_MIPI_DATA_TYPE_FRAME_END_CODE */
	64,	/* [0x02]   IPU_INSYS_MIPI_DATA_TYPE_LINE_START_CODE */
	64,	/* [0x03]   IPU_INSYS_MIPI_DATA_TYPE_LINE_END_CODE */
	IPU_FW_UNSUPPORTED_DATA_TYPE,	/* [0x04] */
	IPU_FW_UNSUPPORTED_DATA_TYPE,	/* [0x05] */
	IPU_FW_UNSUPPORTED_DATA_TYPE,	/* [0x06] */
	IPU_FW_UNSUPPORTED_DATA_TYPE,	/* [0x07] */
	64,	/* [0x08]   IPU_INSYS_MIPI_DATA_TYPE_GENERIC_SHORT1 */
	64,	/* [0x09]   IPU_INSYS_MIPI_DATA_TYPE_GENERIC_SHORT2 */
	64,	/* [0x0A]   IPU_INSYS_MIPI_DATA_TYPE_GENERIC_SHORT3 */
	64,	/* [0x0B]   IPU_INSYS_MIPI_DATA_TYPE_GENERIC_SHORT4 */
	64,	/* [0x0C]   IPU_INSYS_MIPI_DATA_TYPE_GENERIC_SHORT5 */
	64,	/* [0x0D]   IPU_INSYS_MIPI_DATA_TYPE_GENERIC_SHORT6 */
	64,	/* [0x0E]   IPU_INSYS_MIPI_DATA_TYPE_GENERIC_SHORT7 */
	64,	/* [0x0F]   IPU_INSYS_MIPI_DATA_TYPE_GENERIC_SHORT8 */
	IPU_FW_UNSUPPORTED_DATA_TYPE,	/* [0x10] */
	IPU_FW_UNSUPPORTED_DATA_TYPE,	/* [0x11] */
	8,	/* [0x12]    IPU_INSYS_MIPI_DATA_TYPE_EMBEDDED */
	IPU_FW_UNSUPPORTED_DATA_TYPE,	/* [0x13] */
	IPU_FW_UNSUPPORTED_DATA_TYPE,	/* [0x14] */
	IPU_FW_UNSUPPORTED_DATA_TYPE,	/* [0x15] */
	IPU_FW_UNSUPPORTED_DATA_TYPE,	/* [0x16] */
	IPU_FW_UNSUPPORTED_DATA_TYPE,	/* [0x17] */
	12,	/* [0x18]   IPU_INSYS_MIPI_DATA_TYPE_YUV420_8 */
	15,	/* [0x19]   IPU_INSYS_MIPI_DATA_TYPE_YUV420_10 */
	12,	/* [0x1A]   IPU_INSYS_MIPI_DATA_TYPE_YUV420_8_LEGACY */
	IPU_FW_UNSUPPORTED_DATA_TYPE,	/* [0x1B] */
	12,	/* [0x1C]   IPU_INSYS_MIPI_DATA_TYPE_YUV420_8_SHIFT */
	15,	/* [0x1D]   IPU_INSYS_MIPI_DATA_TYPE_YUV420_10_SHIFT */
	16,	/* [0x1E]   IPU_INSYS_MIPI_DATA_TYPE_YUV422_8 */
	20,	/* [0x1F]   IPU_INSYS_MIPI_DATA_TYPE_YUV422_10 */
	16,	/* [0x20]   IPU_INSYS_MIPI_DATA_TYPE_RGB_444 */
	16,	/* [0x21]   IPU_INSYS_MIPI_DATA_TYPE_RGB_555 */
	16,	/* [0x22]   IPU_INSYS_MIPI_DATA_TYPE_RGB_565 */
	18,	/* [0x23]   IPU_INSYS_MIPI_DATA_TYPE_RGB_666 */
	24,	/* [0x24]   IPU_INSYS_MIPI_DATA_TYPE_RGB_888 */
	IPU_FW_UNSUPPORTED_DATA_TYPE,	/* [0x25] */
	IPU_FW_UNSUPPORTED_DATA_TYPE,	/* [0x26] */
	IPU_FW_UNSUPPORTED_DATA_TYPE,	/* [0x27] */
	6,	/* [0x28]    IPU_INSYS_MIPI_DATA_TYPE_RAW_6 */
	7,	/* [0x29]    IPU_INSYS_MIPI_DATA_TYPE_RAW_7 */
	8,	/* [0x2A]    IPU_INSYS_MIPI_DATA_TYPE_RAW_8 */
	10,	/* [0x2B]    IPU_INSYS_MIPI_DATA_TYPE_RAW_10 */
	12,	/* [0x2C]    IPU_INSYS_MIPI_DATA_TYPE_RAW_12 */
	14,	/* [0x2D]    IPU_INSYS_MIPI_DATA_TYPE_RAW_14 */
	16,	/* [0x2E]    IPU_INSYS_MIPI_DATA_TYPE_RAW_16 */
	8,	/* [0x2F]    IPU_INSYS_MIPI_DATA_TYPE_BINARY_8 */
	8,	/* [0x30]    IPU_INSYS_MIPI_DATA_TYPE_USER_DEF1 */
	8,	/* [0x31]    IPU_INSYS_MIPI_DATA_TYPE_USER_DEF2 */
	8,	/* [0x32]    IPU_INSYS_MIPI_DATA_TYPE_USER_DEF3 */
	8,	/* [0x33]    IPU_INSYS_MIPI_DATA_TYPE_USER_DEF4 */
	8,	/* [0x34]    IPU_INSYS_MIPI_DATA_TYPE_USER_DEF5 */
	8,	/* [0x35]    IPU_INSYS_MIPI_DATA_TYPE_USER_DEF6 */
	8,	/* [0x36]    IPU_INSYS_MIPI_DATA_TYPE_USER_DEF7 */
	8,	/* [0x37]    IPU_INSYS_MIPI_DATA_TYPE_USER_DEF8 */
	IPU_FW_UNSUPPORTED_DATA_TYPE,	/* [0x38] */
	IPU_FW_UNSUPPORTED_DATA_TYPE,	/* [0x39] */
	IPU_FW_UNSUPPORTED_DATA_TYPE,	/* [0x3A] */
	IPU_FW_UNSUPPORTED_DATA_TYPE,	/* [0x3B] */
	IPU_FW_UNSUPPORTED_DATA_TYPE,	/* [0x3C] */
	IPU_FW_UNSUPPORTED_DATA_TYPE,	/* [0x3D] */
	IPU_FW_UNSUPPORTED_DATA_TYPE,	/* [0x3E] */
	IPU_FW_UNSUPPORTED_DATA_TYPE	/* [0x3F] */
};

static const char send_msg_types[N_IPU_INSYS_SEND_TYPE][32] = {
	"STREAM_OPEN",
	"STREAM_START_AND_CAPTURE",
	"STREAM_CAPTURE",
	"STREAM_ABORT",
	"STREAM_FLUSH",
	"STREAM_CLOSE"
};

int
ipu_fw_isys_complex_cmd(struct ipu_isys *isys,
			const unsigned int stream_handle,
			void *cpu_mapped_buf,
			dma_addr_t dma_mapped_buf,
			size_t size, enum ipu_insys_send_type send_type)
{
	struct ipu_syscom_context *ctx = isys->adev->syscom;
	struct ipu_insys_send_queue_token *token;

	if (send_type >= N_IPU_INSYS_SEND_TYPE)
		return -EINVAL;

	dev_dbg(&isys->adev->dev, "send_token: %s handle %u size %zu type %d\n",
		send_msg_types[send_type], stream_handle, size, send_type);

	/*
	 * Time to flush cache in case we have some payload. Not all messages
	 * have that
	 */
	if (cpu_mapped_buf)
		clflush_cache_range(cpu_mapped_buf, size);

	token = ipu_syscom_get_token(ctx,
				     stream_handle + IPU_INSYS_INPUT_MSG_QUEUE);
	if (!token)
		return -EBUSY;

	token->addr = dma_mapped_buf;
	token->buf_handle = (unsigned long)cpu_mapped_buf;
	token->send_type = send_type;
	token->stream_id = stream_handle;
	token->flag = IPU_INSYS_SEND_QUEUE_TOKEN_FLAG_NONE;

	ipu_syscom_put_token(ctx, stream_handle + IPU_INSYS_INPUT_MSG_QUEUE);
	/* now wakeup FW */
	ipu_buttress_wakeup_is_uc(isys->adev->isp);

	return 0;
}

int ipu_fw_isys_simple_cmd(struct ipu_isys *isys,
			   const unsigned int stream_handle,
			   enum ipu_insys_send_type send_type)
{
	return ipu_fw_isys_complex_cmd(isys, stream_handle, NULL, 0, 0,
				       send_type);
}

int ipu_fw_isys_init(struct ipu_isys *isys)
{
	struct ipu_bus_device *adev = isys->adev;
	struct ipu_syscom_context *syscom;
	struct ipu_insys_config *isys_config;
	struct syscom_queue_config *queue_configs;
	dma_addr_t isys_config_dma_addr;
	unsigned int i, num_queues;
	u32 freq;
	int ret;

	/* Allocate and init syscom context. */
	syscom = devm_kzalloc(&adev->dev, sizeof(struct ipu_syscom_context),
			      GFP_KERNEL);
	if (!syscom)
		return -ENOMEM;

	adev->syscom = syscom;
	syscom->num_input_queues = IPU_INSYS_MAX_INPUT_QUEUES;
	syscom->num_output_queues = IPU_INSYS_MAX_OUTPUT_QUEUES;
	num_queues = syscom->num_input_queues + syscom->num_output_queues;
	queue_configs = devm_kzalloc(&adev->dev,
				     FW_QUEUE_CONFIG_SIZE(num_queues),
				     GFP_KERNEL);
	if (!queue_configs) {
		dev_err(&adev->dev, "Failed to allocate queue configs.\n");
		ipu_fw_isys_release(isys);
		return -ENOMEM;
	}
	syscom->queue_configs = queue_configs;
	queue_configs[IPU_INSYS_OUTPUT_MSG_QUEUE].max_capacity =
		IPU_ISYS_SIZE_RECV_QUEUE;
	queue_configs[IPU_INSYS_OUTPUT_MSG_QUEUE].token_size_in_bytes =
		sizeof(struct ipu_insys_resp);
	queue_configs[IPU_INSYS_OUTPUT_LOG_QUEUE].max_capacity =
		IPU_ISYS_SIZE_LOG_QUEUE;
	queue_configs[IPU_INSYS_OUTPUT_LOG_QUEUE].token_size_in_bytes =
		sizeof(struct ipu_insys_resp);
	queue_configs[IPU_INSYS_OUTPUT_RESERVED_QUEUE].max_capacity = 0;
	queue_configs[IPU_INSYS_OUTPUT_RESERVED_QUEUE].token_size_in_bytes = 0;

	queue_configs[IPU_INSYS_INPUT_DEV_QUEUE].max_capacity =
		IPU_ISYS_MAX_STREAMS;
	queue_configs[IPU_INSYS_INPUT_DEV_QUEUE].token_size_in_bytes =
		sizeof(struct ipu_insys_send_queue_token);

	for (i = IPU_INSYS_INPUT_MSG_QUEUE; i < num_queues; i++) {
		queue_configs[i].max_capacity = IPU_ISYS_SIZE_SEND_QUEUE;
		queue_configs[i].token_size_in_bytes =
			sizeof(struct ipu_insys_send_queue_token);
	}

	/* Allocate ISYS subsys config. */
	isys_config = dma_alloc_attrs(&adev->dev,
				      sizeof(struct ipu_insys_config),
				      &isys_config_dma_addr, GFP_KERNEL, 0);
	if (!isys_config) {
		dev_err(&adev->dev, "Failed to allocate isys subsys config.\n");
		ipu_fw_isys_release(isys);
		return -ENOMEM;
	}
	isys->subsys_config = isys_config;
	isys->subsys_config_dma_addr = isys_config_dma_addr;
	memset(isys_config, 0, sizeof(struct ipu_insys_config));
	isys_config->logger_config.use_source_severity = 0;
	isys_config->logger_config.use_channels_enable_bitmask = 1;
	isys_config->logger_config.channels_enable_bitmask =
		LOGGER_CONFIG_CHANNEL_ENABLE_SYSCOM_BITMASK;
	isys_config->wdt_config.wdt_timer1_us = 0;
	isys_config->wdt_config.wdt_timer2_us = 0;
	ret = ipu_buttress_get_isys_freq(adev->isp, &freq);
	if (ret) {
		dev_err(&adev->dev, "Failed to get ISYS frequency.\n");
		ipu_fw_isys_release(isys);
		return ret;
	}

	dma_sync_single_for_device(&adev->dev, isys_config_dma_addr,
				   sizeof(struct ipu_insys_config),
				   DMA_TO_DEVICE);

	ret = ipu_boot_init_boot_config(adev, queue_configs, num_queues,
					freq, isys_config_dma_addr);
	if (ret)
		ipu_fw_isys_release(isys);
	return ret;
}

void ipu_fw_isys_release(struct ipu_isys *isys)
{
	struct ipu_bus_device *adev = isys->adev;

	ipu_boot_release_boot_config(adev);
	if (isys->subsys_config) {
		dma_free_attrs(&adev->dev,
			       sizeof(struct ipu_insys_config),
			       isys->subsys_config,
			       isys->subsys_config_dma_addr, 0);
		isys->subsys_config = NULL;
		isys->subsys_config_dma_addr = 0;
	}
}

int ipu_fw_isys_open(struct ipu_isys *isys)
{
	return ipu_boot_start_fw(isys->adev);
}

int ipu_fw_isys_close(struct ipu_isys *isys)
{
	return ipu_boot_stop_fw(isys->adev);
}

struct ipu_insys_resp *ipu_fw_isys_get_resp(struct ipu_isys *isys,
					    struct ipu_insys_resp *response)
{
	struct ipu_insys_resp *resp;

	resp = ipu_syscom_get_token(isys->adev->syscom,
				    IPU_INSYS_OUTPUT_MSG_QUEUE);

	return resp;
}

/* TODO: address the memory crash caused by offline logger */
int ipu_fw_isys_get_log(struct ipu_isys *isys)
{
	void *token;
	struct ia_gofo_msg_log *log_msg;
	u8 msg_type, msg_len;
	u32 count, fmt_id;
	struct device *dev = &isys->adev->dev;
	struct isys_fw_log *fw_log = isys->fw_log;

	token = ipu_syscom_get_token(isys->adev->syscom,
				     IPU_INSYS_OUTPUT_LOG_QUEUE);
	if (!token)
		return -ENODATA;

	while (token) {
		log_msg = (struct ia_gofo_msg_log *)token;

		msg_type = log_msg->header.tlv_header.tlv_type;
		msg_len = log_msg->header.tlv_header.tlv_len32;
		if (msg_type != IPU_MSG_TYPE_DEV_LOG || !msg_len)
			dev_warn(dev, "Invalid msg data from Log queue!\n");

		count = log_msg->log_info_ts.log_info.log_counter;
		fmt_id = log_msg->log_info_ts.log_info.fmt_id;
		if (count > fw_log->count + 1)
			dev_warn(dev, "log msg lost, count %u+1 != %u!\n",
				 count, fw_log->count);

		if (fmt_id == IA_GOFO_MSG_LOG_FMT_ID_INVALID) {
			dev_err(dev, "invalid log msg fmt_id 0x%x!\n", fmt_id);
			ipu_syscom_put_token(isys->adev->syscom,
					     IPU_INSYS_OUTPUT_LOG_QUEUE);
			return -EIO;
		}

		memcpy(fw_log->head, (void *)&log_msg->log_info_ts,
		       sizeof(struct ia_gofo_msg_log_info_ts));

		fw_log->count = count;
		fw_log->head += sizeof(struct ia_gofo_msg_log_info_ts);
		fw_log->size += sizeof(struct ia_gofo_msg_log_info_ts);

		ipu_syscom_put_token(isys->adev->syscom,
				     IPU_INSYS_OUTPUT_LOG_QUEUE);

		token = ipu_syscom_get_token(isys->adev->syscom,
					     IPU_INSYS_OUTPUT_LOG_QUEUE);
	};

	return 0;
}

void ipu_fw_isys_put_resp(struct ipu_syscom_context *syscom,
			  unsigned int queue)
{
	ipu_syscom_put_token(syscom, queue);
}

void ipu_fw_isys_dump_stream_cfg(struct device *dev,
				 struct ipu_insys_stream_cfg *cfg)
{
	unsigned int i;

	dev_dbg(dev, "---------------------------\n");
	dev_dbg(dev, "IPU_FW_ISYS_STREAM_CFG_DATA\n");

	dev_dbg(dev, ".port id %d\n", cfg->port_id);
	dev_dbg(dev, ".vc %d\n", cfg->vc);
	dev_dbg(dev, ".nof_input_pins = %d\n", cfg->nof_input_pins);
	dev_dbg(dev, ".nof_output_pins = %d\n", cfg->nof_output_pins);
	dev_dbg(dev, ".stream_msg_map = 0x%x\n", cfg->stream_msg_map);

	for (i = 0; i < cfg->nof_input_pins; i++) {
		dev_dbg(dev, ".input_pin[%d]:\n", i);
		dev_dbg(dev, "\t.dt = 0x%0x\n",
			cfg->input_pins[i].dt);
		dev_dbg(dev, "\t.disable_mipi_unpacking = %d\n",
			cfg->input_pins[i].disable_mipi_unpacking);
		dev_dbg(dev, "\t.dt_rename_mode = %d\n",
			cfg->input_pins[i].dt_rename_mode);
		dev_dbg(dev, "\t.mapped_dt = 0x%0x\n",
			cfg->input_pins[i].mapped_dt);
		dev_dbg(dev, "\t.input_res = %d x %d\n",
			cfg->input_pins[i].input_res.width,
			cfg->input_pins[i].input_res.height);
		dev_dbg(dev, "\t.sync_msg_map = 0x%x\n",
			cfg->input_pins[i].sync_msg_map);
	}

	for (i = 0; i < cfg->nof_output_pins; i++) {
		dev_dbg(dev, ".output_pin[%d]:\n", i);
		dev_dbg(dev, "\t.input_pin_id = %d\n",
			cfg->output_pins[i].input_pin_id);
		dev_dbg(dev, "\t.stride = %d\n", cfg->output_pins[i].stride);
		dev_dbg(dev, "\t.send_irq = %d\n",
			cfg->output_pins[i].send_irq);
		dev_dbg(dev, "\t.ft = %d\n", cfg->output_pins[i].ft);

		dev_dbg(dev, "\t.link.buffer_lines = %d\n",
			cfg->output_pins[i].link.buffer_lines);
		dev_dbg(dev, "\t.link.foreign_key = %d\n",
			cfg->output_pins[i].link.foreign_key);
		dev_dbg(dev, "\t.link.granularity_pointer_update = %d\n",
			cfg->output_pins[i].link.granularity_pointer_update);
		dev_dbg(dev, "\t.link.msg_link_streaming_mode = %d\n",
			cfg->output_pins[i].link.msg_link_streaming_mode);
		dev_dbg(dev, "\t.link.pbk_id = %d\n",
			cfg->output_pins[i].link.pbk_id);
		dev_dbg(dev, "\t.link.pbk_slot_id = %d\n",
			cfg->output_pins[i].link.pbk_slot_id);
		dev_dbg(dev, "\t.link.dest = %d\n",
			cfg->output_pins[i].link.dest);
		dev_dbg(dev, "\t.link.use_sw_managed = %d\n",
			cfg->output_pins[i].link.use_sw_managed);

		dev_dbg(dev, "\t.crop.line_top = %d\n",
			cfg->output_pins[i].crop.line_top);
		dev_dbg(dev, "\t.crop.line_bottom = %d\n",
			cfg->output_pins[i].crop.line_bottom);

		dev_dbg(dev, "\t.dpcm_enable = %d\n",
			cfg->output_pins[i].dpcm.enable);
		dev_dbg(dev, "\t.dpcm.type = %d\n",
			cfg->output_pins[i].dpcm.type);
		dev_dbg(dev, "\t.dpcm.predictor = %d\n",
			cfg->output_pins[i].dpcm.predictor);
	}
	dev_dbg(dev, "---------------------------\n");
}

void ipu_fw_isys_dump_frame_buff_set(struct device *dev,
				     struct ipu_insys_buffset *buf,
				     unsigned int outputs)
{
	unsigned int i;

	dev_dbg(dev, "--------------------------\n");
	dev_dbg(dev, "IPU_ISYS_BUFF_SET\n");
	dev_dbg(dev, ".capture_msg_map = %d\n", buf->capture_msg_map);
	dev_dbg(dev, ".frame_id = %d\n", buf->frame_id);
	dev_dbg(dev, ".skip_frame = %d\n", buf->skip_frame);

	for (i = 0; i < outputs; i++) {
		dev_dbg(dev, ".output_pin[%d]:\n", i);
		dev_dbg(dev, "\t.user_token = %llx\n",
			buf->output_pins[i].user_token);
		dev_dbg(dev, "\t.addr = 0x%x\n", buf->output_pins[i].addr);
	}
	dev_dbg(dev, "---------------------------\n");
}
