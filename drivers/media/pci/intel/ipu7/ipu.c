// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2013 - 2024 Intel Corporation

#include <linux/acpi.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/firmware.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pci.h>
#include <linux/pm_qos.h>
#include <linux/pm_runtime.h>
#include <linux/timer.h>
#include <linux/sched.h>

#include "ipu.h"
#include "ipu-buttress.h"
#include "ipu-platform.h"
#include "ipu-buttress-regs.h"
#include "ipu-cpd.h"
#include "ipu-bus.h"
#include "ipu-mmu.h"
#include "ipu-platform-regs.h"
#include "ipu-isys-csi2-regs.h"
#include "ia_gofo_msg_log.h"

#if defined(CONFIG_INTEL_IPU7_ACPI)
#include <media/ipu-acpi.h>
#endif

#define IPU_DRV_SAI_RW      0x0101U
#define IPU_SEC_SAI_RW      0x2020U

#define IPU_PCI_BAR		0
#define IPU_PCI_PBBAR		4

#if defined(CONFIG_INTEL_IPU7_ACPI)
static int isys_init_acpi_add_device(struct device *dev, void *priv,
				struct ipu_isys_csi2_config *csi2,
				bool reprobe)
{
	return 0;
}
#endif

struct ipu_cell_program_t {
	unsigned int magic_number;

	unsigned int blob_offset;
	unsigned int blob_size;

	unsigned int start[3];

	unsigned int icache_source;
	unsigned int icache_target;
	unsigned int icache_size;

	unsigned int pmem_source;
	unsigned int pmem_target;
	unsigned int pmem_size;

	unsigned int data_source;
	unsigned int data_target;
	unsigned int data_size;

	unsigned int bss_target;
	unsigned int bss_size;

	unsigned int cell_id;
	unsigned int regs_addr;

	unsigned int cell_pmem_data_bus_address;
	unsigned int cell_dmem_data_bus_address;
	unsigned int cell_pmem_control_bus_address;
	unsigned int cell_dmem_control_bus_address;

	unsigned int next;
	unsigned int dummy[2];
};

#ifdef CONFIG_VIDEO_INTEL_IPU_MGC
static unsigned int ipu_tpg_offsets[] = {
	MGC_MG_PORT(0),
	MGC_MG_PORT(1),
	MGC_MG_PORT(2),
	MGC_MG_PORT(3),
};
#endif

static unsigned int ipu_csi_offsets[] = {
	IPU_CSI_PORT_A_ADDR_OFFSET,
	IPU_CSI_PORT_B_ADDR_OFFSET,
	IPU_CSI_PORT_C_ADDR_OFFSET,
	IPU_CSI_PORT_D_ADDR_OFFSET,
};

struct ipu_isys_internal_pdata ipu7p5_isys_ipdata = {
	.hw_variant = {
		.offset = IPU_UNIFIED_OFFSET,
		.nr_mmus = IPU7P5_IS_MMU_NUM,
		.mmu_hw = {
			{
				.name = "IS_FW_RD",
				.offset = IPU7P5_IS_MMU_FW_RD_OFFSET,
				.zlx_offset = IPU7P5_IS_ZLX_UC_RD_OFFSET,
				.uao_offset = IPU7P5_IS_UAO_UC_RD_OFFSET,
				.info_bits = 0x20005101,
				.refill = 0x00002726,
				.collapse_en_bitmap = 0x1,
				.at_sp_arb_cfg = 0x1,
				.l1_block = IPU7P5_IS_MMU_FW_RD_L1_BLOCKNR_REG,
				.l2_block = IPU7P5_IS_MMU_FW_RD_L2_BLOCKNR_REG,
				.nr_l1streams = IPU7P5_IS_MMU_FW_RD_STREAM_NUM,
				.nr_l2streams = IPU7P5_IS_MMU_FW_RD_STREAM_NUM,
				.l1_block_sz = {
					0x0, 0x8, 0xa,
				},
				.l2_block_sz = {
					0x0, 0x2, 0x4,
				},
				.zlx_nr = IPU7P5_IS_ZLX_UC_RD_NUM,
				.zlx_axi_pool = {
					0x00000f30,
				},
				.zlx_en = {
					0, 1, 0, 0
				},
				.zlx_conf = {
					0x0,
				},
				.uao_p_num = IPU7P5_IS_UAO_UC_RD_PLANENUM,
				.uao_p2tlb = {
					0x00000049,
					0x0000004c,
					0x0000004d,
					0x00000000,
				},
			},
			{
				.name = "IS_FW_WR",
				.offset = IPU7P5_IS_MMU_FW_WR_OFFSET,
				.zlx_offset = IPU7P5_IS_ZLX_UC_WR_OFFSET,
				.uao_offset = IPU7P5_IS_UAO_UC_WR_OFFSET,
				.info_bits = 0x20005001,
				.refill = 0x00002524,
				.collapse_en_bitmap = 0x1,
				.l1_block = IPU7P5_IS_MMU_FW_WR_L1_BLOCKNR_REG,
				.l2_block = IPU7P5_IS_MMU_FW_WR_L2_BLOCKNR_REG,
				.nr_l1streams = IPU7P5_IS_MMU_FW_WR_STREAM_NUM,
				.nr_l2streams = IPU7P5_IS_MMU_FW_WR_STREAM_NUM,
				.l1_block_sz = {
					0x0, 0x8, 0xa,
				},
				.l2_block_sz = {
					0x0, 0x2, 0x4,
				},
				.zlx_nr = IPU7P5_IS_ZLX_UC_WR_NUM,
				.zlx_axi_pool = {
					0x00000f20,
				},
				.zlx_en = {
					0, 1, 1, 0,
				},
				.zlx_conf = {
					0x0,
					0x00010101,
					0x00010101,
					0x0,
				},
				.uao_p_num = IPU7P5_IS_UAO_UC_WR_PLANENUM,
				.uao_p2tlb = {
					0x00000049,
					0x0000004a,
					0x0000004b,
					0x00000000,
				},
			},
			{
				.name = "IS_DATA_WR_ISOC",
				.offset = IPU7P5_IS_MMU_M0_OFFSET,
				.zlx_offset = IPU7P5_IS_ZLX_M0_OFFSET,
				.uao_offset = IPU7P5_IS_UAO_M0_WR_OFFSET,
				.info_bits = 0x20004e01,
				.refill = 0x00002120,
				.collapse_en_bitmap = 0x1,
				.l1_block = IPU7P5_IS_MMU_M0_L1_BLOCKNR_REG,
				.l2_block = IPU7P5_IS_MMU_M0_L2_BLOCKNR_REG,
				.nr_l1streams = IPU7P5_IS_MMU_M0_STREAM_NUM,
				.nr_l2streams = IPU7P5_IS_MMU_M0_STREAM_NUM,
				.l1_block_sz = {
					0x00000000,
					0x00000002,
					0x00000004,
					0x00000006,
					0x00000008,
					0x0000000a,
					0x0000000c,
					0x0000000e,
					0x00000010,
					0x00000012,
					0x00000014,
					0x00000016,
					0x00000018,
					0x0000001a,
					0x0000001c,
					0x0000001e,
				},
				.l2_block_sz = {
					0x00000000,
					0x00000002,
					0x00000004,
					0x00000006,
					0x00000008,
					0x0000000a,
					0x0000000c,
					0x0000000e,
					0x00000010,
					0x00000012,
					0x00000014,
					0x00000016,
					0x00000018,
					0x0000001a,
					0x0000001c,
					0x0000001e,
				},
				.zlx_nr = IPU7P5_IS_ZLX_M0_NUM,
				.zlx_axi_pool = {
					0x00000f10,
				},
				.zlx_en = {
					1, 1, 1, 1, 1, 1, 1, 1,
					1, 1, 1, 1, 1, 1, 1, 1,
				},
				.zlx_conf = {
					0x00010103,
					0x00010103,
					0x00010103,
					0x00010103,
					0x00010103,
					0x00010103,
					0x00010103,
					0x00010103,
					0x00010103,
					0x00010103,
					0x00010103,
					0x00010103,
					0x00010103,
					0x00010103,
					0x00010103,
					0x00010103,
				},
				.uao_p_num = IPU7P5_IS_UAO_M0_WR_PLANENUM,
				.uao_p2tlb = {
					0x00000041,
					0x00000042,
					0x00000043,
					0x00000044,
					0x00000041,
					0x00000042,
					0x00000043,
					0x00000044,
					0x00000041,
					0x00000042,
					0x00000043,
					0x00000044,
					0x00000041,
					0x00000042,
					0x00000043,
					0x00000044,
				},
			},
			{
				.name = "IS_DATA_WR_SNOOP",
				.offset = IPU7P5_IS_MMU_M1_OFFSET,
				.zlx_offset = IPU7P5_IS_ZLX_M1_OFFSET,
				.uao_offset = IPU7P5_IS_UAO_M1_WR_OFFSET,
				.info_bits = 0x20004f01,
				.refill = 0x00002322,
				.collapse_en_bitmap = 0x1,
				.l1_block = IPU7P5_IS_MMU_M1_L1_BLOCKNR_REG,
				.l2_block = IPU7P5_IS_MMU_M1_L2_BLOCKNR_REG,
				.nr_l1streams = IPU7P5_IS_MMU_M1_STREAM_NUM,
				.nr_l2streams = IPU7P5_IS_MMU_M1_STREAM_NUM,
				.l1_block_sz = {
					0x00000000,
					0x00000002,
					0x00000004,
					0x00000006,
					0x00000008,
					0x0000000a,
					0x0000000c,
					0x0000000e,
					0x00000010,
					0x00000012,
					0x00000014,
					0x00000016,
					0x00000018,
					0x0000001a,
					0x0000001c,
					0x0000001e,
				},
				.l2_block_sz = {
					0x00000000,
					0x00000002,
					0x00000004,
					0x00000006,
					0x00000008,
					0x0000000a,
					0x0000000c,
					0x0000000e,
					0x00000010,
					0x00000012,
					0x00000014,
					0x00000016,
					0x00000018,
					0x0000001a,
					0x0000001c,
					0x0000001e,
				},
				.zlx_nr = IPU7P5_IS_ZLX_M1_NUM,
				.zlx_axi_pool = {
					0x00000f20,
				},
				.zlx_en = {
					1, 1, 1, 1, 1, 1, 1, 1,
					1, 1, 1, 1, 1, 1, 1, 1,
				},
				.zlx_conf = {
					0x00010103,
					0x00010103,
					0x00010103,
					0x00010103,
					0x00010103,
					0x00010103,
					0x00010103,
					0x00010103,
					0x00010103,
					0x00010103,
					0x00010103,
					0x00010103,
					0x00010103,
					0x00010103,
					0x00010103,
					0x00010103,
				},
				.uao_p_num = IPU7P5_IS_UAO_M1_WR_PLANENUM,
				.uao_p2tlb = {
					0x00000045,
					0x00000046,
					0x00000047,
					0x00000048,
					0x00000045,
					0x00000046,
					0x00000047,
					0x00000048,
					0x00000045,
					0x00000046,
					0x00000047,
					0x00000048,
					0x00000045,
					0x00000046,
					0x00000047,
					0x00000048,
				},
			},
		},
		.cdc_fifos = 3,
		.cdc_fifo_threshold = {6, 8, 2},
		.dmem_offset = IPU_ISYS_DMEM_OFFSET,
		.spc_offset = IPU_ISYS_SPC_OFFSET,
	},
	.isys_dma_overshoot = IPU_ISYS_OVERALLOC_MIN,
};

struct ipu_psys_internal_pdata ipu7p5_psys_ipdata = {
	.hw_variant = {
		.offset = IPU_UNIFIED_OFFSET,
		.nr_mmus = IPU7P5_PS_MMU_NUM,
		.mmu_hw = {
			{
				.name = "PS_FW_RD",
				.offset = IPU7P5_PS_MMU_FW_RD_OFFSET,
				.zlx_offset = IPU7P5_PS_ZLX_FW_RD_OFFSET,
				.uao_offset = IPU7P5_PS_UAO_FW_RD_OFFSET,
				.info_bits = 0x20004001,
				.refill = 0x00002726,
				.collapse_en_bitmap = 0x1,
				.l1_block = IPU7P5_PS_MMU_FW_RD_L1_BLOCKNR_REG,
				.l2_block = IPU7P5_PS_MMU_FW_RD_L2_BLOCKNR_REG,
				.nr_l1streams = IPU7P5_PS_MMU_FW_RD_STREAM_NUM,
				.nr_l2streams = IPU7P5_PS_MMU_FW_RD_STREAM_NUM,
				.l1_block_sz = {
					0x00000000,
					0x00000008,
					0x0000000a,
					0x0000000c,
					0x0000000d,
					0x0000000f,
					0x00000011,
					0x00000012,
					0x00000013,
					0x00000014,
					0x00000016,
					0x00000018,
					0x00000019,
					0x0000001a,
					0x0000001a,
					0x0000001a,
				},
				.l2_block_sz = {
					0x00000000,
					0x00000002,
					0x00000004,
					0x00000006,
					0x00000008,
					0x0000000a,
					0x0000000c,
					0x0000000e,
					0x00000010,
					0x00000012,
					0x00000014,
					0x00000016,
					0x00000018,
					0x0000001a,
					0x0000001c,
					0x0000001e,
				},
				.zlx_nr = IPU7P5_PS_ZLX_FW_RD_NUM,
				.zlx_axi_pool = {
					0x00000f30,
				},
				.zlx_en = {
					0, 1, 0, 0, 1, 1, 0, 0,
					0, 1, 1, 0, 0, 0, 0, 0,
				},
				.zlx_conf = {
					0x00000000,
					0x00010101,
					0x00000000,
					0x00000000,
					0x00010101,
					0x00010101,
					0x00000000,
					0x00000000,
					0x00000000,
					0x00010101,
					0x00010101,
					0x00000000,
					0x00000000,
					0x00000000,
					0x00000000,
					0x00000000,
				},
				.uao_p_num = IPU7P5_PS_UAO_FW_RD_PLANENUM,
				.uao_p2tlb = {
					0x0000002e,
					0x00000035,
					0x00000036,
					0x00000031,
					0x00000037,
					0x00000038,
					0x00000039,
					0x00000032,
					0x00000033,
					0x0000003a,
					0x0000003b,
					0x0000003c,
					0x00000034,
					0x0,
					0x0,
					0x0,
				},
			},
			{
				.name = "PS_FW_WR",
				.offset = IPU7P5_PS_MMU_FW_WR_OFFSET,
				.zlx_offset = IPU7P5_PS_ZLX_FW_WR_OFFSET,
				.uao_offset = IPU7P5_PS_UAO_FW_WR_OFFSET,
				.info_bits = 0x20003e01,
				.refill = 0x00002322,
				.collapse_en_bitmap = 0x1,
				.l1_block = IPU7P5_PS_MMU_FW_WR_L1_BLOCKNR_REG,
				.l2_block = IPU7P5_PS_MMU_FW_WR_L2_BLOCKNR_REG,
				.nr_l1streams = IPU7P5_PS_MMU_FW_WR_STREAM_NUM,
				.nr_l2streams = IPU7P5_PS_MMU_FW_WR_STREAM_NUM,
				.l1_block_sz = {
					0x00000000,
					0x00000008,
					0x0000000a,
					0x0000000c,
					0x0000000d,
					0x0000000e,
					0x0000000f,
					0x00000010,
					0x00000010,
					0x00000010,
				},
				.l2_block_sz = {
					0x00000000,
					0x00000002,
					0x00000004,
					0x00000006,
					0x00000008,
					0x0000000a,
					0x0000000c,
					0x0000000e,
					0x00000010,
					0x00000012,
				},
				.zlx_nr = IPU7P5_PS_ZLX_FW_WR_NUM,
				.zlx_axi_pool = {
					0x00000f20,
				},
				.zlx_en = {
					0, 1, 1, 0, 0, 0, 0, 0, 0, 0,
				},
				.zlx_conf = {
					0x00000000,
					0x00010101,
					0x00010101,
					0x00000000,
					0x00000000,
					0x00000000,
					0x00000000,
					0x00000000,
					0x00000000,
					0x00000000,
				},
				.uao_p_num = IPU7P5_PS_UAO_FW_WR_PLANENUM,
				.uao_p2tlb = {
					0x0000002e,
					0x0000002f,
					0x00000030,
					0x00000031,
					0x00000032,
					0x00000033,
					0x00000034,
					0x0,
					0x0,
					0x0,
				},
			},
			{
				.name = "PS_DATA_RD",
				.offset = IPU7P5_PS_MMU_SRT_RD_OFFSET,
				.zlx_offset = IPU7P5_PS_ZLX_DATA_RD_OFFSET,
				.uao_offset = IPU7P5_PS_UAO_SRT_RD_OFFSET,
				.info_bits = 0x20003f01,
				.refill = 0x00002524,
				.collapse_en_bitmap = 0x1,
				.l1_block = IPU7P5_PS_MMU_SRT_RD_L1_BLOCKNR_REG,
				.l2_block = IPU7P5_PS_MMU_SRT_RD_L2_BLOCKNR_REG,
				.nr_l1streams = IPU7P5_PS_MMU_SRT_RD_STREAM_NUM,
				.nr_l2streams = IPU7P5_PS_MMU_SRT_RD_STREAM_NUM,
				.l1_block_sz = {
					0x00000000,
					0x00000004,
					0x00000006,
					0x00000008,
					0x0000000b,
					0x0000000d,
					0x0000000f,
					0x00000013,
					0x00000017,
					0x00000019,
					0x0000001b,
					0x0000001d,
					0x0000001f,
					0x0000002b,
					0x00000033,
					0x0000003f,
					0x00000047,
					0x00000049,
					0x0000004b,
					0x0000004c,
					0x0000004d,
					0x0000004e,
				},
				.l2_block_sz = {
					0x00000000,
					0x00000002,
					0x00000004,
					0x00000006,
					0x00000008,
					0x0000000a,
					0x0000000c,
					0x0000000e,
					0x00000010,
					0x00000012,
					0x00000014,
					0x00000016,
					0x00000018,
					0x0000001a,
					0x0000001c,
					0x0000001e,
					0x00000020,
					0x00000022,
					0x00000024,
					0x00000026,
					0x00000028,
					0x0000002a,
				},
				.zlx_nr = IPU7P5_PS_ZLX_DATA_RD_NUM,
				.zlx_axi_pool = {
					0x00000f30,
				},
				.zlx_en = {
					1, 1, 1, 1, 1, 1, 1, 1,
					1, 1, 1, 1, 1, 1, 1, 1,
					1, 1, 0, 0, 0, 0,
				},
				.zlx_conf = {
					0x00030303,
					0x00010101,
					0x00010101,
					0x00030202,
					0x00010101,
					0x00010101,
					0x00030303,
					0x00030303,
					0x00010101,
					0x00030800,
					0x00030500,
					0x00020101,
					0x00042000,
					0x00031000,
					0x00042000,
					0x00031000,
					0x00020400,
					0x00010101,
					0x00000000,
					0x00000000,
					0x00000000,
					0x00000000,
				},
				.uao_p_num = IPU7P5_PS_UAO_SRT_RD_PLANENUM,
				.uao_p2tlb = {
					0x0000001c,
					0x0000001d,
					0x0000001e,
					0x0000001f,
					0x00000020,
					0x00000021,
					0x00000022,
					0x00000023,
					0x00000024,
					0x00000025,
					0x00000026,
					0x00000027,
					0x00000028,
					0x00000029,
					0x0000002a,
					0x0000002b,
					0x0000002c,
					0x0000002d,
					0x00000000,
					0x00000000,
					0x00000000,
					0x00000000,
				},
			},
			{
				.name = "PS_DATA_WR",
				.offset = IPU7P5_PS_MMU_SRT_WR_OFFSET,
				.zlx_offset = IPU7P5_PS_ZLX_DATA_WR_OFFSET,
				.uao_offset = IPU7P5_PS_UAO_SRT_WR_OFFSET,
				.info_bits = 0x20003d01,
				.refill = 0x00002120,
				.collapse_en_bitmap = 0x1,
				.l1_block = IPU7P5_PS_MMU_SRT_WR_L1_BLOCKNR_REG,
				.l2_block = IPU7P5_PS_MMU_SRT_WR_L2_BLOCKNR_REG,
				.nr_l1streams = IPU7P5_PS_MMU_SRT_WR_STREAM_NUM,
				.nr_l2streams = IPU7P5_PS_MMU_SRT_WR_STREAM_NUM,
				.l1_block_sz = {
					0x00000000,
					0x00000002,
					0x00000006,
					0x0000000a,
					0x0000000c,
					0x0000000e,
					0x00000010,
					0x00000012,
					0x00000014,
					0x00000016,
					0x00000018,
					0x0000001a,
					0x0000001c,
					0x0000001e,
					0x00000020,
					0x00000022,
					0x00000024,
					0x00000028,
					0x0000002a,
					0x00000036,
					0x0000003e,
					0x00000040,
					0x00000042,
					0x0000004e,
					0x00000056,
					0x0000005c,
					0x00000068,
					0x00000070,
					0x00000076,
					0x00000077,
					0x00000078,
					0x00000079,
				},
				.l2_block_sz = {
					0x00000000,
					0x00000002,
					0x00000006,
					0x0000000a,
					0x0000000c,
					0x0000000e,
					0x00000010,
					0x00000012,
					0x00000014,
					0x00000016,
					0x00000018,
					0x0000001a,
					0x0000001c,
					0x0000001e,
					0x00000020,
					0x00000022,
					0x00000024,
					0x00000028,
					0x0000002a,
					0x00000036,
					0x0000003e,
					0x00000040,
					0x00000042,
					0x0000004e,
					0x00000056,
					0x0000005c,
					0x00000068,
					0x00000070,
					0x00000076,
					0x00000077,
					0x00000078,
					0x00000079,
				},
				.zlx_nr = IPU7P5_PS_ZLX_DATA_WR_NUM,
				.zlx_axi_pool = {
					0x00000f50,
				},
				.zlx_en = {
					1, 1, 1, 1, 1, 1, 1, 1,
					0, 0, 1, 1, 1, 1, 1, 1,
					1, 1, 1, 1, 1, 1, 1, 1,
					1, 1, 1, 1, 0, 0, 0, 0,
				},
				.zlx_conf = {
					0x00010102,
					0x00030103,
					0x00030103,
					0x00010101,
					0x00010101,
					0x00030101,
					0x00010101,
					0x38010101,
					0x00000000,
					0x00000000,
					0x38010101,
					0x38010101,
					0x38010101,
					0x38010101,
					0x38010101,
					0x38010101,
					0x00030303,
					0x00010101,
					0x00042000,
					0x00031000,
					0x00010101,
					0x00010101,
					0x00042000,
					0x00031000,
					0x00031000,
					0x00042000,
					0x00031000,
					0x00031000,
					0x00000000,
					0x00000000,
					0x00000000,
					0x00000000,
				},
				.uao_p_num = IPU7P5_PS_UAO_SRT_WR_PLANENUM,
				.uao_p2tlb = {
					0x00000000,
					0x00000001,
					0x00000002,
					0x00000003,
					0x00000004,
					0x00000005,
					0x00000006,
					0x00000007,
					0x00000008,
					0x00000009,
					0x0000000a,
					0x0000000b,
					0x0000000c,
					0x0000000d,
					0x0000000e,
					0x0000000f,
					0x00000010,
					0x00000011,
					0x00000012,
					0x00000013,
					0x00000014,
					0x00000015,
					0x00000016,
					0x00000017,
					0x00000018,
					0x00000019,
					0x0000001a,
					0x0000001b,
					0x00000000,
					0x00000000,
					0x00000000,
					0x00000000,
				},
			},
		},
		.dmem_offset = IPU_PSYS_DMEM_OFFSET,
	},
};

struct ipu_isys_internal_pdata ipu7_isys_ipdata = {
	.hw_variant = {
		.offset = IPU_UNIFIED_OFFSET,
		.nr_mmus = IPU7_IS_MMU_NUM,
		.mmu_hw = {
			{
				.name = "IS_FW_RD",
				.offset = IPU7_IS_MMU_FW_RD_OFFSET,
				.zlx_offset = IPU7_IS_ZLX_UC_RD_OFFSET,
				.uao_offset = IPU7_IS_UAO_UC_RD_OFFSET,
				.info_bits = 0x20006701,
				.refill = 0x00002726,
				.collapse_en_bitmap = 0x0,
				.l1_block = IPU7_IS_MMU_FW_RD_L1_BLOCKNR_REG,
				.l2_block = IPU7_IS_MMU_FW_RD_L2_BLOCKNR_REG,
				.nr_l1streams = IPU7_IS_MMU_FW_RD_STREAM_NUM,
				.nr_l2streams = IPU7_IS_MMU_FW_RD_STREAM_NUM,
				.l1_block_sz = {
					0x0, 0x8, 0xa,
				},
				.l2_block_sz = {
					0x0, 0x2, 0x4,
				},
				.zlx_nr = IPU7_IS_ZLX_UC_RD_NUM,
				.zlx_axi_pool = {
					0x00000f30,
				},
				.zlx_en = {
					0, 0, 0, 0
				},
				.zlx_conf = {
					0x0, 0x0, 0x0, 0x0,
				},
				.uao_p_num = IPU7_IS_UAO_UC_RD_PLANENUM,
				.uao_p2tlb = {
					0x00000061,
					0x00000064,
					0x00000065,
				},
			},
			{
				.name = "IS_FW_WR",
				.offset = IPU7_IS_MMU_FW_WR_OFFSET,
				.zlx_offset = IPU7_IS_ZLX_UC_WR_OFFSET,
				.uao_offset = IPU7_IS_UAO_UC_WR_OFFSET,
				.info_bits = 0x20006801,
				.refill = 0x00002524,
				.collapse_en_bitmap = 0x0,
				.l1_block = IPU7_IS_MMU_FW_WR_L1_BLOCKNR_REG,
				.l2_block = IPU7_IS_MMU_FW_WR_L2_BLOCKNR_REG,
				.nr_l1streams = IPU7_IS_MMU_FW_WR_STREAM_NUM,
				.nr_l2streams = IPU7_IS_MMU_FW_WR_STREAM_NUM,
				.l1_block_sz = {
					0x0, 0x8, 0xa,
				},
				.l2_block_sz = {
					0x0, 0x2, 0x4,
				},
				.zlx_nr = IPU7_IS_ZLX_UC_WR_NUM,
				.zlx_axi_pool = {
					0x00000f20,
				},
				.zlx_en = {
					0, 1, 1, 0,
				},
				.zlx_conf = {
					0x0,
					0x00010101,
					0x00010101,
				},
				.uao_p_num = IPU7_IS_UAO_UC_WR_PLANENUM,
				.uao_p2tlb = {
					0x00000061,
					0x00000062,
					0x00000063,
				},
			},
			{
				.name = "IS_DATA_WR_ISOC",
				.offset = IPU7_IS_MMU_M0_OFFSET,
				.zlx_offset = IPU7_IS_ZLX_M0_OFFSET,
				.uao_offset = IPU7_IS_UAO_M0_WR_OFFSET,
				.info_bits = 0x20006601,
				.refill = 0x00002120,
				.collapse_en_bitmap = 0x0,
				.l1_block = IPU7_IS_MMU_M0_L1_BLOCKNR_REG,
				.l2_block = IPU7_IS_MMU_M0_L2_BLOCKNR_REG,
				.nr_l1streams = IPU7_IS_MMU_M0_STREAM_NUM,
				.nr_l2streams = IPU7_IS_MMU_M0_STREAM_NUM,
				.l1_block_sz = {
					0x0, 0x3, 0x6, 0x8, 0xa, 0xc, 0xe, 0x10,
				},
				.l2_block_sz = {
					0x0, 0x2, 0x4, 0x6, 0x8, 0xa, 0xc, 0xe,
				},
				.zlx_nr = IPU7_IS_ZLX_M0_NUM,
				.zlx_axi_pool = {
					0x00000f10,
				},
				.zlx_en = {
					1, 1, 1, 1, 1, 1, 1, 1,
				},
				.zlx_conf = {
					0x00010103,
					0x00010103,
					0x00010101,
					0x00010101,
					0x00010101,
					0x00010101,
					0x00010101,
					0x00010101,
				},
				.uao_p_num = IPU7_IS_UAO_M0_WR_PLANENUM,
				.uao_p2tlb = {
					0x00000049,
					0x0000004a,
					0x0000004b,
					0x0000004c,
					0x0000004d,
					0x0000004e,
					0x0000004f,
					0x00000050,
				},
			},
			{
				.name = "IS_DATA_WR_SNOOP",
				.offset = IPU7_IS_MMU_M1_OFFSET,
				.zlx_offset = IPU7_IS_ZLX_M1_OFFSET,
				.uao_offset = IPU7_IS_UAO_M1_WR_OFFSET,
				.info_bits = 0x20006901,
				.refill = 0x00002322,
				.collapse_en_bitmap = 0x0,
				.l1_block = IPU7_IS_MMU_M1_L1_BLOCKNR_REG,
				.l2_block = IPU7_IS_MMU_M1_L2_BLOCKNR_REG,
				.nr_l1streams = IPU7_IS_MMU_M1_STREAM_NUM,
				.nr_l2streams = IPU7_IS_MMU_M1_STREAM_NUM,
				.l1_block_sz = {
					0x0, 0x3, 0x6, 0x9, 0xc,
					0xe, 0x10, 0x12, 0x14, 0x16,
					0x18, 0x1a, 0x1c, 0x1e, 0x20,
					0x22,
				},
				.l2_block_sz = {
					0x0, 0x2, 0x4, 0x6, 0x8,
					0xa, 0xc, 0xe, 0x10, 0x12,
					0x14, 0x16, 0x18, 0x1a, 0x1c,
					0x1e,
				},
				.zlx_nr = IPU7_IS_ZLX_M1_NUM,
				.zlx_axi_pool = {
					0x00000f20,
				},
				.zlx_en = {
					1, 1, 1, 1, 1, 1, 1, 1,
					1, 1, 1, 1, 1, 1, 1, 1,
				},
				.zlx_conf = {
					0x00010103,
					0x00010103,
					0x00010103,
					0x00010103,
					0x00010103,
					0x00010103,
					0x00010103,
					0x00010103,
					0x00010101,
					0x00010101,
					0x00010101,
					0x00010101,
					0x00010101,
					0x00010101,
					0x00010101,
					0x00010101,
				},
				.uao_p_num = IPU7_IS_UAO_M1_WR_PLANENUM,
				.uao_p2tlb = {
					0x00000051,
					0x00000052,
					0x00000053,
					0x00000054,
					0x00000055,
					0x00000056,
					0x00000057,
					0x00000058,
					0x00000059,
					0x0000005a,
					0x0000005b,
					0x0000005c,
					0x0000005d,
					0x0000005e,
					0x0000005f,
					0x00000060,
				},
			},
		},
		.cdc_fifos = 3,
		.cdc_fifo_threshold = {6, 8, 2},
		.dmem_offset = IPU_ISYS_DMEM_OFFSET,
		.spc_offset = IPU_ISYS_SPC_OFFSET,
	},
	.isys_dma_overshoot = IPU_ISYS_OVERALLOC_MIN,
};

struct ipu_psys_internal_pdata ipu7_psys_ipdata = {
	.hw_variant = {
		.offset = IPU_UNIFIED_OFFSET,
		.nr_mmus = IPU7_PS_MMU_NUM,
		.mmu_hw = {
			{
				.name = "PS_FW_RD",
				.offset = IPU7_PS_MMU_FW_RD_OFFSET,
				.zlx_offset = IPU7_PS_ZLX_FW_RD_OFFSET,
				.uao_offset = IPU7_PS_UAO_FW_RD_OFFSET,
				.info_bits = 0x20004801,
				.refill = 0x00002726,
				.collapse_en_bitmap = 0x0,
				.l1_block = IPU7_PS_MMU_FW_RD_L1_BLOCKNR_REG,
				.l2_block = IPU7_PS_MMU_FW_RD_L2_BLOCKNR_REG,
				.nr_l1streams = IPU7_PS_MMU_FW_RD_STREAM_NUM,
				.nr_l2streams = IPU7_PS_MMU_FW_RD_STREAM_NUM,
				.l1_block_sz = {
					0, 0x8, 0xa, 0xc, 0xd,
					0xf, 0x11, 0x12, 0x13, 0x14,
					0x16, 0x18, 0x19, 0x1a, 0x1a,
					0x1a, 0x1a, 0x1a, 0x1a, 0x1a,
				},
				.l2_block_sz = {
					0x0, 0x2, 0x4, 0x6, 0x8,
					0xa, 0xc, 0xe, 0x10, 0x12,
					0x14, 0x16, 0x18, 0x1a, 0x1c,
					0x1e, 0x20, 0x22, 0x24, 0x26,
				},
				.zlx_nr = IPU7_PS_ZLX_FW_RD_NUM,
				.zlx_axi_pool = {
					0x00000f30,
				},
				.zlx_en = {
					0, 0, 0, 0, 0, 0, 0, 0,
					0, 0, 0, 0, 0, 0, 0, 0,
				},
				.zlx_conf = {
					0x0,
				},
				.uao_p_num = IPU7_PS_UAO_FW_RD_PLANENUM,
				.uao_p2tlb = {
					0x00000036,
					0x0000003d,
					0x0000003e,
					0x00000039,
					0x0000003f,
					0x00000040,
					0x00000041,
					0x0000003a,
					0x0000003b,
					0x00000042,
					0x00000043,
					0x00000044,
					0x0000003c,
				},
			},
			{
				.name = "PS_FW_WR",
				.offset = IPU7_PS_MMU_FW_WR_OFFSET,
				.zlx_offset = IPU7_PS_ZLX_FW_WR_OFFSET,
				.uao_offset = IPU7_PS_UAO_FW_WR_OFFSET,
				.info_bits = 0x20004601,
				.refill = 0x00002322,
				.collapse_en_bitmap = 0x0,
				.l1_block = IPU7_PS_MMU_FW_WR_L1_BLOCKNR_REG,
				.l2_block = IPU7_PS_MMU_FW_WR_L2_BLOCKNR_REG,
				.nr_l1streams = IPU7_PS_MMU_FW_WR_STREAM_NUM,
				.nr_l2streams = IPU7_PS_MMU_FW_WR_STREAM_NUM,
				.l1_block_sz = {
					0, 0x8, 0xa, 0xc, 0xd,
					0xe, 0xf, 0x10, 0x10, 0x10,
				},
				.l2_block_sz = {
					0x0, 0x2, 0x4, 0x6, 0x8,
					0xa, 0xc, 0xe, 0x10, 0x12,
				},
				.zlx_nr = IPU7_PS_ZLX_FW_WR_NUM,
				.zlx_axi_pool = {
					0x00000f20,
				},
				.zlx_en = {
					0, 1, 1, 0, 0, 0, 0, 0,
					0, 0,
				},
				.zlx_conf = {
					0x0,
					0x00010101,
					0x00010101,
				},
				.uao_p_num = IPU7_PS_UAO_FW_WR_PLANENUM,
				.uao_p2tlb = {
					0x00000036,
					0x00000037,
					0x00000038,
					0x00000039,
					0x0000003a,
					0x0000003b,
					0x0000003c,
				},
			},
			{
				.name = "PS_DATA_RD",
				.offset = IPU7_PS_MMU_SRT_RD_OFFSET,
				.zlx_offset = IPU7_PS_ZLX_DATA_RD_OFFSET,
				.uao_offset = IPU7_PS_UAO_SRT_RD_OFFSET,
				.info_bits = 0x20004701,
				.refill = 0x00002120,
				.collapse_en_bitmap = 0x0,
				.l1_block = IPU7_PS_MMU_SRT_RD_L1_BLOCKNR_REG,
				.l2_block = IPU7_PS_MMU_SRT_RD_L2_BLOCKNR_REG,
				.nr_l1streams = IPU7_PS_MMU_SRT_RD_STREAM_NUM,
				.nr_l2streams = IPU7_PS_MMU_SRT_RD_STREAM_NUM,
				.l1_block_sz = {
					0x0, 0x4, 0x6, 0x8, 0xb,
					0xd, 0xf, 0x11, 0x13, 0x15,
					0x17, 0x23, 0x2b, 0x37, 0x3f,
					0x41, 0x43, 0x44, 0x45, 0x46,
					0x47, 0x48, 0x49, 0x4a, 0x4b,
					0x4c, 0x4d, 0x4e, 0x4f, 0x50,
					0x51, 0x52, 0x53, 0x55, 0x57,
					0x59, 0x5b, 0x5d, 0x5f, 0x61,
				},
				.l2_block_sz = {
					0x0, 0x2, 0x4, 0x6, 0x8,
					0xa, 0xc, 0xe, 0x10, 0x12,
					0x14, 0x16, 0x18, 0x1a, 0x1c,
					0x1e, 0x20, 0x22, 0x24, 0x26,
					0x28, 0x2a, 0x2c, 0x2e, 0x30,
					0x32, 0x34, 0x36, 0x38, 0x3a,
					0x3c, 0x3e, 0x40, 0x42, 0x44,
					0x46, 0x48, 0x4a, 0x4c, 0x4e,
				},
				.zlx_nr = IPU7_PS_ZLX_DATA_RD_NUM,
				.zlx_axi_pool = {
					0x00000f30,
				},
				.zlx_en = {
					1, 1, 1, 1, 1, 1, 1, 1,
					1, 1, 1, 1, 1, 1, 1, 1,
					0, 0, 0, 0, 0, 0, 0, 0,
					0, 0, 0, 0, 0, 0, 0, 0,
				},
				.zlx_conf = {
					0x00030303,
					0x00010101,
					0x00010101,
					0x00030202,
					0x00010101,
					0x00010101,
					0x00010101,
					0x00030800,
					0x00030500,
					0x00020101,
					0x00042000,
					0x00031000,
					0x00042000,
					0x00031000,
					0x00020400,
					0x00010101,
				},
				.uao_p_num = IPU7_PS_UAO_SRT_RD_PLANENUM,
				.uao_p2tlb = {
					0x00000022,
					0x00000023,
					0x00000024,
					0x00000025,
					0x00000026,
					0x00000027,
					0x00000028,
					0x00000029,
					0x0000002a,
					0x0000002b,
					0x0000002c,
					0x0000002d,
					0x0000002e,
					0x0000002f,
					0x00000030,
					0x00000031,
					0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
					0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
					0x0000001e,
					0x0000001f,
					0x00000020,
					0x00000021,
					0x00000032,
					0x00000033,
					0x00000034,
					0x00000035,
				},
			},
			{
				.name = "PS_DATA_WR",
				.offset = IPU7_PS_MMU_SRT_WR_OFFSET,
				.zlx_offset = IPU7_PS_ZLX_DATA_WR_OFFSET,
				.uao_offset = IPU7_PS_UAO_SRT_WR_OFFSET,
				.info_bits = 0x20004501,
				.refill = 0x00002120,
				.collapse_en_bitmap = 0x0,
				.l1_block = IPU7_PS_MMU_SRT_WR_L1_BLOCKNR_REG,
				.l2_block = IPU7_PS_MMU_SRT_WR_L2_BLOCKNR_REG,
				.nr_l1streams = IPU7_PS_MMU_SRT_WR_STREAM_NUM,
				.nr_l2streams = IPU7_PS_MMU_SRT_WR_STREAM_NUM,
				.l1_block_sz = {
					0x0, 0x2, 0x6, 0xa, 0xc,
					0xe, 0x10, 0x12, 0x14, 0x16,
					0x18, 0x1a, 0x1c, 0x1e, 0x20,
					0x22, 0x24, 0x26, 0x32, 0x3a,
					0x3c, 0x3e, 0x4a, 0x52, 0x58,
					0x64, 0x6c, 0x72, 0x7e, 0x86,
					0x8c, 0x8d, 0x8e, 0x8f, 0x90,
					0x91, 0x92, 0x94, 0x96, 0x98,
				},
				.l2_block_sz = {
					0x0, 0x2, 0x4, 0x6, 0x8,
					0xa, 0xc, 0xe, 0x10, 0x12,
					0x14, 0x16, 0x18, 0x1a, 0x1c,
					0x1e, 0x20, 0x22, 0x24, 0x26,
					0x28, 0x2a, 0x2c, 0x2e, 0x30,
					0x32, 0x34, 0x36, 0x38, 0x3a,
					0x3c, 0x3e, 0x40, 0x42, 0x44,
					0x46, 0x48, 0x4a, 0x4c, 0x4e,
				},
				.zlx_nr = IPU7_PS_ZLX_DATA_WR_NUM,
				.zlx_axi_pool = {
					0x00000f50,
				},
				.zlx_en = {
					1, 1, 1, 1, 1, 1, 1, 1,
					0, 0, 1, 1, 1, 1, 1, 1,
					1, 1, 1, 1, 1, 1, 1, 1,
					1, 1, 1, 1, 1, 1, 0, 0,
				},
				.zlx_conf = {
					0x00010102,
					0x00030103,
					0x00030103,
					0x00010101,
					0x00010101,
					0x00030101,
					0x00010101,
					0x38010101,
					0x0,
					0x0,
					0x38010101,
					0x38010101,
					0x38010101,
					0x38010101,
					0x38010101,
					0x38010101,
					0x00010101,
					0x00042000,
					0x00031000,
					0x00010101,
					0x00010101,
					0x00042000,
					0x00031000,
					0x00031000,
					0x00042000,
					0x00031000,
					0x00031000,
					0x00042000,
					0x00031000,
					0x00031000,
					0x0,
					0x0,
				},
				.uao_p_num = IPU7_PS_UAO_SRT_WR_PLANENUM,
				.uao_p2tlb = {
					0x00000000,
					0x00000001,
					0x00000002,
					0x00000003,
					0x00000004,
					0x00000005,
					0x00000006,
					0x00000007,
					0x00000008,
					0x00000009,
					0x0000000a,
					0x0000000b,
					0x0000000c,
					0x0000000d,
					0x0000000e,
					0x0000000f,
					0x00000010,
					0x00000011,
					0x00000012,
					0x00000013,
					0x00000014,
					0x00000015,
					0x00000016,
					0x00000017,
					0x00000018,
					0x00000019,
					0x0000001a,
					0x0000001b,
					0x0000001c,
					0x0000001d,
					0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
					0x0000001e,
					0x0000001f,
					0x00000020,
					0x00000021,
				},
			},
		},
		.dmem_offset = IPU_PSYS_DMEM_OFFSET,
	},
};

const struct ipu_buttress_ctrl isys_buttress_ctrl = {
	.subsys_id = IPU_IS,
	.ratio = IPU7_IS_FREQ_CTL_DEFAULT_RATIO,
	.ratio_shift = IPU_FREQ_CTL_RATIO_SHIFT,
	.cdyn = IPU_FREQ_CTL_CDYN,
	.cdyn_shift = IPU_FREQ_CTL_CDYN_SHIFT,
	.freq_ctl = BUTTRESS_REG_IS_WORKPOINT_REQ,
	.pwr_sts_shift = IPU_BUTTRESS_PWR_STATE_IS_PWR_SHIFT,
	.pwr_sts_mask = IPU_BUTTRESS_PWR_STATE_IS_PWR_MASK,
	.pwr_sts_on = IPU_BUTTRESS_PWR_STATE_UP_DONE,
	.pwr_sts_off = IPU_BUTTRESS_PWR_STATE_DN_DONE,
	.ovrd_clk = BUTTRESS_OVERRIDE_IS_CLK,
	.own_clk_ack = BUTTRESS_OWN_ACK_IS_CLK,
};

const struct ipu_buttress_ctrl psys_buttress_ctrl = {
	.subsys_id = IPU_PS,
	.ratio = IPU7_PS_FREQ_CTL_DEFAULT_RATIO,
	.ratio_shift = IPU_FREQ_CTL_RATIO_SHIFT,
	.cdyn = IPU_FREQ_CTL_CDYN,
	.cdyn_shift = IPU_FREQ_CTL_CDYN_SHIFT,
	.freq_ctl = BUTTRESS_REG_PS_WORKPOINT_REQ,
	.pwr_sts_shift = IPU_BUTTRESS_PWR_STATE_PS_PWR_SHIFT,
	.pwr_sts_mask = IPU_BUTTRESS_PWR_STATE_PS_PWR_MASK,
	.pwr_sts_on = IPU_BUTTRESS_PWR_STATE_UP_DONE,
	.pwr_sts_off = IPU_BUTTRESS_PWR_STATE_DN_DONE,
	.ovrd_clk = BUTTRESS_OVERRIDE_PS_CLK,
	.own_clk_ack = BUTTRESS_OWN_ACK_PS_CLK,
};

int ipu_buttress_psys_freq_get(void *data, u64 *val)
{
	struct ipu_device *isp = data;
	u32 reg_val;
	int ret;

	ret = pm_runtime_get_sync(&isp->psys->dev);
	if (ret < 0) {
		pm_runtime_put(&isp->psys->dev);
		dev_err(&isp->pdev->dev, "Runtime PM failed (%d)\n", ret);
		return ret;
	}

	reg_val = readl(isp->base + BUTTRESS_REG_PS_WORKPOINT_REQ);

	pm_runtime_put(&isp->psys->dev);

	*val = BUTTRESS_PS_FREQ_RATIO_STEP *
	    (reg_val & BUTTRESS_PS_FREQ_CTL_RATIO_MASK);

	return 0;
}

void ipu_internal_pdata_init(struct ipu_isys_internal_pdata *isys_ipdata,
			     struct ipu_psys_internal_pdata *psys_ipdata)
{
	isys_ipdata->csi2.nports = ARRAY_SIZE(ipu_csi_offsets);
	isys_ipdata->csi2.offsets = ipu_csi_offsets;
#ifdef CONFIG_VIDEO_INTEL_IPU_MGC
	isys_ipdata->tpg.ntpgs = ARRAY_SIZE(ipu_tpg_offsets);
	isys_ipdata->tpg.offsets = ipu_tpg_offsets;
	isys_ipdata->tpg.sels = NULL;
#endif
	isys_ipdata->num_parallel_streams = IPU7_ISYS_NUM_STREAMS;
	psys_ipdata->hw_variant.spc_offset = IPU7_PSYS_SPC_OFFSET;
}

static int ipu_isys_check_fwnode_graph(struct fwnode_handle *fwnode)
{
	struct fwnode_handle *endpoint;

	if (IS_ERR_OR_NULL(fwnode))
		return -EINVAL;

	endpoint = fwnode_graph_get_next_endpoint(fwnode, NULL);
	if (endpoint) {
		fwnode_handle_put(endpoint);
		return 0;
	}

	return ipu_isys_check_fwnode_graph(fwnode->secondary);
}

static struct ipu_bus_device *ipu_isys_init(struct pci_dev *pdev,
					    struct device *parent,
					    struct ipu_buttress_ctrl *ctrl,
					    void __iomem *base,
					    const struct ipu_isys_internal_pdata
					    *ipdata,
					    unsigned int nr)
{
	struct ipu_bus_device *isys;
	struct ipu_isys_pdata *pdata;
#if IS_ENABLED(CONFIG_INTEL_IPU7_ACPI)
	struct ipu_isys_subdev_pdata *acpi_pdata;
#endif
	struct fwnode_handle *fwnode = dev_fwnode(&pdev->dev);
	int ret;

	ret = ipu_isys_check_fwnode_graph(fwnode);
	if (ret) {
		if (fwnode && !IS_ERR_OR_NULL(fwnode->secondary)) {
			dev_err(&pdev->dev,
				"fwnode graph has no endpoints connection\n");
			return ERR_PTR(-EINVAL);
		}
#if defined(CONFIG_IPU_ISYS_BRIDGE)
		ret = cio2_bridge_init(pdev);
		if (ret) {
			dev_err_probe(&pdev->dev, ret,
				      "ipu_isys_bridge_init() failed\n");
			return ERR_PTR(ret);
		}
#endif
	}

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	pdata->base = base;
	pdata->ipdata = ipdata;

	isys = ipu_bus_add_device(pdev, parent, pdata, ctrl,
				  IPU_ISYS_NAME, nr);

#if IS_ENABLED(CONFIG_INTEL_IPU7_ACPI)
	if (!spdata) {
		dev_err(&pdev->dev, "No subdevice info provided");
		ipu_get_acpi_devices(isys, &isys->dev, &acpi_pdata, NULL,
				     isys_init_acpi_add_device);
		pdata->spdata = acpi_pdata;
	} else {
		dev_info(&pdev->dev, "Subdevice info found");
		ipu_get_acpi_devices(isys, &isys->dev, &acpi_pdata, &spdata,
				     isys_init_acpi_add_device);
	}
#endif

	if (IS_ERR(isys)) {
		dev_err_probe(&pdev->dev, PTR_ERR(isys),
			      "ipu_bus_add_device(isys) failed\n");
		return ERR_CAST(isys);
	}
	isys->subsys = IPU_IS;
	isys->mmu = ipu_mmu_init(&pdev->dev, base, ISYS_MMID,
				 &ipdata->hw_variant);
	if (IS_ERR(isys->mmu)) {
		dev_err_probe(&pdev->dev, PTR_ERR(isys),
			      "ipu_mmu_init(isys->mmu) failed\n");
		return ERR_CAST(isys->mmu);
	}

	isys->mmu->dev = &isys->dev;

	return isys;
}

static struct ipu_bus_device *ipu_psys_init(struct pci_dev *pdev,
					    struct device *parent,
					    struct ipu_buttress_ctrl *ctrl,
					    void __iomem *base,
					    const struct ipu_psys_internal_pdata
					    *ipdata, unsigned int nr)
{
	struct ipu_bus_device *psys;
	struct ipu_psys_pdata *pdata;

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	pdata->base = base;
	pdata->ipdata = ipdata;

	psys = ipu_bus_add_device(pdev, parent, pdata, ctrl,
				  IPU_PSYS_NAME, nr);
	if (IS_ERR(psys)) {
		dev_err_probe(&pdev->dev, PTR_ERR(psys),
			      "ipu_bus_add_device(psys) failed\n");
		return ERR_CAST(psys);
	}

	psys->subsys = IPU_PS;
	psys->mmu = ipu_mmu_init(&pdev->dev, base, PSYS_MMID,
				 &ipdata->hw_variant);
	if (IS_ERR(psys->mmu)) {
		dev_err_probe(&pdev->dev, PTR_ERR(psys),
			      "ipu_mmu_init(psys->mmu) failed\n");
		return ERR_CAST(psys->mmu);
	}

	psys->mmu->dev = &psys->dev;

	return psys;
}

int ipu_fw_authenticate(void *data, u64 val)
{
	struct ipu_device *isp = data;
	int ret;

	if (!isp->secure_mode)
		return -EINVAL;

	ret = ipu_buttress_reset_authentication(isp);
	if (ret) {
		dev_err(&isp->pdev->dev, "Failed to reset authentication!\n");
		return ret;
	}

	ret = pm_runtime_get_sync(&isp->psys->dev);
	if (ret < 0) {
		dev_err(&isp->pdev->dev, "Runtime PM failed (%d)\n", ret);
		return ret;
	}

	ret = ipu_buttress_authenticate(isp);
	if (ret) {
		dev_err(&isp->pdev->dev, "FW authentication failed\n");
		return ret;
	}

	pm_runtime_put(&isp->psys->dev);

	return 0;
}
EXPORT_SYMBOL(ipu_fw_authenticate);
DEFINE_SIMPLE_ATTRIBUTE(authenticate_fops, NULL, ipu_fw_authenticate, "%llu\n");

static struct ia_gofo_msg_log_info_ts fw_error_log[IPU_SUBSYS_NUM];
void ipu_dump_fw_error_log(const struct ipu_bus_device *adev)
{
	void __iomem *reg = adev->isp->base + ((adev->subsys == IPU_IS) ?
		BUTTRESS_REG_FW_GP24 : BUTTRESS_REG_FW_GP8);

	memcpy_fromio(&fw_error_log[adev->subsys], reg,
		      sizeof(fw_error_log[adev->subsys]));
}
EXPORT_SYMBOL_GPL(ipu_dump_fw_error_log);

#ifdef CONFIG_DEBUG_FS
static int resume_ipu_bus_device(struct ipu_bus_device *adev)
{
	struct device *dev = &adev->dev;
	const struct dev_pm_ops *pm = dev->driver ? dev->driver->pm : NULL;

	if (!pm || !pm->resume)
		return -EIO;

	return pm->resume(dev);
}

static int suspend_ipu_bus_device(struct ipu_bus_device *adev)
{
	struct device *dev = &adev->dev;
	const struct dev_pm_ops *pm = dev->driver ? dev->driver->pm : NULL;

	if (!pm || !pm->suspend)
		return -EIO;

	return pm->suspend(dev);
}

static int force_suspend_get(void *data, u64 *val)
{
	struct ipu_device *isp = data;
	struct ipu_buttress *b = &isp->buttress;

	*val = b->force_suspend;
	return 0;
}

static int force_suspend_set(void *data, u64 val)
{
	struct ipu_device *isp = data;
	struct ipu_buttress *b = &isp->buttress;
	int ret = 0;

	if (val == b->force_suspend)
		return 0;

	if (val) {
		b->force_suspend = 1;
		ret = suspend_ipu_bus_device(isp->psys);
		if (ret) {
			dev_err(&isp->pdev->dev, "Failed to suspend psys\n");
			return ret;
		}
		ret = suspend_ipu_bus_device(isp->isys);
		if (ret) {
			dev_err(&isp->pdev->dev, "Failed to suspend isys\n");
			return ret;
		}
		ret = pci_set_power_state(isp->pdev, PCI_D3hot);
		if (ret) {
			dev_err(&isp->pdev->dev,
				"Failed to suspend IUnit PCI device\n");
			return ret;
		}
	} else {
		ret = pci_set_power_state(isp->pdev, PCI_D0);
		if (ret) {
			dev_err(&isp->pdev->dev,
				"Failed to suspend IUnit PCI device\n");
			return ret;
		}
		ret = resume_ipu_bus_device(isp->isys);
		if (ret) {
			dev_err(&isp->pdev->dev, "Failed to resume isys\n");
			return ret;
		}
		ret = resume_ipu_bus_device(isp->psys);
		if (ret) {
			dev_err(&isp->pdev->dev, "Failed to resume psys\n");
			return ret;
		}
		b->force_suspend = 0;
	}

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(force_suspend_fops, force_suspend_get,
			force_suspend_set, "%llu\n");

struct debugfs_blob_wrapper isys_fw_error;
struct debugfs_blob_wrapper psys_fw_error;

static int ipu_init_debugfs(struct ipu_device *isp)
{
	struct dentry *file;
	struct dentry *dir;

	dir = debugfs_create_dir(pci_name(isp->pdev), NULL);
	if (!dir)
		return -ENOMEM;

	file = debugfs_create_file("force_suspend", 0700, dir, isp,
				   &force_suspend_fops);
	if (!file)
		goto err;
	file = debugfs_create_file("authenticate", 0700, dir, isp,
				   &authenticate_fops);
	if (!file)
		goto err;

	isys_fw_error.data = &fw_error_log[IPU_IS];
	isys_fw_error.size = sizeof(fw_error_log[IPU_IS]);
	file = debugfs_create_blob("is_fw_error", 0400, dir, &isys_fw_error);
	if (!file)
		goto err;
	psys_fw_error.data = &fw_error_log[IPU_PS];
	psys_fw_error.size = sizeof(fw_error_log[IPU_PS]);
	file = debugfs_create_blob("ps_fw_error", 0400, dir, &psys_fw_error);
	if (!file)
		goto err;

	isp->ipu_dir = dir;

	if (ipu_buttress_debugfs_init(isp))
		goto err;

	return 0;
err:
	debugfs_remove_recursive(dir);
	return -ENOMEM;
}

static void ipu_remove_debugfs(struct ipu_device *isp)
{
	/*
	 * Since isys and psys debugfs dir will be created under ipu root dir,
	 * mark its dentry to NULL to avoid duplicate removal.
	 */
	debugfs_remove_recursive(isp->ipu_dir);
	isp->ipu_dir = NULL;
}
#endif /* CONFIG_DEBUG_FS */

static int ipu_pci_config_setup(struct pci_dev *dev)
{
	u16 pci_command;
	int ret;

	pci_read_config_word(dev, PCI_COMMAND, &pci_command);
	pci_command |= PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER;
	pci_write_config_word(dev, PCI_COMMAND, pci_command);

	ret = pci_enable_msi(dev);
	if (ret)
		dev_err(&dev->dev, "Failed to enable msi (%d)\n", ret);

	return ret;
}

static int ipu_map_fw_code_region(struct ipu_bus_device *sys,
				  void *data, size_t size)
{
	struct sg_table *sgt = &sys->fw_code_region_sgt;
	struct page **pages;
	unsigned long n_pages, i;
	int ret;

	n_pages = PAGE_ALIGN(size) >> PAGE_SHIFT;

	pages = kmalloc_array(n_pages, sizeof(*pages), GFP_KERNEL);
	if (!pages)
		return -ENOMEM;

	for (i = 0; i < n_pages; i++) {
		struct page *p = vmalloc_to_page(data);

		if (!p) {
			ret = -ENODEV;
			goto out;
		}

		pages[i] = p;
		data += PAGE_SIZE;
	}

	ret = sg_alloc_table_from_pages(sgt, pages, n_pages, 0, size,
					GFP_KERNEL);
	if (ret) {
		ret = -ENOMEM;
		goto out;
	}

	ret = dma_map_sgtable(&sys->dev, sgt, DMA_TO_DEVICE, 0);
	if (ret < 0) {
		dev_err(&sys->dev, "map fw code[%lu pages %u nents] failed\n",
			n_pages, sgt->nents);
		ret = -ENOMEM;
		sg_free_table(sgt);
		goto out;
	}

	dev_dbg(&sys->dev, "fw code region mapped at 0x%llx entries %d\n",
		sgt->sgl->dma_address, sgt->nents);

	dma_sync_sgtable_for_device(&sys->dev, sgt, DMA_TO_DEVICE);
out:
	kfree(pages);

	return ret;
}

static void ipu_unmap_fw_code_region(struct ipu_bus_device *sys)
{
	dma_unmap_sg(&sys->dev, sys->fw_code_region_sgt.sgl,
		     sys->fw_code_region_sgt.nents, DMA_TO_DEVICE);
	sg_free_table(&sys->fw_code_region_sgt);
}

static int ipu_init_fw_code_region_by_sys(struct ipu_bus_device *sys,
					  char *sys_name)
{
	struct ipu_device *isp = sys->isp;
	int ret;

	/* Copy FW binaries to specific location. */
	ret = ipu_cpd_copy_binary(isp->cpd_fw->data, sys_name,
				  isp->fw_code_region, &sys->fw_entry);
	if (ret) {
		dev_err(&sys->dev, "%s binary not found.\n", sys_name);
		return ret;
	}

	ret = pm_runtime_get_sync(&sys->dev);
	if (ret < 0) {
		dev_err(&sys->dev, "Failed to get runtime PM\n");
		return ret;
	}

	ret = ipu_mmu_hw_init(sys->mmu);
	if (ret) {
		dev_err(&sys->dev, "Failed to set mmu hw\n");
		pm_runtime_put(&sys->dev);
		return ret;
	}

	/* Map code region. */
	ret = ipu_map_fw_code_region(sys, isp->fw_code_region,
				     IPU_FW_CODE_REGION_SIZE);
	if (ret)
		dev_err(&sys->dev,
			"Failed to map fw code region for %s.\n", sys_name);

	ipu_mmu_hw_cleanup(sys->mmu);
	pm_runtime_put(&sys->dev);

	return ret;
}

static int ipu_init_fw_code_region(struct ipu_device *isp)
{
	int ret;

	/*
	 * Allocate and map memory for FW execution.
	 * Not required in secure mode, in which FW runs in IMR.
	 */
	isp->fw_code_region = vmalloc(IPU_FW_CODE_REGION_SIZE);
	if (!isp->fw_code_region)
		return -ENOMEM;

	ret = ipu_init_fw_code_region_by_sys(isp->isys, "isys");
	if (ret)
		return ret;

	ret = ipu_init_fw_code_region_by_sys(isp->psys, "psys");
	if (ret)
		return ret;

	return 0;
}

static int ipu_pci_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct ipu_isys_internal_pdata *isys_ipdata;
	struct ipu_psys_internal_pdata *psys_ipdata;
	struct ipu_device *isp;
	phys_addr_t phys, pb_phys;
	void __iomem *const *iomap;
	void __iomem *isys_base = NULL;
	void __iomem *psys_base = NULL;
	struct ipu_buttress_ctrl *isys_ctrl = NULL, *psys_ctrl = NULL;
	unsigned int dma_mask = IPU_DMA_MASK;
	struct fwnode_handle *fwnode = dev_fwnode(&pdev->dev);
	u32 is_es;
	int ret;

	if (!fwnode || fwnode_property_read_u32(fwnode, "is_es", &is_es))
		is_es = 0;

	isp = devm_kzalloc(&pdev->dev, sizeof(*isp), GFP_KERNEL);
	if (!isp)
		return -ENOMEM;

	dev_set_name(&pdev->dev, "intel-ipu");
	isp->pdev = pdev;
	INIT_LIST_HEAD(&isp->devices);

	ret = pcim_enable_device(pdev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to enable CI ISP device (%d)\n",
			ret);
		return ret;
	}

	dev_info(&pdev->dev, "Device 0x%x (rev: 0x%x)\n",
		 pdev->device, pdev->revision);

	phys = pci_resource_start(pdev, IPU_PCI_BAR);
	pb_phys = pci_resource_start(pdev, IPU_PCI_PBBAR);
	dev_info(&pdev->dev, "BAR0 base %llx BAR2 base %llx\n", phys, pb_phys);

	ret = pcim_iomap_regions(pdev, BIT(IPU_PCI_BAR) | BIT(IPU_PCI_PBBAR),
				 pci_name(pdev));
	if (ret) {
		dev_err(&pdev->dev, "Failed to I/O memory remapping (%d)\n",
			ret);
		return ret;
	}

	iomap = pcim_iomap_table(pdev);
	if (!iomap) {
		dev_err(&pdev->dev, "Failed to iomap table (%d)\n", ret);
		return -ENODEV;
	}

	isp->base = iomap[IPU_PCI_BAR];
	isp->pb_base = iomap[IPU_PCI_PBBAR];
	dev_info(&pdev->dev, "BAR0 mapped at %p BAR2 mapped at %p\n",
		 isp->base, isp->pb_base);

	pci_set_drvdata(pdev, isp);
	pci_set_master(pdev);

	switch (id->device) {
	case IPU7_PCI_ID:
		isp->hw_ver = IPU7_VER_7;
		isp->cpd_fw_name = IPU7_FIRMWARE_NAME;
		isys_ipdata = &ipu7_isys_ipdata;
		psys_ipdata = &ipu7_psys_ipdata;
		break;
	case IPU7P5_PCI_ID:
		isp->hw_ver = IPU7_VER_7P5;
		isp->cpd_fw_name = IPU7P5_FIRMWARE_NAME;
		isys_ipdata = &ipu7p5_isys_ipdata;
		psys_ipdata = &ipu7p5_psys_ipdata;
		break;
	default:
		WARN(1, "Unsupported IPU device");
		return -ENODEV;
	}

	ipu_internal_pdata_init(isys_ipdata, psys_ipdata);

	isys_base = isp->base + isys_ipdata->hw_variant.offset;
	psys_base = isp->base + psys_ipdata->hw_variant.offset;

	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(dma_mask));
	if (ret) {
		dev_err(&pdev->dev, "Failed to set DMA mask (%d)\n", ret);
		return ret;
	}

	dma_set_max_seg_size(&pdev->dev, UINT_MAX);

	ret = ipu_pci_config_setup(pdev);
	if (ret)
		return ret;

	ret = devm_request_threaded_irq(&pdev->dev, pdev->irq,
					ipu_buttress_isr,
					ipu_buttress_isr_threaded,
					IRQF_SHARED, IPU_NAME, isp);
	if (ret) {
		dev_err(&pdev->dev, "Requesting irq failed(%d)\n", ret);
		return ret;
	}

	ret = ipu_buttress_init(isp);
	if (ret)
		return ret;

	dev_info(&pdev->dev, "cpd file name: %s\n", isp->cpd_fw_name);

	ret = request_firmware(&isp->cpd_fw, isp->cpd_fw_name, &pdev->dev);
	if (ret) {
		dev_err(&isp->pdev->dev, "Requesting signed firmware failed\n");
		goto buttress_exit;
	}

	ret = ipu_cpd_validate_cpd_file(isp, isp->cpd_fw->data,
					isp->cpd_fw->size);
	if (ret) {
		dev_err(&isp->pdev->dev, "Failed to validate cpd\n");
		goto out_ipu_bus_del_devices;
	}

	/*
	 * NOTE Device hierarchy below is important to ensure proper
	 * runtime suspend and resume order.
	 * Also registration order is important to ensure proper
	 * suspend and resume order during system
	 * suspend. Registration order is as follows:
	 * isys->psys
	 */
	isys_ctrl = devm_kzalloc(&pdev->dev, sizeof(*isys_ctrl), GFP_KERNEL);
	if (!isys_ctrl) {
		ret = -ENOMEM;
		goto out_ipu_bus_del_devices;
	}

	/* Init butress control with default values based on the HW */
	memcpy(isys_ctrl, &isys_buttress_ctrl, sizeof(*isys_ctrl));

	isp->isys = ipu_isys_init(pdev, &pdev->dev,
				  isys_ctrl, isys_base,
				  isys_ipdata,
				  0);
	if (IS_ERR(isp->isys)) {
		ret = PTR_ERR(isp->isys);
		goto out_ipu_bus_del_devices;
	}

	psys_ctrl = devm_kzalloc(&pdev->dev, sizeof(*psys_ctrl), GFP_KERNEL);
	if (!psys_ctrl) {
		ret = -ENOMEM;
		goto out_ipu_bus_del_devices;
	}

	/* Init butress control with default values based on the HW */
	memcpy(psys_ctrl, &psys_buttress_ctrl, sizeof(*psys_ctrl));

	isp->psys = ipu_psys_init(pdev, &isp->isys->dev,
				  psys_ctrl, psys_base,
				  psys_ipdata, 0);
	if (IS_ERR(isp->psys)) {
		ret = PTR_ERR(isp->psys);
		goto out_ipu_bus_del_devices;
	}

	if (!isp->secure_mode) {
		ret = ipu_init_fw_code_region(isp);
		if (ret)
			goto out_ipu_bus_del_devices;
	} else {
		ret = pm_runtime_get_sync(&isp->psys->dev);
		if (ret < 0) {
			dev_err(&isp->psys->dev, "Failed to get runtime PM\n");
			goto out_ipu_bus_del_devices;
		}

		ret = ipu_mmu_hw_init(isp->psys->mmu);
		if (ret) {
			dev_err(&isp->pdev->dev, "Failed to set mmu hw\n");
			goto out_ipu_bus_del_devices;
		}

		ret = ipu_map_fw_code_region(isp->psys,
					     (void *)isp->cpd_fw->data,
					     isp->cpd_fw->size);
		if (ret) {
			dev_err(&isp->pdev->dev, "failed to map fw image\n");
			goto out_ipu_bus_del_devices;
		}

		ret = ipu_buttress_authenticate(isp);
		if (ret) {
			dev_err(&isp->pdev->dev,
				"FW authentication failed(%d)\n", ret);
			goto out_ipu_bus_del_devices;
		}

		ipu_mmu_hw_cleanup(isp->psys->mmu);
		pm_runtime_put(&isp->psys->dev);
	}

#ifdef CONFIG_DEBUG_FS
	ret = ipu_init_debugfs(isp);
	if (ret) {
		dev_err(&pdev->dev, "Failed to initialize debugfs");
		goto out_ipu_bus_del_devices;
	}
#endif
	pm_runtime_put_noidle(&pdev->dev);
	pm_runtime_allow(&pdev->dev);

	isp->ipu_bus_ready_to_probe = true;

	return 0;

out_ipu_bus_del_devices:
	if (!IS_ERR_OR_NULL(isp->isys) && isp->isys->fw_code_region_sgt.nents)
		ipu_unmap_fw_code_region(isp->isys);
	if (!IS_ERR_OR_NULL(isp->psys) && isp->psys->fw_code_region_sgt.nents)
		ipu_unmap_fw_code_region(isp->psys);
	if (!IS_ERR_OR_NULL(isp->fw_code_region))
		vfree(isp->fw_code_region);
	if (!IS_ERR_OR_NULL(isp->psys) && !IS_ERR_OR_NULL(isp->psys->mmu))
		ipu_mmu_cleanup(isp->psys->mmu);
	if (!IS_ERR_OR_NULL(isp->isys) && !IS_ERR_OR_NULL(isp->isys->mmu))
		ipu_mmu_cleanup(isp->isys->mmu);
	if (!IS_ERR_OR_NULL(isp->psys))
		pm_runtime_put(&isp->psys->dev);
	ipu_bus_del_devices(pdev);
	release_firmware(isp->cpd_fw);
buttress_exit:
	ipu_buttress_exit(isp);

	return ret;
}

static void ipu_pci_remove(struct pci_dev *pdev)
{
	struct ipu_device *isp = pci_get_drvdata(pdev);

#ifdef CONFIG_DEBUG_FS
	ipu_remove_debugfs(isp);
#endif
	ipu_unmap_fw_code_region(isp->isys);
	ipu_unmap_fw_code_region(isp->psys);
	vfree(isp->fw_code_region);

	ipu_bus_del_devices(pdev);

	pm_runtime_forbid(&pdev->dev);
	pm_runtime_get_noresume(&pdev->dev);

	pci_release_regions(pdev);
	pci_disable_device(pdev);

	ipu_buttress_exit(isp);

	release_firmware(isp->cpd_fw);

	ipu_mmu_cleanup(isp->psys->mmu);
	ipu_mmu_cleanup(isp->isys->mmu);
}

static void ipu_pci_reset_prepare(struct pci_dev *pdev)
{
	struct ipu_device *isp = pci_get_drvdata(pdev);

	dev_warn(&pdev->dev, "FLR prepare\n");
	pm_runtime_forbid(&isp->pdev->dev);
}

static void ipu_pci_reset_done(struct pci_dev *pdev)
{
	struct ipu_device *isp = pci_get_drvdata(pdev);

	ipu_buttress_restore(isp);
	if (isp->secure_mode)
		ipu_buttress_reset_authentication(isp);

	ipu_bus_flr_recovery();
	isp->ipc_reinit = true;
	pm_runtime_allow(&isp->pdev->dev);

	dev_warn(&pdev->dev, "FLR completed\n");
}

#ifdef CONFIG_PM

/*
 * PCI base driver code requires driver to provide these to enable
 * PCI device level PM state transitions (D0<->D3)
 */
static int ipu_suspend(struct device *dev)
{
	return 0;
}

static int ipu_resume(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct ipu_device *isp = pci_get_drvdata(pdev);
	struct ipu_buttress *b = &isp->buttress;
	int ret;

	isp->secure_mode = ipu_buttress_get_secure_mode(isp);
	dev_info(dev, "IPU in %s mode\n",
		 isp->secure_mode ? "secure" : "non-secure");

	ipu_buttress_restore(isp);

	ret = ipu_buttress_ipc_reset(isp, &b->cse);
	if (ret)
		dev_err(&isp->pdev->dev, "IPC reset protocol failed!\n");

	ret = pm_runtime_get_sync(&isp->psys->dev);
	if (ret < 0) {
		dev_err(&isp->psys->dev, "Failed to get runtime PM\n");
		return 0;
	}

	ret = ipu_buttress_authenticate(isp);
	if (ret)
		dev_err(&isp->pdev->dev, "FW authentication failed(%d)\n",
			ret);

	pm_runtime_put(&isp->psys->dev);

	return 0;
}

static int ipu_runtime_resume(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct ipu_device *isp = pci_get_drvdata(pdev);
	int ret;

	ipu_buttress_restore(isp);

	if (isp->ipc_reinit) {
		struct ipu_buttress *b = &isp->buttress;

		isp->ipc_reinit = false;
		ret = ipu_buttress_ipc_reset(isp, &b->cse);
		if (ret)
			dev_err(&isp->pdev->dev,
				"IPC reset protocol failed!\n");
	}

	return 0;
}

static const struct dev_pm_ops ipu_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(&ipu_suspend, &ipu_resume)
	    SET_RUNTIME_PM_OPS(&ipu_suspend,	/* Same as in suspend flow */
			       &ipu_runtime_resume,
			       NULL)
};

#define IPU_PM (&ipu_pm_ops)
#else
#define IPU_PM NULL
#endif

static const struct pci_device_id ipu_pci_tbl[] = {
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, IPU7_PCI_ID)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, IPU7P5_PCI_ID)},
	{0,}
};
MODULE_DEVICE_TABLE(pci, ipu_pci_tbl);

static const struct pci_error_handlers pci_err_handlers = {
	.reset_prepare = ipu_pci_reset_prepare,
	.reset_done = ipu_pci_reset_done,
};

static struct pci_driver ipu_pci_driver = {
	.name = IPU_NAME,
	.id_table = ipu_pci_tbl,
	.probe = ipu_pci_probe,
	.remove = ipu_pci_remove,
	.driver = {
		   .pm = IPU_PM,
		   },
	.err_handler = &pci_err_handlers,
};

static int __init ipu_init(void)
{
	int ret = ipu_bus_register();

	if (ret) {
		pr_warn("can't register ipu bus (%d)\n", ret);
		return ret;
	}

	ret = pci_register_driver(&ipu_pci_driver);
	if (ret) {
		pr_warn("can't register pci driver (%d)\n", ret);
		goto out_pci_register_driver;
	}

	return 0;

out_pci_register_driver:
	ipu_bus_unregister();

	return ret;
}

static void __exit ipu_exit(void)
{
	pci_unregister_driver(&ipu_pci_driver);
	ipu_bus_unregister();
}

module_init(ipu_init);
module_exit(ipu_exit);

MODULE_IMPORT_NS(INTEL_IPU_BRIDGE);
MODULE_AUTHOR("Bingbu Cao <bingbu.cao@intel.com>");
MODULE_AUTHOR("Tianshu Qiu <tian.shu.qiu@intel.com>");
MODULE_AUTHOR("Qingwu Zhang <qingwu.zhang@intel.com>");
MODULE_AUTHOR("Intel");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Intel ipu pci driver");
