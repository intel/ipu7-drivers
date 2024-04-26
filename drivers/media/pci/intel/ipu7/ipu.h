/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2013 - 2024 Intel Corporation */

#ifndef IPU_H
#define IPU_H

#include <linux/ioport.h>
#include <linux/list.h>
#include <uapi/linux/media.h>
#include <linux/version.h>

#include "ipu-bus.h"
#include "ipu-buttress.h"

#define IPU7_PCI_ID	0x645d
#define IPU7P5_PCI_ID	0xb05d

enum ipu_version {
	IPU7_VER_INVALID = 0,
	IPU7_VER_7 = 1,
	IPU7_VER_7P5 = 2,
};

static inline bool is_ipu7p5(u8 hw_ver)
{
	return hw_ver == IPU7_VER_7P5;
}

#define IPU_UNIFIED_OFFSET			0

/*
 * ISYS DMA can overshoot. For higher resolutions over allocation is one line
 * but it must be at minimum 1024 bytes. Value could be different in
 * different versions / generations thus provide it via platform data.
 */
#define IPU_ISYS_OVERALLOC_MIN		1024

/*
 * Physical pages in GDA is 128, page size is 2K for IPU6, 1K for others.
 */
#define IPU_DEVICE_GDA_NR_PAGES		128

/*
 * Virtualization factor to calculate the available virtual pages.
 */
#define IPU_DEVICE_GDA_VIRT_FACTOR	32

#define IPU_FW_CODE_REGION_SIZE		0x1000000 /* 16MB */
#define IPU_FW_CODE_REGION_START	0x4000000 /* 64MB */
#define IPU_FW_CODE_REGION_END		(IPU_FW_CODE_REGION_START + \
					 IPU_FW_CODE_REGION_SIZE) /* 80MB */

struct pci_dev;
struct list_head;
struct firmware;

struct ipu_device {
	struct pci_dev *pdev;
	struct list_head devices;
	struct ipu_bus_device *isys;
	struct ipu_bus_device *psys;
	struct ipu_buttress buttress;

	const struct firmware *cpd_fw;
	const char *cpd_fw_name;
	/* Only for non-secure mode. */
	void *fw_code_region;

	void __iomem *base;
	void __iomem *pb_base;
#ifdef CONFIG_DEBUG_FS
	struct dentry *ipu_dir;
#endif
	u8 hw_ver;
	bool ipc_reinit;
	bool secure_mode;
	bool ipu_bus_ready_to_probe;
};

#define IPU_DMA_MASK	39
#define IPU_LIB_CALL_TIMEOUT_MS		2000
#define IPU_PSYS_CMD_TIMEOUT_MS	2000
#define IPU_PSYS_OPEN_CLOSE_TIMEOUT_US	   50
#define IPU_PSYS_OPEN_CLOSE_RETRY (10000 / IPU_PSYS_OPEN_CLOSE_TIMEOUT_US)

#define IPU_ISYS_CSI2_NAME IPU_NAME "-csi2"
#define IPU_ISYS_NAME IPU_NAME "-isys"
#define IPU_PSYS_NAME IPU_NAME "-psys"
#define IPU_BUTTRESS_NAME IPU_NAME "-buttress"

#define IPU_MMU_NAME				IPU_NAME "-mmu"
#define IPU_MMU_ADDRESS_BITS			32
/* FW is accessible within the first 2 GiB only in non-secure mode. */
#define IPU_MMU_ADDRESS_BITS_NON_SECURE		31

#define IPU7_IS_MMU_NUM				4
#define IPU7_PS_MMU_NUM				4
#define IPU7P5_IS_MMU_NUM			4
#define IPU7P5_PS_MMU_NUM			4
#define IPU_MMU_MAX_NUM				4 /* max(IS, PS) */
#define IPU_MMU_MAX_TLB_L1_STREAMS		40
#define IPU_MMU_MAX_TLB_L2_STREAMS		40
#define IPU_ZLX_MAX_NUM				32
#define IPU_ZLX_POOL_NUM			8
#define IPU_UAO_PLANE_MAX_NUM			64

/*
 * To maximize the IOSF utlization, IPU need to send requests in bursts.
 * At the DMA interface with the buttress, there are CDC FIFOs with burst
 * collection capability. CDC FIFO burst collectors have a configurable
 * threshold and is configured based on the outcome of performance measurements.
 *
 * isys has 3 ports with IOSF interface for VC0, VC1 and VC2
 * psys has 4 ports with IOSF interface for VC0, VC1w, VC1r and VC2
 *
 * Threshold values are pre-defined and are arrived at after performance
 * evaluations on a type of IPU
 */
#define IPU_MAX_VC_IOSF_PORTS		4

/*
 * IPU must configure correct arbitration mechanism related to the IOSF VC
 * requests. There are two options per VC0 and VC1 - > 0 means rearbitrate on
 * stall and 1 means stall until the request is completed.
 */
#define IPU_BTRS_ARB_MODE_TYPE_REARB	0
#define IPU_BTRS_ARB_MODE_TYPE_STALL	1

/* Currently chosen arbitration mechanism for VC0 */
#define IPU_BTRS_ARB_STALL_MODE_VC0	\
			IPU_BTRS_ARB_MODE_TYPE_REARB

/* Currently chosen arbitration mechanism for VC1 */
#define IPU_BTRS_ARB_STALL_MODE_VC1	\
			IPU_BTRS_ARB_MODE_TYPE_REARB

struct ipu_isys_subdev_pdata;

/* One L2 entry maps 1024 L1 entries and one L1 entry per page */
#define IPU_MMUV2_L2_RANGE		(1024 * PAGE_SIZE)
/* Max L2 blocks per stream */
#define IPU_MMUV2_MAX_L2_BLOCKS		2
/* Max L1 blocks per stream */
#define IPU_MMUV2_MAX_L1_BLOCKS		16
#define IPU_MMUV2_TRASH_RANGE		(IPU_MMUV2_L2_RANGE * \
						 IPU_MMUV2_MAX_L2_BLOCKS)
/* Entries per L1 block */
#define MMUV2_ENTRIES_PER_L1_BLOCK		16
#define MMUV2_TRASH_L1_BLOCK_OFFSET		(MMUV2_ENTRIES_PER_L1_BLOCK * \
						 PAGE_SIZE)
#define MMUV2_TRASH_L2_BLOCK_OFFSET		IPU_MMUV2_L2_RANGE

struct ipu_mmu_hw {
	char name[32];

	void __iomem *base;
	void __iomem *zlx_base;
	void __iomem *uao_base;

	u32 offset;
	u32 zlx_offset;
	u32 uao_offset;

	u32 info_bits;
	u32 refill;
	u32 collapse_en_bitmap;
	u32 at_sp_arb_cfg;

	u32 l1_block;
	u32 l2_block;

	u8 nr_l1streams;
	u8 nr_l2streams;
	u32 l1_block_sz[IPU_MMU_MAX_TLB_L1_STREAMS];
	u32 l2_block_sz[IPU_MMU_MAX_TLB_L2_STREAMS];

	u8 zlx_nr;
	u32 zlx_axi_pool[IPU_ZLX_POOL_NUM];
	u32 zlx_en[IPU_ZLX_MAX_NUM];
	u32 zlx_conf[IPU_ZLX_MAX_NUM];

	u32 uao_p_num;
	u32 uao_p2tlb[IPU_UAO_PLANE_MAX_NUM];
};

struct ipu_mmu_pdata {
	unsigned int nr_mmus;
	struct ipu_mmu_hw mmu_hw[IPU_MMU_MAX_NUM];
	int mmid;
};

struct ipu_isys_csi2_pdata {
	void __iomem *base;
};

#define IPU_EV_AUTO 0xff

struct ipu_isys_internal_csi2_pdata {
	unsigned int nports;
	unsigned int *offsets;
};

#ifdef CONFIG_VIDEO_INTEL_IPU_MGC
struct ipu_isys_internal_tpg_pdata {
	unsigned int ntpgs;
	unsigned int *offsets;
	unsigned int *sels;
};
#endif

/*
 * One place to handle all the IPU HW variations
 */
struct ipu_hw_variants {
	unsigned long offset;
	unsigned int nr_mmus;
	struct ipu_mmu_hw mmu_hw[IPU_MMU_MAX_NUM];
	u8 cdc_fifos;
	u8 cdc_fifo_threshold[IPU_MAX_VC_IOSF_PORTS];
	u32 dmem_offset;
	u32 spc_offset;	/* SPC offset from psys base */
};

struct ipu_isys_internal_pdata {
	struct ipu_isys_internal_csi2_pdata csi2;
#ifdef CONFIG_VIDEO_INTEL_IPU_MGC
	struct ipu_isys_internal_tpg_pdata tpg;
#endif
	struct ipu_hw_variants hw_variant;
	u32 num_parallel_streams;
	u32 isys_dma_overshoot;
};

struct ipu_isys_pdata {
	void __iomem *base;
	const struct ipu_isys_internal_pdata *ipdata;
};

struct ipu_psys_internal_pdata {
	struct ipu_hw_variants hw_variant;
};

struct ipu_psys_pdata {
	void __iomem *base;
	const struct ipu_psys_internal_pdata *ipdata;
};

int ipu_fw_authenticate(void *data, u64 val);
int request_cpd_fw(const struct firmware **firmware_p, const char *name,
		   struct device *device);
extern enum ipu_version ipu_ver;
void ipu_internal_pdata_init(struct ipu_isys_internal_pdata *isys_ipdata,
			     struct ipu_psys_internal_pdata *psys_ipdata);
void ipu_dump_fw_error_log(const struct ipu_bus_device *adev);
#if defined(CONFIG_IPU_ISYS_BRIDGE)
int cio2_bridge_init(struct pci_dev *cio2);
#endif
#endif /* IPU_H */
