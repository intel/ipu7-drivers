// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2013 - 2024 Intel Corporation

#include <asm/cacheflush.h>

#include <linux/device.h>
#include <linux/iopoll.h>
#include <linux/iova.h>
#include <linux/module.h>
#include <linux/pfn.h>
#include <linux/sizes.h>

#include "ipu.h"
#include "ipu-platform.h"
#include "ipu-dma.h"
#include "ipu-mmu.h"
#include "ipu-platform-regs.h"

#define ISP_PAGE_SHIFT		12
#define ISP_PAGE_SIZE		BIT(ISP_PAGE_SHIFT)
#define ISP_PAGE_MASK		(~(ISP_PAGE_SIZE - 1))

#define ISP_L1PT_SHIFT		22
#define ISP_L1PT_MASK		(~((1U << ISP_L1PT_SHIFT) - 1))

#define ISP_L2PT_SHIFT		12
#define ISP_L2PT_MASK		(~(ISP_L1PT_MASK | (~(ISP_PAGE_MASK))))

#define ISP_L1PT_PTES           1024
#define ISP_L2PT_PTES           1024

#define ISP_PADDR_SHIFT		12

#define REG_L1_PHYS		0x0004	/* 27-bit pfn */
#define REG_INFO		0x0008

#define TBL_PHYS_ADDR(a)	((phys_addr_t)(a) << ISP_PADDR_SHIFT)

#define MMU_TLB_INVALIDATE_TIMEOUT	2000

static __maybe_unused void mmu_irq_handler(struct ipu_mmu *mmu)
{
	u32 irq_cause;
	unsigned int i;

	for (i = 0; i < mmu->nr_mmus; i++) {
		irq_cause = readl(mmu->mmu_hw[i].base + MMU_REG_IRQ_CAUSE);
		pr_info("mmu %s irq_cause = 0x%x", mmu->mmu_hw[i].name,
			irq_cause);
		writel(0x1ffff, mmu->mmu_hw[i].base + MMU_REG_IRQ_CLEAR);
	}
}

static void tlb_invalidate(struct ipu_mmu *mmu)
{
	unsigned int i;
	unsigned long flags;
	int ret;
	u32 val;

	spin_lock_irqsave(&mmu->ready_lock, flags);
	if (!mmu->ready) {
		spin_unlock_irqrestore(&mmu->ready_lock, flags);
		return;
	}

	for (i = 0; i < mmu->nr_mmus; i++) {
		writel(0xffffffff, mmu->mmu_hw[i].base +
		       MMU_REG_INVALIDATE_0);

		/* Need check with HW, use l1streams or l2streams */
		if (mmu->mmu_hw[i].nr_l2streams > 31)
			writel(0xffffffff, mmu->mmu_hw[i].base +
			       MMU_REG_INVALIDATE_1);

		/*
		 * The TLB invalidation is a "single cycle" (IOMMU clock cycles)
		 * When the actual MMIO write reaches the IPU TLB Invalidate
		 * register, wmb() will force the TLB invalidate out if the CPU
		 * attempts to update the IOMMU page table (or sooner).
		 */
		wmb();

		/* wait invalidation done */
		ret = readl_poll_timeout_atomic(mmu->mmu_hw[i].base +
						MMU_REG_INVALIDATION_STATUS,
						val, !(val & 0x1), 500,
						MMU_TLB_INVALIDATE_TIMEOUT);
	}

	spin_unlock_irqrestore(&mmu->ready_lock, flags);
}

#ifdef DEBUG
static void page_table_dump(struct ipu_mmu_info *mmu_info)
{
	u32 l1_idx;

	dev_dbg(mmu_info->dev, "begin IOMMU page table dump\n");

	for (l1_idx = 0; l1_idx < ISP_L1PT_PTES; l1_idx++) {
		u32 l2_idx;
		u32 iova = (phys_addr_t)l1_idx << ISP_L1PT_SHIFT;

		if (mmu_info->l1_pt[l1_idx] == mmu_info->dummy_l2_pteval)
			continue;
		dev_dbg(mmu_info->dev,
			"l1 entry %u; iovas 0x%8.8x-0x%8.8x, at %p\n",
			l1_idx, iova, iova + ISP_PAGE_SIZE,
			(void *)TBL_PHYS_ADDR(mmu_info->l1_pt[l1_idx]));

		for (l2_idx = 0; l2_idx < ISP_L2PT_PTES; l2_idx++) {
			u32 *l2_pt = mmu_info->l2_pts[l1_idx];
			u32 iova2 = iova + (l2_idx << ISP_L2PT_SHIFT);

			if (l2_pt[l2_idx] == mmu_info->dummy_page_pteval)
				continue;

			dev_dbg(mmu_info->dev,
				"\tl2 entry %u; iova 0x%8.8x, phys %p\n",
				l2_idx, iova2,
				(void *)TBL_PHYS_ADDR(l2_pt[l2_idx]));
		}
	}

	dev_dbg(mmu_info->dev, "end IOMMU page table dump\n");
}
#endif /* DEBUG */

static dma_addr_t map_single(struct ipu_mmu_info *mmu_info, void *ptr)
{
	dma_addr_t dma;

	dma = dma_map_single(mmu_info->dev, ptr, PAGE_SIZE, DMA_BIDIRECTIONAL);
	if (dma_mapping_error(mmu_info->dev, dma))
		return 0;

	return dma;
}

static int get_dummy_page(struct ipu_mmu_info *mmu_info)
{
	dma_addr_t dma;
	void *pt = (void *)get_zeroed_page(GFP_ATOMIC | GFP_DMA32);

	if (!pt)
		return -ENOMEM;

	dev_dbg(mmu_info->dev, "dummy_page: get_zeroed_page() == %p\n", pt);

	dma = map_single(mmu_info, pt);
	if (!dma) {
		dev_err(mmu_info->dev, "Failed to map dummy page\n");
		goto err_free_page;
	}

	mmu_info->dummy_page = pt;
	mmu_info->dummy_page_pteval = dma >> ISP_PAGE_SHIFT;

	return 0;

err_free_page:
	free_page((unsigned long)pt);
	return -ENOMEM;
}

static void free_dummy_page(struct ipu_mmu_info *mmu_info)
{
	dma_unmap_single(mmu_info->dev,
			 TBL_PHYS_ADDR(mmu_info->dummy_page_pteval),
			 PAGE_SIZE, DMA_BIDIRECTIONAL);
	free_page((unsigned long)mmu_info->dummy_page);
}

static int alloc_dummy_l2_pt(struct ipu_mmu_info *mmu_info)
{
	dma_addr_t dma;
	u32 *pt = (u32 *)get_zeroed_page(GFP_ATOMIC | GFP_DMA32);
	unsigned int i;

	if (!pt)
		return -ENOMEM;

	dev_dbg(mmu_info->dev, "dummy_l2: get_zeroed_page() == %p\n", pt);

	dma = map_single(mmu_info, pt);
	if (!dma) {
		dev_err(mmu_info->dev, "Failed to map l2pt page\n");
		goto err_free_page;
	}

	for (i = 0; i < ISP_L2PT_PTES; i++)
		pt[i] = mmu_info->dummy_page_pteval;

	mmu_info->dummy_l2_pt = pt;
	mmu_info->dummy_l2_pteval = dma >> ISP_PAGE_SHIFT;

	return 0;

err_free_page:
	free_page((unsigned long)pt);
	return -ENOMEM;
}

static void free_dummy_l2_pt(struct ipu_mmu_info *mmu_info)
{
	dma_unmap_single(mmu_info->dev,
			 TBL_PHYS_ADDR(mmu_info->dummy_l2_pteval),
			 PAGE_SIZE, DMA_BIDIRECTIONAL);
	free_page((unsigned long)mmu_info->dummy_l2_pt);
}

static u32 *alloc_l1_pt(struct ipu_mmu_info *mmu_info)
{
	dma_addr_t dma;
	u32 *pt = (u32 *)get_zeroed_page(GFP_ATOMIC | GFP_DMA32);
	unsigned int i;

	if (!pt)
		return NULL;

	dev_dbg(mmu_info->dev, "alloc_l1: get_zeroed_page() == %p\n", pt);

	for (i = 0; i < ISP_L1PT_PTES; i++)
		pt[i] = mmu_info->dummy_l2_pteval;

	dma = map_single(mmu_info, pt);
	if (!dma) {
		dev_err(mmu_info->dev, "Failed to map l1pt page\n");
		goto err_free_page;
	}

	mmu_info->l1_pt_dma = dma >> ISP_PADDR_SHIFT;
	dev_dbg(mmu_info->dev, "l1 pt %p mapped at %llx\n", pt, dma);

	return pt;

err_free_page:
	free_page((unsigned long)pt);
	return NULL;
}

static u32 *alloc_l2_pt(struct ipu_mmu_info *mmu_info)
{
	u32 *pt = (u32 *)get_zeroed_page(GFP_ATOMIC | GFP_DMA32);
	unsigned int i;

	if (!pt)
		return NULL;

	dev_dbg(mmu_info->dev, "alloc_l2: get_zeroed_page() == %p\n", pt);

	for (i = 0; i < ISP_L1PT_PTES; i++)
		pt[i] = mmu_info->dummy_page_pteval;

	return pt;
}

static size_t l2_unmap(struct ipu_mmu_info *mmu_info, unsigned long iova,
		       phys_addr_t dummy, size_t size);
static int l2_map(struct ipu_mmu_info *mmu_info, unsigned long iova,
		  phys_addr_t paddr, size_t size)
{
	struct device *dev = mmu_info->dev;
	u32 l1_idx;
	u32 l1_entry;
	u32 *l2_pt, *l2_virt;
	unsigned int l2_idx;
	unsigned long flags;
	dma_addr_t dma;
	unsigned int l2_entries;
	size_t mapped_size = 0;
	int err = 0;

	spin_lock_irqsave(&mmu_info->lock, flags);

	paddr = ALIGN(paddr, ISP_PAGE_SIZE);
	for (l1_idx = iova >> ISP_L1PT_SHIFT;
	     size > 0 && l1_idx < ISP_L1PT_PTES; l1_idx++) {
		dev_dbg(dev,
			"mapping l2 page table for l1 index %u (iova %8.8x)\n",
			l1_idx, (u32)iova);

		l1_entry = mmu_info->l1_pt[l1_idx];
		if (l1_entry == mmu_info->dummy_l2_pteval) {
			l2_virt = mmu_info->l2_pts[l1_idx];
			if (likely(!l2_virt)) {
				l2_virt = alloc_l2_pt(mmu_info);
				if (!l2_virt) {
					spin_unlock_irqrestore(&mmu_info->lock,
							       flags);
					err = -ENOMEM;
					goto error;
				}
			}

			dma = map_single(mmu_info, l2_virt);
			if (!dma) {
				dev_err(dev, "Failed to map l2pt page\n");
				free_page((unsigned long)l2_virt);

				spin_unlock_irqrestore(&mmu_info->lock, flags);
				err = -EINVAL;
				goto error;
			}

			l1_entry = dma >> ISP_PADDR_SHIFT;

			dev_dbg(dev, "page for l1_idx %u %p allocated\n",
				l1_idx, l2_virt);
			mmu_info->l1_pt[l1_idx] = l1_entry;
			mmu_info->l2_pts[l1_idx] = l2_virt;

			clflush_cache_range(&mmu_info->l1_pt[l1_idx],
					    sizeof(mmu_info->l1_pt[l1_idx]));
		}

		l2_pt = mmu_info->l2_pts[l1_idx];
		l2_entries = 0;

		for (l2_idx = (iova & ISP_L2PT_MASK) >> ISP_L2PT_SHIFT;
		     size > 0 && l2_idx < ISP_L2PT_PTES; l2_idx++) {
			l2_pt[l2_idx] = paddr >> ISP_PADDR_SHIFT;

			dev_dbg(dev, "l2 index %u mapped as 0x%8.8x\n", l2_idx,
				l2_pt[l2_idx]);

			iova += ISP_PAGE_SIZE;
			paddr += ISP_PAGE_SIZE;
			mapped_size += ISP_PAGE_SIZE;
			size -= ISP_PAGE_SIZE;

			l2_entries++;
		}

		if (l2_entries)
			clflush_cache_range(&l2_pt[l2_idx - l2_entries],
					    sizeof(l2_pt[0]) * l2_entries);
	}

	spin_unlock_irqrestore(&mmu_info->lock, flags);

	return 0;

error:
	/* unroll mapping in case something went wrong */
	if (size && mapped_size)
		l2_unmap(mmu_info, iova - mapped_size, paddr - mapped_size,
			 mapped_size);

	return err;
}

static int __ipu_mmu_map(struct ipu_mmu_info *mmu_info, unsigned long iova,
			 phys_addr_t paddr, size_t size)
{
	u32 iova_start = round_down(iova, ISP_PAGE_SIZE);
	u32 iova_end = ALIGN(iova + size, ISP_PAGE_SIZE);

	dev_dbg(mmu_info->dev,
		"mapping iova 0x%8.8x--0x%8.8x, size %zu at paddr 0x%10.10llx\n",
		iova_start, iova_end, size, paddr);

	return l2_map(mmu_info, iova_start, paddr, size);
}

static size_t l2_unmap(struct ipu_mmu_info *mmu_info, unsigned long iova,
		       phys_addr_t dummy, size_t size)
{
	u32 l1_idx;
	u32 *l2_pt;
	unsigned int l2_idx;
	unsigned int l2_entries;
	size_t unmapped = 0;
	unsigned long flags;

	spin_lock_irqsave(&mmu_info->lock, flags);
	for (l1_idx = iova >> ISP_L1PT_SHIFT;
	     size > 0 && l1_idx < ISP_L1PT_PTES; l1_idx++) {
		dev_dbg(mmu_info->dev,
			"unmapping l2 page table for l1 index %u (iova 0x%8.8lx)\n",
			l1_idx, iova);

		if (mmu_info->l1_pt[l1_idx] == mmu_info->dummy_l2_pteval) {
			spin_unlock_irqrestore(&mmu_info->lock, flags);
			dev_err(mmu_info->dev,
				"unmap iova 0x%8.8lx l1 idx %u which was not mapped\n",
				iova, l1_idx);
			return unmapped << ISP_PAGE_SHIFT;
		}
		l2_pt = mmu_info->l2_pts[l1_idx];

		l2_entries = 0;
		for (l2_idx = (iova & ISP_L2PT_MASK) >> ISP_L2PT_SHIFT;
		     size > 0 && l2_idx < ISP_L2PT_PTES; l2_idx++) {
			dev_dbg(mmu_info->dev,
				"unmap l2 index %u with pteval 0x%10.10llx\n",
				l2_idx, TBL_PHYS_ADDR(l2_pt[l2_idx]));
			l2_pt[l2_idx] = mmu_info->dummy_page_pteval;

			iova += ISP_PAGE_SIZE;
			size -= ISP_PAGE_SIZE;
			l2_entries++;
		}

		if (l2_entries) {
			clflush_cache_range(&l2_pt[l2_idx - l2_entries],
					    sizeof(l2_pt[0]) * l2_entries);
			unmapped += l2_entries;
		}
	}

	spin_unlock_irqrestore(&mmu_info->lock, flags);

	return unmapped << ISP_PAGE_SHIFT;
}

static size_t __ipu_mmu_unmap(struct ipu_mmu_info *mmu_info,
			      unsigned long iova, size_t size)
{
	return l2_unmap(mmu_info, iova, 0, size);
}

static int allocate_trash_buffer(struct ipu_mmu *mmu)
{
	unsigned int n_pages = PAGE_ALIGN(IPU_MMUV2_TRASH_RANGE) >> PAGE_SHIFT;
	struct iova *iova;
	u32 iova_addr;
	unsigned int i;
	dma_addr_t dma;
	int ret;

	/* Allocate 8MB in iova range */
	iova = alloc_iova(&mmu->dmap->iovad, n_pages,
			  mmu->dmap->mmu_info->aperture_end >> PAGE_SHIFT, 0);
	if (!iova) {
		dev_err(mmu->dev, "cannot allocate iova range for trash\n");
		return -ENOMEM;
	}

	dma = dma_map_page(mmu->dmap->mmu_info->dev, mmu->trash_page, 0,
			   PAGE_SIZE, DMA_BIDIRECTIONAL);
	if (dma_mapping_error(mmu->dmap->mmu_info->dev, dma)) {
		dev_err(mmu->dmap->mmu_info->dev, "Failed to map trash page\n");
		ret = -ENOMEM;
		goto out_free_iova;
	}

	mmu->pci_trash_page = dma;

	/*
	 * Map the 8MB iova address range to the same physical trash page
	 * mmu->trash_page which is already reserved at the probe
	 */
	iova_addr = iova->pfn_lo;
	for (i = 0; i < n_pages; i++) {
		ret = ipu_mmu_map(mmu->dmap->mmu_info, iova_addr << PAGE_SHIFT,
				  mmu->pci_trash_page, PAGE_SIZE);
		if (ret) {
			dev_err(mmu->dev,
				"mapping trash buffer range failed\n");
			goto out_unmap;
		}

		iova_addr++;
	}

	mmu->iova_trash_page = iova->pfn_lo << PAGE_SHIFT;
	dev_dbg(mmu->dev, "iova trash buffer for MMUID: %d is %u\n",
		mmu->mmid, (unsigned int)mmu->iova_trash_page);
	return 0;

out_unmap:
	ipu_mmu_unmap(mmu->dmap->mmu_info, iova->pfn_lo << PAGE_SHIFT,
		      (iova->pfn_hi - iova->pfn_lo + 1) << PAGE_SHIFT);
	dma_unmap_page(mmu->dmap->mmu_info->dev, mmu->pci_trash_page,
		       PAGE_SIZE, DMA_BIDIRECTIONAL);
out_free_iova:
	__free_iova(&mmu->dmap->iovad, iova);
	return ret;
}

static void __mmu_at_init(struct ipu_mmu *mmu)
{
	unsigned int i;
	struct ipu_mmu_info *mmu_info;

	mmu_info = mmu->dmap->mmu_info;
	for (i = 0; i < mmu->nr_mmus; i++) {
		struct ipu_mmu_hw *mmu_hw = &mmu->mmu_hw[i];
		unsigned int j;

		/* Write page table address per MMU */
		writel((phys_addr_t)mmu_info->l1_pt_dma,
		       mmu_hw->base + MMU_REG_PAGE_TABLE_BASE_ADDR);
		dev_dbg(mmu->dev, "mmu %s base was set as %x\n", mmu_hw->name,
			readl(mmu_hw->base + MMU_REG_PAGE_TABLE_BASE_ADDR));

		/* Set info bits and axi_refill per MMU */
		writel(mmu_hw->info_bits,
		       mmu_hw->base + MMU_REG_USER_INFO_BITS);
		writel(mmu_hw->refill, mmu_hw->base + MMU_REG_AXI_REFILL_IF_ID);
		writel(mmu_hw->collapse_en_bitmap,
		       mmu_hw->base + MMU_REG_COLLAPSE_ENABLE_BITMAP);

		if (mmu_hw->at_sp_arb_cfg)
			writel(mmu_hw->at_sp_arb_cfg,
			       mmu_hw->base + MMU_REG_AT_SP_ARB_CFG);

		/* default irq configuration */
		writel(0x3ff, mmu_hw->base + MMU_REG_IRQ_MASK);
		writel(0x3ff, mmu_hw->base + MMU_REG_IRQ_ENABLE);

		/* Configure MMU TLB stream configuration for L1/L2 */
		for (j = 0; j < mmu_hw->nr_l1streams; j++) {
			writel(mmu_hw->l1_block_sz[j], mmu_hw->base +
			       mmu_hw->l1_block + 4 * j);
		}

		for (j = 0; j < mmu_hw->nr_l2streams; j++) {
			writel(mmu_hw->l2_block_sz[j], mmu_hw->base +
			       mmu_hw->l2_block + 4 * j);
		}

		for (j = 0; j < mmu_hw->uao_p_num; j++) {
			if (!mmu_hw->uao_p2tlb[j])
				continue;
			writel(mmu_hw->uao_p2tlb[j], mmu_hw->uao_base + 4 * j);
		}
	}
}

static void __mmu_zlx_init(struct ipu_mmu *mmu)
{
	unsigned int i;

	dev_dbg(mmu->dev, "mmu zlx init\n");

	for (i = 0; i < mmu->nr_mmus; i++) {
		struct ipu_mmu_hw *mmu_hw = &mmu->mmu_hw[i];
		unsigned int j;

		dev_dbg(mmu->dev, "mmu %s zlx init\n", mmu_hw->name);
		for (j = 0; j < IPU_ZLX_POOL_NUM; j++) {
			if (!mmu_hw->zlx_axi_pool[j])
				continue;
			writel(mmu_hw->zlx_axi_pool[j],
			       mmu_hw->zlx_base + ZLX_REG_AXI_POOL + j * 0x4);
		}

		for (j = 0; j < mmu_hw->zlx_nr; j++) {
			if (!mmu_hw->zlx_conf[j])
				continue;

			writel(mmu_hw->zlx_conf[j],
			       mmu_hw->zlx_base + ZLX_REG_CONF + j * 0x8);
		}

		for (j = 0; j < mmu_hw->zlx_nr; j++) {
			if (!mmu_hw->zlx_en[j])
				continue;

			writel(mmu_hw->zlx_en[j],
			       mmu_hw->zlx_base + ZLX_REG_EN + j * 0x8);
		}
	}
}

int ipu_mmu_hw_init(struct ipu_mmu *mmu)
{
	unsigned long flags;
	struct ipu_mmu_info *mmu_info;

	dev_dbg(mmu->dev, "IPU mmu hardware init\n");

	mmu_info = mmu->dmap->mmu_info;

	/* Initialise the each MMU and ZLX */
	__mmu_at_init(mmu);
	__mmu_zlx_init(mmu);

	if (!mmu->trash_page) {
		int ret;

		mmu->trash_page = alloc_page(GFP_KERNEL);
		if (!mmu->trash_page) {
			dev_err(mmu->dev, "insufficient memory for trash buffer\n");
			return -ENOMEM;
		}

		ret = allocate_trash_buffer(mmu);
		if (ret) {
			__free_page(mmu->trash_page);
			mmu->trash_page = NULL;
			dev_err(mmu->dev, "trash buffer allocation failed\n");
			return ret;
		}
	}

	spin_lock_irqsave(&mmu->ready_lock, flags);
	mmu->ready = true;
	spin_unlock_irqrestore(&mmu->ready_lock, flags);

	return 0;
}
EXPORT_SYMBOL(ipu_mmu_hw_init);

static struct ipu_mmu_info *ipu_mmu_alloc(struct ipu_device *isp)
{
	struct ipu_mmu_info *mmu_info;
	int ret;

	mmu_info = kzalloc(sizeof(*mmu_info), GFP_KERNEL);
	if (!mmu_info)
		return NULL;

	mmu_info->aperture_start = IPU_FW_CODE_REGION_START;
	mmu_info->aperture_end = DMA_BIT_MASK(isp->secure_mode ?
					      IPU_MMU_ADDRESS_BITS :
					      IPU_MMU_ADDRESS_BITS_NON_SECURE);
	mmu_info->pgsize_bitmap = SZ_4K;
	mmu_info->dev = &isp->pdev->dev;

	ret = get_dummy_page(mmu_info);
	if (ret)
		goto err_free_info;

	ret = alloc_dummy_l2_pt(mmu_info);
	if (ret)
		goto err_free_dummy_page;

	mmu_info->l2_pts = vzalloc(ISP_L2PT_PTES * sizeof(*mmu_info->l2_pts));
	if (!mmu_info->l2_pts)
		goto err_free_dummy_l2_pt;

	/*
	 * We always map the L1 page table (a single page as well as
	 * the L2 page tables).
	 */
	mmu_info->l1_pt = alloc_l1_pt(mmu_info);
	if (!mmu_info->l1_pt)
		goto err_free_l2_pts;

	spin_lock_init(&mmu_info->lock);

	dev_dbg(mmu_info->dev, "domain initialised\n");

	return mmu_info;

err_free_l2_pts:
	vfree(mmu_info->l2_pts);
err_free_dummy_l2_pt:
	free_dummy_l2_pt(mmu_info);
err_free_dummy_page:
	free_dummy_page(mmu_info);
err_free_info:
	kfree(mmu_info);

	return NULL;
}

int ipu_mmu_hw_cleanup(struct ipu_mmu *mmu)
{
	unsigned long flags;

	spin_lock_irqsave(&mmu->ready_lock, flags);
	mmu->ready = false;
	spin_unlock_irqrestore(&mmu->ready_lock, flags);

	return 0;
}
EXPORT_SYMBOL(ipu_mmu_hw_cleanup);

static struct ipu_dma_mapping *alloc_dma_mapping(struct ipu_device *isp)
{
	struct ipu_dma_mapping *dmap;
	unsigned long base_pfn;

	dmap = kzalloc(sizeof(*dmap), GFP_KERNEL);
	if (!dmap)
		return NULL;

	dmap->mmu_info = ipu_mmu_alloc(isp);
	if (!dmap->mmu_info) {
		kfree(dmap);
		return NULL;
	}

	/* 0~64M is forbidden for uctile controller */
	base_pfn = max_t(unsigned long, 1,
			 PFN_DOWN(dmap->mmu_info->aperture_start));
	init_iova_domain(&dmap->iovad, SZ_4K, base_pfn);
	dmap->mmu_info->dmap = dmap;

	kref_init(&dmap->ref);

	dev_dbg(&isp->pdev->dev, "alloc mapping\n");

	iova_cache_get();

	return dmap;
}

phys_addr_t ipu_mmu_iova_to_phys(struct ipu_mmu_info *mmu_info,
				 dma_addr_t iova)
{
	unsigned long flags;
	u32 *l2_pt;
	phys_addr_t phy_addr;

	spin_lock_irqsave(&mmu_info->lock, flags);
	l2_pt = mmu_info->l2_pts[iova >> ISP_L1PT_SHIFT];
	phy_addr = (phys_addr_t)l2_pt[(iova & ISP_L2PT_MASK) >> ISP_L2PT_SHIFT];
	phy_addr <<= ISP_PAGE_SHIFT;
	spin_unlock_irqrestore(&mmu_info->lock, flags);

	return phy_addr;
}

/* drivers/iommu/iommu.c:iommu_unmap() */
size_t ipu_mmu_unmap(struct ipu_mmu_info *mmu_info, unsigned long iova,
		     size_t size)
{
	unsigned int min_pagesz;

	dev_dbg(mmu_info->dev, "unmapping iova 0x%lx size 0x%zx\n", iova, size);

	/* find out the minimum page size supported */
	min_pagesz = 1 << __ffs(mmu_info->pgsize_bitmap);

	/*
	 * The virtual address, as well as the size of the mapping, must be
	 * aligned (at least) to the size of the smallest page supported
	 * by the hardware
	 */
	if (!IS_ALIGNED(iova | size, min_pagesz)) {
		dev_err(NULL, "unaligned: iova 0x%lx size 0x%zx min_pagesz 0x%x\n",
			iova, size, min_pagesz);
		return -EINVAL;
	}

	return __ipu_mmu_unmap(mmu_info, iova, size);
}

/* drivers/iommu/iommu.c:iommu_map() */
int ipu_mmu_map(struct ipu_mmu_info *mmu_info, unsigned long iova,
		phys_addr_t paddr, size_t size)
{
	unsigned int min_pagesz;

	if (mmu_info->pgsize_bitmap == 0UL)
		return -ENODEV;

	/* find out the minimum page size supported */
	min_pagesz = 1 << __ffs(mmu_info->pgsize_bitmap);

	/*
	 * both the virtual address and the physical one, as well as
	 * the size of the mapping, must be aligned (at least) to the
	 * size of the smallest page supported by the hardware
	 */
	if (!IS_ALIGNED(iova | paddr | size, min_pagesz)) {
		dev_err(mmu_info->dev,
			"unaligned: iova 0x%lx pa %pa size 0x%zx min_pagesz 0x%x\n",
			iova, &paddr, size, min_pagesz);
		return -EINVAL;
	}

	dev_dbg(mmu_info->dev, "map: iova 0x%lx pa %pa size 0x%zx\n",
		iova, &paddr, size);

	return  __ipu_mmu_map(mmu_info, iova, paddr, size);
}

static void ipu_mmu_destroy(struct ipu_mmu *mmu)
{
	struct ipu_dma_mapping *dmap = mmu->dmap;
	struct ipu_mmu_info *mmu_info = dmap->mmu_info;
	struct iova *iova;
	u32 l1_idx;

	if (mmu->iova_trash_page) {
		iova = find_iova(&dmap->iovad,
				 mmu->iova_trash_page >> PAGE_SHIFT);
		if (iova) {
			/* unmap and free the trash buffer iova */
			ipu_mmu_unmap(mmu_info, iova->pfn_lo << PAGE_SHIFT,
				      (iova->pfn_hi - iova->pfn_lo + 1) <<
				      PAGE_SHIFT);
			__free_iova(&dmap->iovad, iova);
		} else {
			dev_err(mmu->dev, "trash buffer iova not found.\n");
		}

		mmu->iova_trash_page = 0;
		dma_unmap_page(mmu_info->dev, mmu->pci_trash_page,
			       PAGE_SIZE, DMA_BIDIRECTIONAL);
		mmu->pci_trash_page = 0;
		__free_page(mmu->trash_page);
	}

	for (l1_idx = 0; l1_idx < ISP_L1PT_PTES; l1_idx++) {
		if (mmu_info->l1_pt[l1_idx] != mmu_info->dummy_l2_pteval) {
			dma_unmap_single(mmu_info->dev,
					 TBL_PHYS_ADDR(mmu_info->l1_pt[l1_idx]),
					 PAGE_SIZE, DMA_BIDIRECTIONAL);
			free_page((unsigned long)mmu_info->l2_pts[l1_idx]);
		}
	}

	free_dummy_page(mmu_info);
	dma_unmap_single(mmu_info->dev, mmu_info->l1_pt_dma << ISP_PADDR_SHIFT,
			 PAGE_SIZE, DMA_BIDIRECTIONAL);
	free_page((unsigned long)mmu_info->dummy_l2_pt);
	free_page((unsigned long)mmu_info->l1_pt);
	kfree(mmu_info);
}

struct ipu_mmu *ipu_mmu_init(struct device *dev,
			     void __iomem *base, int mmid,
			     const struct ipu_hw_variants *hw)
{
	struct ipu_mmu *mmu;
	struct ipu_mmu_pdata *pdata;
	struct ipu_device *isp = pci_get_drvdata(to_pci_dev(dev));
	unsigned int i;

	if (hw->nr_mmus > IPU_MMU_MAX_NUM)
		return ERR_PTR(-EINVAL);

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	for (i = 0; i < hw->nr_mmus; i++) {
		struct ipu_mmu_hw *pdata_mmu = &pdata->mmu_hw[i];
		const struct ipu_mmu_hw *src_mmu = &hw->mmu_hw[i];

		if (src_mmu->nr_l1streams > IPU_MMU_MAX_TLB_L1_STREAMS ||
		    src_mmu->nr_l2streams > IPU_MMU_MAX_TLB_L2_STREAMS)
			return ERR_PTR(-EINVAL);

		*pdata_mmu = *src_mmu;
		pdata_mmu->base = base + src_mmu->offset;
		pdata_mmu->zlx_base = base + src_mmu->zlx_offset;
		pdata_mmu->uao_base = base + src_mmu->uao_offset;
	}

	mmu = devm_kzalloc(dev, sizeof(*mmu), GFP_KERNEL);
	if (!mmu)
		return ERR_PTR(-ENOMEM);

	mmu->mmid = mmid;
	mmu->mmu_hw = pdata->mmu_hw;
	mmu->nr_mmus = hw->nr_mmus;
	mmu->tlb_invalidate = tlb_invalidate;
	mmu->ready = false;
	INIT_LIST_HEAD(&mmu->vma_list);
	spin_lock_init(&mmu->ready_lock);

	mmu->dmap = alloc_dma_mapping(isp);
	if (!mmu->dmap) {
		dev_err(dev, "can't alloc dma mapping\n");
		return ERR_PTR(-ENOMEM);
	}

	return mmu;
}

void ipu_mmu_cleanup(struct ipu_mmu *mmu)
{
	struct ipu_dma_mapping *dmap = mmu->dmap;

	ipu_mmu_destroy(mmu);
	mmu->dmap = NULL;
	iova_cache_put();
	put_iova_domain(&dmap->iovad);
	kfree(dmap);
}

MODULE_AUTHOR("Bingbu Cao <bingbu.cao@intel.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Intel ipu mmu driver");
