/*
 * IPMMU VMSA
 *
 * Copyright (C) 2014 Renesas Electronics Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 */

#include <linux/bitmap.h>
#include <linux/delay.h>
#include <linux/dma-iommu.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_iommu.h>
#include <linux/platform_device.h>
#include <linux/sizes.h>
#include <linux/slab.h>

#if defined(CONFIG_ARM) && !defined(CONFIG_IOMMU_DMA)
#include <asm/dma-iommu.h>
#include <asm/pgalloc.h>
#endif

#include "io-pgtable.h"

#define IPMMU_CTX_MAX 8

struct ipmmu_features {
	bool use_ns_alias_offset;
	bool has_cache_leaf_nodes;
	bool has_eight_ctx;
	bool setup_imbuscr;
	bool twobit_imttbcr_sl0;
};

struct ipmmu_vmsa_device {
	struct device *dev;
	void __iomem *base;
	struct list_head list;
	const struct ipmmu_features *features;
	bool is_leaf;
	unsigned int num_utlbs;
	unsigned int num_ctx;
	spinlock_t lock;			/* Protects ctx and domains[] */
	DECLARE_BITMAP(ctx, IPMMU_CTX_MAX);
	struct ipmmu_vmsa_domain *domains[IPMMU_CTX_MAX];

	struct dma_iommu_mapping *mapping;
};

struct ipmmu_vmsa_domain {
	struct ipmmu_vmsa_device *mmu;
	struct ipmmu_vmsa_device *root;
	struct iommu_domain io_domain;

	struct io_pgtable_cfg cfg;
	struct io_pgtable_ops *iop;

	unsigned int context_id;
	spinlock_t lock;			/* Protects mappings */
};

struct ipmmu_vmsa_archdata {
	struct ipmmu_vmsa_device *mmu;
	unsigned int *utlbs;
	unsigned int num_utlbs;
	struct device *dev;
	struct list_head list;
};

static DEFINE_SPINLOCK(ipmmu_devices_lock);
static LIST_HEAD(ipmmu_devices);

static DEFINE_SPINLOCK(ipmmu_slave_devices_lock);
static LIST_HEAD(ipmmu_slave_devices);

static struct ipmmu_vmsa_domain *to_vmsa_domain(struct iommu_domain *dom)
{
	return container_of(dom, struct ipmmu_vmsa_domain, io_domain);
}

#if defined(CONFIG_ARM) || defined(CONFIG_ARM64)
static struct ipmmu_vmsa_archdata *to_archdata(struct device *dev)
{
	return dev->archdata.iommu;
}
static void set_archdata(struct device *dev, struct ipmmu_vmsa_archdata *p)
{
	dev->archdata.iommu = p;
}
#else
static struct ipmmu_vmsa_archdata *to_archdata(struct device *dev)
{
	return NULL;
}
static void set_archdata(struct device *dev, struct ipmmu_vmsa_archdata *p)
{
}
#endif

#define TLB_LOOP_TIMEOUT		100	/* 100us */

/* -----------------------------------------------------------------------------
 * Registers Definition
 */

#define IM_NS_ALIAS_OFFSET		0x800

#define IM_CTX_SIZE			0x40

#define IMCTR				0x0000
#define IMCTR_TRE			(1 << 17)
#define IMCTR_AFE			(1 << 16)
#define IMCTR_RTSEL_MASK		(3 << 4)
#define IMCTR_RTSEL_SHIFT		4
#define IMCTR_TREN			(1 << 3)
#define IMCTR_INTEN			(1 << 2)
#define IMCTR_FLUSH			(1 << 1)
#define IMCTR_MMUEN			(1 << 0)

#define IMCAAR				0x0004

#define IMTTBCR				0x0008
#define IMTTBCR_EAE			(1 << 31)
#define IMTTBCR_PMB			(1 << 30)
#define IMTTBCR_SH1_NON_SHAREABLE	(0 << 28)
#define IMTTBCR_SH1_OUTER_SHAREABLE	(2 << 28)
#define IMTTBCR_SH1_INNER_SHAREABLE	(3 << 28)
#define IMTTBCR_SH1_MASK		(3 << 28)
#define IMTTBCR_ORGN1_NC		(0 << 26)
#define IMTTBCR_ORGN1_WB_WA		(1 << 26)
#define IMTTBCR_ORGN1_WT		(2 << 26)
#define IMTTBCR_ORGN1_WB		(3 << 26)
#define IMTTBCR_ORGN1_MASK		(3 << 26)
#define IMTTBCR_IRGN1_NC		(0 << 24)
#define IMTTBCR_IRGN1_WB_WA		(1 << 24)
#define IMTTBCR_IRGN1_WT		(2 << 24)
#define IMTTBCR_IRGN1_WB		(3 << 24)
#define IMTTBCR_IRGN1_MASK		(3 << 24)
#define IMTTBCR_TSZ1_MASK		(7 << 16)
#define IMTTBCR_TSZ1_SHIFT		16
#define IMTTBCR_SH0_NON_SHAREABLE	(0 << 12)
#define IMTTBCR_SH0_OUTER_SHAREABLE	(2 << 12)
#define IMTTBCR_SH0_INNER_SHAREABLE	(3 << 12)
#define IMTTBCR_SH0_MASK		(3 << 12)
#define IMTTBCR_ORGN0_NC		(0 << 10)
#define IMTTBCR_ORGN0_WB_WA		(1 << 10)
#define IMTTBCR_ORGN0_WT		(2 << 10)
#define IMTTBCR_ORGN0_WB		(3 << 10)
#define IMTTBCR_ORGN0_MASK		(3 << 10)
#define IMTTBCR_IRGN0_NC		(0 << 8)
#define IMTTBCR_IRGN0_WB_WA		(1 << 8)
#define IMTTBCR_IRGN0_WT		(2 << 8)
#define IMTTBCR_IRGN0_WB		(3 << 8)
#define IMTTBCR_IRGN0_MASK		(3 << 8)
#define IMTTBCR_SL0_LVL_2		(0 << 4)
#define IMTTBCR_SL0_LVL_1		(1 << 4)
#define IMTTBCR_TSZ0_MASK		(7 << 0)
#define IMTTBCR_TSZ0_SHIFT		O

#define IMTTBCR_SL0_TWOBIT_LVL_3	(0 << 6)
#define IMTTBCR_SL0_TWOBIT_LVL_2	(1 << 6)
#define IMTTBCR_SL0_TWOBIT_LVL_1	(2 << 6)

#define IMBUSCR				0x000c
#define IMBUSCR_DVM			(1 << 2)
#define IMBUSCR_BUSSEL_SYS		(0 << 0)
#define IMBUSCR_BUSSEL_CCI		(1 << 0)
#define IMBUSCR_BUSSEL_IMCAAR		(2 << 0)
#define IMBUSCR_BUSSEL_CCI_IMCAAR	(3 << 0)
#define IMBUSCR_BUSSEL_MASK		(3 << 0)

#define IMTTLBR0			0x0010
#define IMTTUBR0			0x0014
#define IMTTLBR1			0x0018
#define IMTTUBR1			0x001c

#define IMSTR				0x0020
#define IMSTR_ERRLVL_MASK		(3 << 12)
#define IMSTR_ERRLVL_SHIFT		12
#define IMSTR_ERRCODE_TLB_FORMAT	(1 << 8)
#define IMSTR_ERRCODE_ACCESS_PERM	(4 << 8)
#define IMSTR_ERRCODE_SECURE_ACCESS	(5 << 8)
#define IMSTR_ERRCODE_MASK		(7 << 8)
#define IMSTR_MHIT			(1 << 4)
#define IMSTR_ABORT			(1 << 2)
#define IMSTR_PF			(1 << 1)
#define IMSTR_TF			(1 << 0)

#define IMMAIR0				0x0028
#define IMMAIR1				0x002c
#define IMMAIR_ATTR_MASK		0xff
#define IMMAIR_ATTR_DEVICE		0x04
#define IMMAIR_ATTR_NC			0x44
#define IMMAIR_ATTR_WBRWA		0xff
#define IMMAIR_ATTR_SHIFT(n)		((n) << 3)
#define IMMAIR_ATTR_IDX_NC		0
#define IMMAIR_ATTR_IDX_WBRWA		1
#define IMMAIR_ATTR_IDX_DEV		2

#define IMEAR				0x0030

#define IMPCTR				0x0200
#define IMPSTR				0x0208
#define IMPEAR				0x020c
#define IMPMBA(n)			(0x0280 + ((n) * 4))
#define IMPMBD(n)			(0x02c0 + ((n) * 4))

#define IMUCTR(n)			(0x0300 + ((n) * 16))
#define IMUCTR_FIXADDEN			(1 << 31)
#define IMUCTR_FIXADD_MASK		(0xff << 16)
#define IMUCTR_FIXADD_SHIFT		16
#define IMUCTR_TTSEL_MMU(n)		((n) << 4)
#define IMUCTR_TTSEL_PMB		(8 << 4)
#define IMUCTR_TTSEL_MASK		(15 << 4)
#define IMUCTR_FLUSH			(1 << 1)
#define IMUCTR_MMUEN			(1 << 0)

#define IMUASID(n)			(0x0308 + ((n) * 16))
#define IMUASID_ASID8_MASK		(0xff << 8)
#define IMUASID_ASID8_SHIFT		8
#define IMUASID_ASID0_MASK		(0xff << 0)
#define IMUASID_ASID0_SHIFT		0

/* -----------------------------------------------------------------------------
 * Root device handling
 */

static bool ipmmu_is_root(struct ipmmu_vmsa_device *mmu)
{
	if (mmu->features->has_cache_leaf_nodes)
		return mmu->is_leaf ? false : true;
	else
		return true; /* older IPMMU hardware treated as single root */
}

static struct ipmmu_vmsa_device *ipmmu_find_root(struct ipmmu_vmsa_device *leaf)
{
	struct ipmmu_vmsa_device *mmu = NULL;

	if (ipmmu_is_root(leaf))
		return leaf;

	spin_lock(&ipmmu_devices_lock);

	list_for_each_entry(mmu, &ipmmu_devices, list) {
		if (ipmmu_is_root(mmu))
			break;
	}

	spin_unlock(&ipmmu_devices_lock);
	return mmu;
}

/* -----------------------------------------------------------------------------
 * Read/Write Access
 */

static u32 ipmmu_read(struct ipmmu_vmsa_device *mmu, unsigned int offset)
{
	return ioread32(mmu->base + offset);
}

static void ipmmu_write(struct ipmmu_vmsa_device *mmu, unsigned int offset,
			u32 data)
{
	iowrite32(data, mmu->base + offset);
}

static u32 ipmmu_ctx_read(struct ipmmu_vmsa_domain *domain, unsigned int reg)
{
	return ipmmu_read(domain->root, domain->context_id * IM_CTX_SIZE + reg);
}

static void ipmmu_ctx_write(struct ipmmu_vmsa_domain *domain, unsigned int reg,
			    u32 data)
{
	ipmmu_write(domain->root, domain->context_id * IM_CTX_SIZE + reg, data);
}

static void ipmmu_ctx_write2(struct ipmmu_vmsa_domain *domain, unsigned int reg,
			     u32 data)
{
	if (domain->mmu != domain->root)
		ipmmu_write(domain->mmu,
			    domain->context_id * IM_CTX_SIZE + reg, data);

	ipmmu_write(domain->root, domain->context_id * IM_CTX_SIZE + reg, data);
}

/* -----------------------------------------------------------------------------
 * TLB and microTLB Management
 */

/* Wait for any pending TLB invalidations to complete */
static void ipmmu_tlb_sync(struct ipmmu_vmsa_domain *domain)
{
	unsigned int count = 0;

	while (ipmmu_ctx_read(domain, IMCTR) & IMCTR_FLUSH) {
		cpu_relax();
		if (++count == TLB_LOOP_TIMEOUT) {
			dev_err_ratelimited(domain->mmu->dev,
			"TLB sync timed out -- MMU may be deadlocked\n");
			return;
		}
		udelay(1);
	}
}

static void ipmmu_tlb_invalidate(struct ipmmu_vmsa_domain *domain)
{
	u32 reg;

	reg = ipmmu_ctx_read(domain, IMCTR);
	reg |= IMCTR_FLUSH;
	ipmmu_ctx_write2(domain, IMCTR, reg);

	ipmmu_tlb_sync(domain);
}

/*
 * Enable MMU translation for the microTLB.
 */
static void ipmmu_utlb_enable(struct ipmmu_vmsa_domain *domain,
			      unsigned int utlb)
{
	struct ipmmu_vmsa_device *mmu = domain->mmu;

	/*
	 * TODO: Reference-count the microTLB as several bus masters can be
	 * connected to the same microTLB.
	 */

	/* TODO: What should we set the ASID to ? */
	ipmmu_write(mmu, IMUASID(utlb), 0);
	/* TODO: Do we need to flush the microTLB ? */
	ipmmu_write(mmu, IMUCTR(utlb),
		    IMUCTR_TTSEL_MMU(domain->context_id) | IMUCTR_FLUSH |
		    IMUCTR_MMUEN);
}

/*
 * Disable MMU translation for the microTLB.
 */
static void ipmmu_utlb_disable(struct ipmmu_vmsa_domain *domain,
			       unsigned int utlb)
{
	struct ipmmu_vmsa_device *mmu = domain->mmu;

	ipmmu_write(mmu, IMUCTR(utlb), 0);
}

static void ipmmu_tlb_flush_all(void *cookie)
{
	struct ipmmu_vmsa_domain *domain = cookie;

	ipmmu_tlb_invalidate(domain);
}

static void ipmmu_tlb_add_flush(unsigned long iova, size_t size,
				size_t granule, bool leaf, void *cookie)
{
	/* The hardware doesn't support selective TLB flush. */
}

static struct iommu_gather_ops ipmmu_gather_ops = {
	.tlb_flush_all = ipmmu_tlb_flush_all,
	.tlb_add_flush = ipmmu_tlb_add_flush,
	.tlb_sync = ipmmu_tlb_flush_all,
};

/* -----------------------------------------------------------------------------
 * Domain/Context Management
 */

static int ipmmu_domain_allocate_context(struct ipmmu_vmsa_device *mmu,
					 struct ipmmu_vmsa_domain *domain)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&mmu->lock, flags);

	ret = find_first_zero_bit(mmu->ctx, mmu->num_ctx);
	if (ret != mmu->num_ctx) {
		mmu->domains[ret] = domain;
		set_bit(ret, mmu->ctx);
	} else
		ret = -EBUSY;

	spin_unlock_irqrestore(&mmu->lock, flags);

	return ret;
}

static int ipmmu_domain_init_context(struct ipmmu_vmsa_domain *domain)
{
	u64 ttbr;
	u32 tmp;
	int ret;

	/*
	 * Allocate the page table operations.
	 *
	 * VMSA states in section B3.6.3 "Control of Secure or Non-secure memory
	 * access, Long-descriptor format" that the NStable bit being set in a
	 * table descriptor will result in the NStable and NS bits of all child
	 * entries being ignored and considered as being set. The IPMMU seems
	 * not to comply with this, as it generates a secure access page fault
	 * if any of the NStable and NS bits isn't set when running in
	 * non-secure mode.
	 */
	domain->cfg.quirks = IO_PGTABLE_QUIRK_ARM_NS;
	domain->cfg.pgsize_bitmap = SZ_1G | SZ_2M | SZ_4K,
	domain->cfg.ias = 32;
	domain->cfg.oas = 40;
	domain->cfg.tlb = &ipmmu_gather_ops;
	/*
	 * TODO: Add support for coherent walk through CCI with DVM and remove
	 * cache handling. For now, delegate it to the io-pgtable code.
	 */
	domain->cfg.iommu_dev = domain->root->dev;

	domain->iop = alloc_io_pgtable_ops(ARM_32_LPAE_S1, &domain->cfg,
					   domain);
	if (!domain->iop)
		return -EINVAL;

	/*
	 * Find an unused context.
	 */
	ret = ipmmu_domain_allocate_context(domain->root, domain);
	if (ret < 0) {
		free_io_pgtable_ops(domain->iop);
		return ret;
	}

	domain->context_id = ret;

	/* TTBR0 */
	ttbr = domain->cfg.arm_lpae_s1_cfg.ttbr[0];
	ipmmu_ctx_write(domain, IMTTLBR0, ttbr);
	ipmmu_ctx_write(domain, IMTTUBR0, ttbr >> 32);

	/*
	 * TTBCR
	 * We use long descriptors with inner-shareable WBWA tables and allocate
	 * the whole 32-bit VA space to TTBR0.
	 */

	if (domain->root->features->twobit_imttbcr_sl0)
		tmp = IMTTBCR_SL0_TWOBIT_LVL_1;
	else
		tmp = IMTTBCR_SL0_LVL_1;

	ipmmu_ctx_write(domain, IMTTBCR, IMTTBCR_EAE |
			IMTTBCR_SH0_INNER_SHAREABLE | IMTTBCR_ORGN0_WB_WA |
			IMTTBCR_IRGN0_WB_WA | tmp);

	/* MAIR0 */
	ipmmu_ctx_write(domain, IMMAIR0, domain->cfg.arm_lpae_s1_cfg.mair[0]);

	/* IMBUSCR */
	if (domain->root->features->setup_imbuscr)
		ipmmu_ctx_write(domain, IMBUSCR,
				ipmmu_ctx_read(domain, IMBUSCR) &
				~(IMBUSCR_DVM | IMBUSCR_BUSSEL_MASK));
	/*
	 * IMSTR
	 * Clear all interrupt flags.
	 */
	ipmmu_ctx_write(domain, IMSTR, ipmmu_ctx_read(domain, IMSTR));

	/*
	 * IMCTR
	 * Enable the MMU and interrupt generation. The long-descriptor
	 * translation table format doesn't use TEX remapping. Don't enable AF
	 * software management as we have no use for it. Flush the TLB as
	 * required when modifying the context registers.
	 */
	ipmmu_ctx_write2(domain, IMCTR,
			 IMCTR_INTEN | IMCTR_FLUSH | IMCTR_MMUEN);

	return 0;
}

static void ipmmu_domain_free_context(struct ipmmu_vmsa_device *mmu,
				      unsigned int context_id)
{
	unsigned long flags;

	spin_lock_irqsave(&mmu->lock, flags);

	clear_bit(context_id, mmu->ctx);
	mmu->domains[context_id] = NULL;

	spin_unlock_irqrestore(&mmu->lock, flags);
}

static void ipmmu_domain_destroy_context(struct ipmmu_vmsa_domain *domain)
{
	/*
	 * Disable the context. Flush the TLB as required when modifying the
	 * context registers.
	 *
	 * TODO: Is TLB flush really needed ?
	 */
	ipmmu_ctx_write2(domain, IMCTR, IMCTR_FLUSH);
	ipmmu_tlb_sync(domain);
	ipmmu_domain_free_context(domain->root, domain->context_id);
}

/* -----------------------------------------------------------------------------
 * Fault Handling
 */

static irqreturn_t ipmmu_domain_irq(struct ipmmu_vmsa_domain *domain)
{
	const u32 err_mask = IMSTR_MHIT | IMSTR_ABORT | IMSTR_PF | IMSTR_TF;
	struct ipmmu_vmsa_device *mmu = domain->mmu;
	u32 status;
	u32 iova;

	status = ipmmu_ctx_read(domain, IMSTR);
	if (!(status & err_mask))
		return IRQ_NONE;

	iova = ipmmu_ctx_read(domain, IMEAR);

	/*
	 * Clear the error status flags. Unlike traditional interrupt flag
	 * registers that must be cleared by writing 1, this status register
	 * seems to require 0. The error address register must be read before,
	 * otherwise its value will be 0.
	 */
	ipmmu_ctx_write(domain, IMSTR, 0);

	/* Log fatal errors. */
	if (status & IMSTR_MHIT)
		dev_err_ratelimited(mmu->dev, "Multiple TLB hits @0x%08x\n",
				    iova);
	if (status & IMSTR_ABORT)
		dev_err_ratelimited(mmu->dev, "Page Table Walk Abort @0x%08x\n",
				    iova);

	if (!(status & (IMSTR_PF | IMSTR_TF)))
		return IRQ_NONE;

	/*
	 * Try to handle page faults and translation faults.
	 *
	 * TODO: We need to look up the faulty device based on the I/O VA. Use
	 * the IOMMU device for now.
	 */
	if (!report_iommu_fault(&domain->io_domain, mmu->dev, iova, 0))
		return IRQ_HANDLED;

	dev_err_ratelimited(mmu->dev,
			    "Unhandled fault: status 0x%08x iova 0x%08x\n",
			    status, iova);

	return IRQ_HANDLED;
}

static irqreturn_t ipmmu_irq(int irq, void *dev)
{
	struct ipmmu_vmsa_device *mmu = dev;
	irqreturn_t status = IRQ_NONE;
	unsigned int i;
	unsigned long flags;

	spin_lock_irqsave(&mmu->lock, flags);

	/*
	 * Check interrupts for all active contexts.
	 */
	for (i = 0; i < mmu->num_ctx; i++) {
		if (!mmu->domains[i])
			continue;
		if (ipmmu_domain_irq(mmu->domains[i]) == IRQ_HANDLED)
			status = IRQ_HANDLED;
	}

	spin_unlock_irqrestore(&mmu->lock, flags);

	return status;
}

/* -----------------------------------------------------------------------------
 * IOMMU Operations
 */

static struct iommu_domain *__ipmmu_domain_alloc(unsigned type)
{
	struct ipmmu_vmsa_domain *domain;

	domain = kzalloc(sizeof(*domain), GFP_KERNEL);
	if (!domain)
		return NULL;

	spin_lock_init(&domain->lock);

	return &domain->io_domain;
}

static void ipmmu_domain_free(struct iommu_domain *io_domain)
{
	struct ipmmu_vmsa_domain *domain = to_vmsa_domain(io_domain);

	/*
	 * Free the domain resources. We assume that all devices have already
	 * been detached.
	 */
	ipmmu_domain_destroy_context(domain);
	free_io_pgtable_ops(domain->iop);
	kfree(domain);
}

static int ipmmu_attach_device(struct iommu_domain *io_domain,
			       struct device *dev)
{
	struct ipmmu_vmsa_archdata *archdata = to_archdata(dev);
	struct ipmmu_vmsa_device *root, *mmu = archdata->mmu;
	struct ipmmu_vmsa_domain *domain = to_vmsa_domain(io_domain);
	unsigned long flags;
	unsigned int i;
	int ret = 0;

	if (!mmu) {
		dev_err(dev, "Cannot attach to IPMMU\n");
		return -ENXIO;
	}

	root = ipmmu_find_root(archdata->mmu);
	if (!root) {
		dev_err(dev, "Unable to locate root IPMMU\n");
		return -EAGAIN;
	}

	spin_lock_irqsave(&domain->lock, flags);

	if (!domain->mmu) {
		/* The domain hasn't been used yet, initialize it. */
		domain->mmu = mmu;
		domain->root = root;
		ret = ipmmu_domain_init_context(domain);
		if (ret < 0) {
			dev_err(dev, "Unable to initialize IPMMU context\n");
			domain->mmu = NULL;
		} else {
			dev_info(dev, "Using IPMMU context %u\n",
				 domain->context_id);
		}
	} else if (domain->mmu != mmu) {
		/*
		 * Something is wrong, we can't attach two devices using
		 * different IOMMUs to the same domain.
		 */
		dev_err(dev, "Can't attach IPMMU %s to domain on IPMMU %s\n",
			dev_name(mmu->dev), dev_name(domain->mmu->dev));
		ret = -EINVAL;
	} else {
			dev_info(dev, "Reusing IPMMU context %u\n",
				 domain->context_id);
	}

	spin_unlock_irqrestore(&domain->lock, flags);

	if (ret < 0)
		return ret;

	for (i = 0; i < archdata->num_utlbs; ++i)
		ipmmu_utlb_enable(domain, archdata->utlbs[i]);

	return 0;
}

static void ipmmu_detach_device(struct iommu_domain *io_domain,
				struct device *dev)
{
	struct ipmmu_vmsa_archdata *archdata = to_archdata(dev);
	struct ipmmu_vmsa_domain *domain = to_vmsa_domain(io_domain);
	unsigned int i;

	for (i = 0; i < archdata->num_utlbs; ++i)
		ipmmu_utlb_disable(domain, archdata->utlbs[i]);

	/*
	 * TODO: Optimize by disabling the context when no device is attached.
	 */
}

static int ipmmu_map(struct iommu_domain *io_domain, unsigned long iova,
		     phys_addr_t paddr, size_t size, int prot)
{
	struct ipmmu_vmsa_domain *domain = to_vmsa_domain(io_domain);

	if (!domain)
		return -ENODEV;

	return domain->iop->map(domain->iop, iova, paddr, size, prot);
}

static size_t ipmmu_unmap(struct iommu_domain *io_domain, unsigned long iova,
			  size_t size)
{
	struct ipmmu_vmsa_domain *domain = to_vmsa_domain(io_domain);

	return domain->iop->unmap(domain->iop, iova, size);
}

static phys_addr_t ipmmu_iova_to_phys(struct iommu_domain *io_domain,
				      dma_addr_t iova)
{
	struct ipmmu_vmsa_domain *domain = to_vmsa_domain(io_domain);

	/* TODO: Is locking needed ? */

	return domain->iop->iova_to_phys(domain->iop, iova);
}

static struct device *ipmmu_find_sibling_device(struct device *dev)
{
	struct ipmmu_vmsa_archdata *archdata = dev->archdata.iommu;
	struct ipmmu_vmsa_archdata *sibling_archdata = NULL;
	bool found = false;

	spin_lock(&ipmmu_slave_devices_lock);

	list_for_each_entry(sibling_archdata, &ipmmu_slave_devices, list) {
		if (archdata == sibling_archdata)
			continue;
		if (sibling_archdata->mmu == archdata->mmu) {
			found = true;
			break;
		}
	}

	spin_unlock(&ipmmu_slave_devices_lock);

	return found ? sibling_archdata->dev : NULL;
}

static struct iommu_group *ipmmu_find_group(struct device *dev)
{
	struct iommu_group *group;
	struct device *sibling;

	sibling = ipmmu_find_sibling_device(dev);
	if (sibling)
		group = iommu_group_get(sibling);
	if (!sibling || IS_ERR(group))
		group = generic_device_group(dev);

	return group;
}

static int ipmmu_find_utlbs(struct ipmmu_vmsa_device *mmu, struct device *dev,
			    unsigned int *utlbs, unsigned int num_utlbs)
{
	unsigned int i;

	for (i = 0; i < num_utlbs; ++i) {
		struct of_phandle_args args;
		int ret;

		ret = of_parse_phandle_with_args(dev->of_node, "iommus",
						 "#iommu-cells", i, &args);
		if (ret < 0)
			return ret;

		of_node_put(args.np);

		if (args.np != mmu->dev->of_node || args.args_count != 1)
			return -EINVAL;

		utlbs[i] = args.args[0];
	}

	return 0;
}

static int ipmmu_init_platform_device(struct device *dev)
{
	struct ipmmu_vmsa_archdata *archdata;
	struct ipmmu_vmsa_device *mmu;
	unsigned int *utlbs;
	unsigned int i;
	int num_utlbs;
	int ret = -ENODEV;

	/* Find the master corresponding to the device. */

	num_utlbs = of_count_phandle_with_args(dev->of_node, "iommus",
					       "#iommu-cells");
	if (num_utlbs < 0)
		return -ENODEV;

	utlbs = kcalloc(num_utlbs, sizeof(*utlbs), GFP_KERNEL);
	if (!utlbs)
		return -ENOMEM;

	spin_lock(&ipmmu_devices_lock);

	list_for_each_entry(mmu, &ipmmu_devices, list) {
		ret = ipmmu_find_utlbs(mmu, dev, utlbs, num_utlbs);
		if (!ret) {
			/*
			 * TODO Take a reference to the MMU to protect
			 * against device removal.
			 */
			break;
		}
	}

	spin_unlock(&ipmmu_devices_lock);

	if (ret < 0)
		goto error;

	for (i = 0; i < num_utlbs; ++i) {
		if (utlbs[i] >= mmu->num_utlbs) {
			ret = -EINVAL;
			goto error;
		}
	}

	archdata = kzalloc(sizeof(*archdata), GFP_KERNEL);
	if (!archdata) {
		ret = -ENOMEM;
		goto error;
	}

	archdata->mmu = mmu;
	archdata->utlbs = utlbs;
	archdata->num_utlbs = num_utlbs;
	archdata->dev = dev;
	set_archdata(dev, archdata);
	return 0;

error:
	kfree(utlbs);
	return ret;
}

#if defined(CONFIG_ARM) && !defined(CONFIG_IOMMU_DMA)

static int ipmmu_add_device(struct device *dev)
{
	struct ipmmu_vmsa_device *mmu = NULL;
	struct iommu_group *group;
	int ret;

	if (to_archdata(dev)) {
		dev_warn(dev, "IOMMU driver already assigned to device %s\n",
			 dev_name(dev));
		return -EINVAL;
	}

	/* Create a device group and add the device to it. */
	group = iommu_group_alloc();
	if (IS_ERR(group)) {
		dev_err(dev, "Failed to allocate IOMMU group\n");
		ret = PTR_ERR(group);
		goto error;
	}

	ret = iommu_group_add_device(group, dev);
	iommu_group_put(group);

	if (ret < 0) {
		dev_err(dev, "Failed to add device to IPMMU group\n");
		group = NULL;
		goto error;
	}

	ret = ipmmu_init_platform_device(dev);
	if (ret < 0)
		goto error;

	/*
	 * Create the ARM mapping, used by the ARM DMA mapping core to allocate
	 * VAs. This will allocate a corresponding IOMMU domain.
	 *
	 * TODO:
	 * - Create one mapping per context (TLB).
	 * - Make the mapping size configurable ? We currently use a 2GB mapping
	 *   at a 1GB offset to ensure that NULL VAs will fault.
	 */
	mmu = to_archdata(dev)->mmu;
	if (!mmu->mapping) {
		struct dma_iommu_mapping *mapping;

		mapping = arm_iommu_create_mapping(&platform_bus_type,
						   SZ_1G, SZ_2G);
		if (IS_ERR(mapping)) {
			dev_err(mmu->dev, "failed to create ARM IOMMU mapping\n");
			ret = PTR_ERR(mapping);
			goto error;
		}

		mmu->mapping = mapping;
	}

	/* Attach the ARM VA mapping to the device. */
	ret = arm_iommu_attach_device(dev, mmu->mapping);
	if (ret < 0) {
		dev_err(dev, "Failed to attach device to VA mapping\n");
		goto error;
	}

	return 0;

error:
	if (mmu)
		arm_iommu_release_mapping(mmu->mapping);

	set_archdata(dev, NULL);

	if (!IS_ERR_OR_NULL(group))
		iommu_group_remove_device(dev);

	return ret;
}

static void ipmmu_remove_device(struct device *dev)
{
	struct ipmmu_vmsa_archdata *archdata = to_archdata(dev);

	arm_iommu_detach_device(dev);
	iommu_group_remove_device(dev);

	kfree(archdata->utlbs);
	kfree(archdata);

	set_archdata(dev, NULL);
}

static struct iommu_domain *ipmmu_domain_alloc(unsigned type)
{
	if (type != IOMMU_DOMAIN_UNMANAGED)
		return NULL;

	return __ipmmu_domain_alloc(type);
}

static const struct iommu_ops ipmmu_ops = {
	.domain_alloc = ipmmu_domain_alloc,
	.domain_free = ipmmu_domain_free,
	.attach_dev = ipmmu_attach_device,
	.detach_dev = ipmmu_detach_device,
	.map = ipmmu_map,
	.unmap = ipmmu_unmap,
	.map_sg = default_iommu_map_sg,
	.iova_to_phys = ipmmu_iova_to_phys,
	.add_device = ipmmu_add_device,
	.remove_device = ipmmu_remove_device,
	.pgsize_bitmap = SZ_1G | SZ_2M | SZ_4K,
};

#endif /* !CONFIG_ARM && CONFIG_IOMMU_DMA */

#ifdef CONFIG_IOMMU_DMA

static struct iommu_domain *ipmmu_domain_alloc_dma(unsigned type)
{
	struct iommu_domain *io_domain;

	if (type != IOMMU_DOMAIN_DMA)
		return NULL;

	io_domain = __ipmmu_domain_alloc(type);
	if (io_domain)
		iommu_get_dma_cookie(io_domain);

	return io_domain;
}

static void ipmmu_domain_free_dma(struct iommu_domain *io_domain)
{
	iommu_put_dma_cookie(io_domain);
	ipmmu_domain_free(io_domain);
}

static int ipmmu_add_device_dma(struct device *dev)
{
	struct ipmmu_vmsa_archdata *archdata = dev->archdata.iommu;
	struct iommu_group *group;

	/* only accept devices with iommus property */
	if (of_count_phandle_with_args(dev->of_node, "iommus",
				       "#iommu-cells") < 0)
		return -ENODEV;

	group = iommu_group_get_for_dev(dev);
	if (IS_ERR(group))
		return PTR_ERR(group);

	archdata = dev->archdata.iommu;
	spin_lock(&ipmmu_slave_devices_lock);
	list_add(&archdata->list, &ipmmu_slave_devices);
	spin_unlock(&ipmmu_slave_devices_lock);
	return 0;
}

static void ipmmu_remove_device_dma(struct device *dev)
{
	struct ipmmu_vmsa_archdata *archdata = dev->archdata.iommu;

	spin_lock(&ipmmu_slave_devices_lock);
	list_del(&archdata->list);
	spin_unlock(&ipmmu_slave_devices_lock);

	iommu_group_remove_device(dev);
}

static struct iommu_group *ipmmu_device_group_dma(struct device *dev)
{
	struct iommu_group *group;
	int ret;

	ret = ipmmu_init_platform_device(dev);
	if (!ret)
		group = ipmmu_find_group(dev);
	else
		group = ERR_PTR(ret);

	return group;
}

static int ipmmu_of_xlate_dma(struct device *dev,
			      struct of_phandle_args *spec)
{
	/* If the IPMMU device is disabled in DT then return error
	 * to make sure the of_iommu code does not install ops
	 * even though the iommu device is disabled
	 */
	if (!of_device_is_available(spec->np))
		return -ENODEV;

	return 0;
}

static const struct iommu_ops ipmmu_ops = {
	.domain_alloc = ipmmu_domain_alloc_dma,
	.domain_free = ipmmu_domain_free_dma,
	.attach_dev = ipmmu_attach_device,
	.detach_dev = ipmmu_detach_device,
	.map = ipmmu_map,
	.unmap = ipmmu_unmap,
	.map_sg = default_iommu_map_sg,
	.iova_to_phys = ipmmu_iova_to_phys,
	.add_device = ipmmu_add_device_dma,
	.remove_device = ipmmu_remove_device_dma,
	.device_group = ipmmu_device_group_dma,
	.pgsize_bitmap = SZ_1G | SZ_2M | SZ_4K,
	.of_xlate = ipmmu_of_xlate_dma,
};

#endif /* CONFIG_IOMMU_DMA */

/* -----------------------------------------------------------------------------
 * Probe/remove and init
 */

static void ipmmu_device_reset(struct ipmmu_vmsa_device *mmu)
{
	unsigned int i;

	/* Disable all contexts. */
	for (i = 0; i < mmu->num_ctx; ++i)
		ipmmu_write(mmu, i * IM_CTX_SIZE + IMCTR, 0);
}

static const struct ipmmu_features ipmmu_features_default = {
	.use_ns_alias_offset = true,
	.has_cache_leaf_nodes = false,
	.has_eight_ctx = false,
	.setup_imbuscr = true,
	.twobit_imttbcr_sl0 = false,
};

static const struct ipmmu_features ipmmu_features_rcar_gen3 = {
	.use_ns_alias_offset = false,
	.has_cache_leaf_nodes = true,
	.has_eight_ctx = true,
	.setup_imbuscr = false,
	.twobit_imttbcr_sl0 = true,
};

static const struct of_device_id ipmmu_of_ids[] = {
	{
		.compatible = "renesas,ipmmu-vmsa",
		.data = &ipmmu_features_default,
	}, {
		.compatible = "renesas,ipmmu-r8a7795",
		.data = &ipmmu_features_rcar_gen3,
	}, {
		.compatible = "renesas,ipmmu-r8a7796",
		.data = &ipmmu_features_rcar_gen3,
	}, {
		/* Terminator */
	},
};

MODULE_DEVICE_TABLE(of, ipmmu_of_ids);

static int ipmmu_probe(struct platform_device *pdev)
{
	struct ipmmu_vmsa_device *mmu;
	const struct of_device_id *match;
	struct resource *res;
	int irq;
	int ret;

	match = of_match_node(ipmmu_of_ids, pdev->dev.of_node);
	if (!match)
		return -EINVAL;

	mmu = devm_kzalloc(&pdev->dev, sizeof(*mmu), GFP_KERNEL);
	if (!mmu) {
		dev_err(&pdev->dev, "cannot allocate device data\n");
		return -ENOMEM;
	}

	mmu->dev = &pdev->dev;
	mmu->num_utlbs = 48;
	spin_lock_init(&mmu->lock);
	bitmap_zero(mmu->ctx, IPMMU_CTX_MAX);
	mmu->features = match->data;
	dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));

	/* Map I/O memory and request IRQ. */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	mmu->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(mmu->base))
		return PTR_ERR(mmu->base);

	/*
	 * The IPMMU has two register banks, for secure and non-secure modes.
	 * The bank mapped at the beginning of the IPMMU address space
	 * corresponds to the running mode of the CPU. When running in secure
	 * mode the non-secure register bank is also available at an offset.
	 *
	 * Secure mode operation isn't clearly documented and is thus currently
	 * not implemented in the driver. Furthermore, preliminary tests of
	 * non-secure operation with the main register bank were not successful.
	 * Offset the registers base unconditionally to point to the non-secure
	 * alias space for now.
	 */
	if (mmu->features->use_ns_alias_offset)
		mmu->base += IM_NS_ALIAS_OFFSET;

	/*
	 * The number of contexts varies with generation and instance.
	 * Newer SoCs get a total of 8 contexts enabled, older ones just one.
	 */
	if (mmu->features->has_eight_ctx)
		mmu->num_ctx = 8;
	else
		mmu->num_ctx = 1;

	WARN_ON(mmu->num_ctx > IPMMU_CTX_MAX);

	irq = platform_get_irq(pdev, 0);

	/*
	 * Determine if this IPMMU instance is a leaf device by checking
	 * if the renesas,ipmmu-main property exists or not.
	 */
	if (mmu->features->has_cache_leaf_nodes &&
	    of_find_property(pdev->dev.of_node, "renesas,ipmmu-main", NULL))
		mmu->is_leaf = true;

	/* Root devices have mandatory IRQs */
	if (ipmmu_is_root(mmu)) {
		if (irq < 0) {
			dev_err(&pdev->dev, "no IRQ found\n");
			return irq;
		}

		ret = devm_request_irq(&pdev->dev, irq, ipmmu_irq, 0,
				       dev_name(&pdev->dev), mmu);
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to request IRQ %d\n", irq);
			return ret;
		}

		ipmmu_device_reset(mmu);
	}

	/*
	 * We can't create the ARM mapping here as it requires the bus to have
	 * an IOMMU, which only happens when bus_set_iommu() is called in
	 * ipmmu_init() after the probe function returns.
	 */

	spin_lock(&ipmmu_devices_lock);
	list_add(&mmu->list, &ipmmu_devices);
	spin_unlock(&ipmmu_devices_lock);

	platform_set_drvdata(pdev, mmu);

	return 0;
}

static int ipmmu_remove(struct platform_device *pdev)
{
	struct ipmmu_vmsa_device *mmu = platform_get_drvdata(pdev);

	spin_lock(&ipmmu_devices_lock);
	list_del(&mmu->list);
	spin_unlock(&ipmmu_devices_lock);

#if defined(CONFIG_ARM) && !defined(CONFIG_IOMMU_DMA)
	arm_iommu_release_mapping(mmu->mapping);
#endif

	ipmmu_device_reset(mmu);

	return 0;
}

static struct platform_driver ipmmu_driver = {
	.driver = {
		.name = "ipmmu-vmsa",
		.of_match_table = of_match_ptr(ipmmu_of_ids),
	},
	.probe = ipmmu_probe,
	.remove	= ipmmu_remove,
};

static int __init ipmmu_init(void)
{
	static bool setup_done;
	int ret;

	if (setup_done)
		return 0;

	ret = platform_driver_register(&ipmmu_driver);
	if (ret < 0)
		return ret;

#if defined(CONFIG_ARM) && !defined(CONFIG_IOMMU_DMA)
	if (!iommu_present(&platform_bus_type))
		bus_set_iommu(&platform_bus_type, &ipmmu_ops);
#endif

	setup_done = true;
	return 0;
}

static void __exit ipmmu_exit(void)
{
	return platform_driver_unregister(&ipmmu_driver);
}

subsys_initcall(ipmmu_init);
module_exit(ipmmu_exit);

#ifdef CONFIG_IOMMU_DMA
static int __init ipmmu_vmsa_iommu_of_setup(struct device_node *np)
{
	static const struct iommu_ops *ops = &ipmmu_ops;

	ipmmu_init();

	of_iommu_set_ops(np, (struct iommu_ops *)ops);
	if (!iommu_present(&platform_bus_type))
		bus_set_iommu(&platform_bus_type, ops);

	return 0;
}

IOMMU_OF_DECLARE(ipmmu_vmsa_iommu_of, "renesas,ipmmu-vmsa",
		 ipmmu_vmsa_iommu_of_setup);
IOMMU_OF_DECLARE(ipmmu_r8a7795_iommu_of, "renesas,ipmmu-r8a7795",
		 ipmmu_vmsa_iommu_of_setup);
IOMMU_OF_DECLARE(ipmmu_r8a7796_iommu_of, "renesas,ipmmu-r8a7796",
		 ipmmu_vmsa_iommu_of_setup);
#endif

MODULE_DESCRIPTION("IOMMU API for Renesas VMSA-compatible IPMMU");
MODULE_AUTHOR("Laurent Pinchart <laurent.pinchart@ideasonboard.com>");
MODULE_LICENSE("GPL v2");
