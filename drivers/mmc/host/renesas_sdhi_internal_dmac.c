/*
 * linux/drivers/mmc/renesas_sdhi_internal_dmac.c
 *
 * Copyright (C) 2015 Renesas Electronics Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * R-Car Gen3 DMA function for TMIO MMC implementations
 */

#include <linux/bug.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/mfd/tmio.h>
#include <linux/mmc/host.h>
#include <linux/pagemap.h>
#include <linux/scatterlist.h>

#include "tmio_mmc.h"

#define DM_CM_DTRAN_MODE	0x820
#define DM_CM_DTRAN_CTRL	0x828
#define DM_CM_RST		0x830
#define DM_CM_INFO1		0x840
#define DM_CM_INFO1_MASK	0x848
#define DM_CM_INFO2		0x850
#define DM_CM_INFO2_MASK	0x858
#define DM_DTRAN_ADDR		0x880

/* DM_CM_DTRAN_MODE */
#define DTRAN_MODE_CH_NUM_CH0	0	/* "downstream" = for write commands */
#define DTRAN_MODE_CH_NUM_CH1	BIT(16)	/* "uptream" = for read commands */
#define DTRAN_MODE_BUS_WID_TH	(BIT(5) | BIT(4))
#define DTRAN_MODE_ADDR_MODE	BIT(0)	/* 1 = Increment address */

/* DM_CM_DTRAN_CTRL */
#define DTRAN_CTRL_DM_START	BIT(0)

/* DM_CM_RST */
#define RST_DTRANRST1		BIT(9)
#define RST_DTRANRST0		BIT(8)
#define RST_RESERVED_BITS	GENMASK_ULL(32, 0)

/* DM_CM_INFO1 and DM_CM_INFO1_MASK */
#define INFO1_CLEAR		0
#define INFO1_DTRANEND1		BIT(17)
#define INFO1_DTRANEND0		BIT(16)

/* DM_CM_INFO2 and DM_CM_INFO2_MASK */
#define INFO2_DTRANERR1		BIT(17)
#define INFO2_DTRANERR0		BIT(16)

/*
 * Specification of this driver:
 * - host->chan_{rx,tx} will be used as a flag of enabling/disabling the dma
 * - Since this SDHI DMAC register set has actual 32-bit and "bus_shift" is 2,
 *   this driver cannot use original sd_ctrl_{write,read}32 functions.
 */

static void tmio_dm_write(struct tmio_mmc_host *host, int addr, u64 val)
{
	writeq(val, host->ctl + addr);
}

static void sdhi_internal_dmac_enable_dma(struct tmio_mmc_host *host,
					  bool enable)
{
	if (!host->chan_tx || !host->chan_rx)
		return;

	if (!enable)
		tmio_dm_write(host, DM_CM_INFO1, INFO1_CLEAR);

	if (host->dma->enable)
		host->dma->enable(host, enable);
}

static void sdhi_internal_dmac_abort_dma(struct tmio_mmc_host *host)
{
	u64 val = RST_DTRANRST1 | RST_DTRANRST0;

	sdhi_internal_dmac_enable_dma(host, false);

	tmio_dm_write(host, DM_CM_RST, RST_RESERVED_BITS & ~val);
	tmio_dm_write(host, DM_CM_RST, RST_RESERVED_BITS | val);

	sdhi_internal_dmac_enable_dma(host, true);
}

static void sdhi_internal_dmac_start_dma(struct tmio_mmc_host *host,
			       struct mmc_data *data)
{
	struct scatterlist *sg = host->sg_ptr;
	u32 dtran_mode = DTRAN_MODE_BUS_WID_TH | DTRAN_MODE_ADDR_MODE;
	enum dma_data_direction dir;
	int ret;
	u32 irq_mask;

	/* This DMAC cannot handle if sg_len is not 1 */
	WARN_ON(host->sg_len > 1);

	/* This DMAC cannot handle if buffer is not 8-bytes alignment */
	if (!IS_ALIGNED(sg->offset, 8)) {
		host->force_pio = true;
		sdhi_internal_dmac_enable_dma(host, false);
		return;
	}

	if (data->flags & MMC_DATA_READ) {
		dtran_mode |= DTRAN_MODE_CH_NUM_CH1;
		dir = DMA_FROM_DEVICE;
		irq_mask = TMIO_STAT_RXRDY;
	} else {
		dtran_mode |= DTRAN_MODE_CH_NUM_CH0;
		dir = DMA_TO_DEVICE;
		irq_mask = TMIO_STAT_TXRQ;
	}

	ret = dma_map_sg(&host->pdev->dev, sg, host->sg_len, dir);
	if (ret < 0) {
		dev_err(&host->pdev->dev, "%s: dma_map_sg failed\n", __func__);
		return;
	}

	sdhi_internal_dmac_enable_dma(host, true);

	/* disable PIO irqs to avoid "PIO IRQ in DMA mode!" */
	tmio_mmc_disable_mmc_irqs(host, irq_mask);

	/* set dma parameters */
	tmio_dm_write(host, DM_CM_DTRAN_MODE, dtran_mode);
	tmio_dm_write(host, DM_DTRAN_ADDR, sg->dma_address);
}

static void sdhi_internal_dmac_issue_tasklet_fn(unsigned long arg)
{
	struct tmio_mmc_host *host = (struct tmio_mmc_host *)arg;

	tmio_mmc_enable_mmc_irqs(host, TMIO_STAT_DATAEND);

	/* start the DMAC */
	tmio_dm_write(host, DM_CM_DTRAN_CTRL, DTRAN_CTRL_DM_START);
}

static void sdhi_internal_dmac_complete_tasklet_fn(unsigned long arg)
{
	struct tmio_mmc_host *host = (struct tmio_mmc_host *)arg;
	enum dma_data_direction dir;

	if (!host->data)
		return;

	if (host->data->flags & MMC_DATA_READ)
		dir = DMA_FROM_DEVICE;
	else
		dir = DMA_TO_DEVICE;

	sdhi_internal_dmac_enable_dma(host, false);
	dma_unmap_sg(&host->pdev->dev, host->sg_ptr, host->sg_len, dir);
	tmio_mmc_do_data_irq(host);
}

static void sdhi_internal_dmac_request_dma(struct tmio_mmc_host *host,
					   struct tmio_mmc_data *pdata)
{
	/* Each value is set to non-zero to assume "enabling" each DMA */
	host->chan_rx = host->chan_tx = (void *)0xdeadbeaf;

	tasklet_init(&host->dma_complete,
		     sdhi_internal_dmac_complete_tasklet_fn,
		     (unsigned long)host);
	tasklet_init(&host->dma_issue, sdhi_internal_dmac_issue_tasklet_fn,
		     (unsigned long)host);
}

static void sdhi_internal_dmac_release_dma(struct tmio_mmc_host *host)
{
	/* Each value is set to zero to assume "disabling" each DMA */
	host->chan_rx = host->chan_tx = NULL;
}

static struct tmio_mmc_dma_ops sdhi_internal_dmac_dma_ops = {
	.start = sdhi_internal_dmac_start_dma,
	.enable = sdhi_internal_dmac_enable_dma,
	.request = sdhi_internal_dmac_request_dma,
	.release = sdhi_internal_dmac_release_dma,
	.abort = sdhi_internal_dmac_abort_dma,
};

void sdhi_internal_dmac_init_dma(void)
{
	tmio_set_dma_ops(&sdhi_internal_dmac_dma_ops);
}
