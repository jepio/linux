// SPDX-License-Identifier: GPL-2.0-only
/*
 * AMD Secure Processor device driver
 *
 * Copyright (C) 2014,2018 Advanced Micro Devices, Inc.
 *
 * Author: Tom Lendacky <thomas.lendacky@amd.com>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/ioport.h>
#include <linux/dma-mapping.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/ccp.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/acpi.h>
#include <linux/iopoll.h>

#include "ccp-dev.h"
#include "psp-dev.h"
#include "sev-dev.h"

struct sp_platform {
	int coherent;
	unsigned int irq_count;
	bool is_vpsp;
	struct sp_device *sp;
};

static int vpsp_wait_event(struct sev_device *sev, unsigned int timeout)
{
	struct psp_device *psp = sev->psp;
	unsigned int status;
	void *__iomem reg = psp->io_regs + psp->vdata->intsts_reg;
	int err;
	err = readl_poll_timeout_atomic(reg, status, status & SEV_CMD_COMPLETE, 2, 100000);
	if (!err)
		writel(status, reg);
	return status;
}

static struct sp_device *sp_dev_master;

static const struct sp_dev_vdata dev_vdata[] = {
	{
		.bar = 0,
#ifdef CONFIG_CRYPTO_DEV_SP_CCP
		.ccp_vdata = &ccpv3_platform,
#endif
	},
};

#ifdef CONFIG_ACPI
static const struct acpi_device_id sp_acpi_match[] = {
	{ "AMDI0C00", (kernel_ulong_t)&dev_vdata[0] },
	{ },
};
MODULE_DEVICE_TABLE(acpi, sp_acpi_match);
#endif

#ifdef CONFIG_OF
static const struct of_device_id sp_of_match[] = {
	{ .compatible = "amd,ccp-seattle-v1a",
	  .data = (const void *)&dev_vdata[0] },
	{ },
};
MODULE_DEVICE_TABLE(of, sp_of_match);
#endif

#ifdef CONFIG_CRYPTO_DEV_SP_PSP
static struct sev_vdata vsevv1 = {
	.cmdresp_reg         = -1,
	.cmdbuff_addr_lo_reg = -1,
	.cmdbuff_addr_hi_reg = -1,
};
static struct psp_vdata vpspv1 = {
	.sev         = &vsevv1,
	.feature_reg = -1,
	.inten_reg   = -1,
	.intsts_reg  = -1,
};
#endif
static const struct sp_dev_vdata vpsp_vdata[] = {
	{
		.bar = 0,
#ifdef CONFIG_CRYPTO_DEV_SP_PSP
		.psp_vdata = &vpspv1,
#endif
	},

};
static const struct platform_device_id sp_plat_match[] = {
	{ "hv-vpsp", (kernel_ulong_t)&vpsp_vdata[0] },
	{ },
};
MODULE_DEVICE_TABLE(platform, sp_plat_match);

static struct resource vpsp_resources[] = {
	{},
};

#ifdef CONFIG_ACPI
static int vsps_parse_aspt(struct device *dev)
{
	struct acpi_aspt_acpi_mbox_regs acpiregs = {};
	struct acpi_aspt_sev_mbox_regs sevregs = {};
	struct acpi_aspt_global_regs gregs = {};
	struct acpi_aspt_header *entry, *end;
	struct acpi_table_aspt *aspt;
	acpi_status status;
	unsigned long long base;
	int err = 0;

	status = acpi_get_table(ACPI_SIG_ASPT, 0, (struct acpi_table_header **)&aspt);
	if (ACPI_FAILURE(status)) {
		const char *msg = acpi_format_exception(status);
		dev_err(dev, "failed to get ASPT table: %s\n", msg);
		return -ENODEV;
	}
	if (aspt->header.revision != ASPT_REVISION_ID) {
		dev_err(dev, "wrong ASPT revision: %d\n", (int)aspt->header.revision);
		err = -ENODEV;
		goto exit;
	}
	entry = (struct acpi_aspt_header *)(aspt + 1);
	end = (struct acpi_aspt_header *)((void *)aspt + aspt->header.length);
	while (entry < end) {
		if (((void *)entry + entry->length) > (void *)end) {
			dev_err(dev, "error parsing ASPT\n");
			err = -ENODEV;
			goto exit;
		}
		switch (entry->type) {
		case ACPI_ASPT_TYPE_GLOBAL_REGS:
			memcpy(&gregs, entry, entry->length);
			break;
		case ACPI_ASPT_TYPE_SEV_MBOX_REGS:
			memcpy(&sevregs, entry, entry->length);
			break;
		case ACPI_ASPT_TYPE_ACPI_MBOX_REGS:
			memcpy(&acpiregs, entry, entry->length);
			break;
		}
		entry = (struct acpi_aspt_header *)((void *)entry + entry->length);
	}
exit:
	if (!gregs.header.length || !sevregs.header.length || !acpiregs.header.length) {
		dev_err(dev, "missing ASPT table entry: %d %d %d\n", (int)gregs.header.length, (int)sevregs.header.length, (int)acpiregs.header.length);
		err = -ENODEV;
		goto exit;
	}
	acpi_put_table((struct acpi_table_header *)aspt);
	base = ALIGN_DOWN(gregs.feature_reg_addr, PAGE_SIZE);

	vpsp_resources[0] = ((struct resource)DEFINE_RES_MEM(base, PAGE_SIZE));
	{
		const struct sev_vdata tmp_vsevv1 = {
			.cmdresp_reg = sevregs.cmd_resp_reg_addr - base,
			.cmdbuff_addr_lo_reg = sevregs.cmd_buf_lo_reg_addr - base,
			.cmdbuff_addr_hi_reg = sevregs.cmd_buf_hi_reg_addr - base,
		};
		const struct psp_vdata tmp_vpspv1 = {
			.sev = &vsevv1,
			.feature_reg = gregs.feature_reg_addr - base,
			.inten_reg = gregs.irq_en_reg_addr - base,
			.intsts_reg = gregs.irq_st_reg_addr - base,
		};
		memcpy(&vsevv1, &tmp_vsevv1, sizeof(struct sev_vdata));
		memcpy(&vpspv1, &tmp_vpspv1, sizeof(struct psp_vdata));
	}
	dev_info(dev, "%pR\n", &vpsp_resources[0]);
	dev_info(dev, "GLBL feature_reg_addr:\t%x\n", vpspv1.feature_reg);
	dev_info(dev, "GLBL irq_en_reg_addr:\t%x\n", vpspv1.inten_reg);
	dev_info(dev, "GLBL irq_st_reg_addr:\t%x\n", vpspv1.intsts_reg);
	dev_info(dev, "SEV  cmd_resp_reg_addr:\t%x\n", vsevv1.cmdresp_reg);
	dev_info(dev, "SEV  cmd_buf_lo_reg_addr:\t%x\n", vsevv1.cmdbuff_addr_lo_reg);
	dev_info(dev, "SEV  cmd_buf_hi_reg_addr:\t%x\n", vsevv1.cmdbuff_addr_hi_reg);
	return err;
}
#else
static int vpsp_parse_aspt(struct device *dev)
{
	return -ENODEV;
}
#endif
static void psp_set_master(struct sp_device *sp)
{
	if (!sp_dev_master) {
		sp_dev_master = sp;
		return;
	}
}

static struct sp_device *psp_get_master(void)
{
	return sp_dev_master;
}

static void psp_clear_master(struct sp_device *sp)
{
	if (sp == sp_dev_master) {
		sp_dev_master = NULL;
		dev_dbg(sp->dev, "Cleared sp_dev_master\n");
	}
}

static struct sp_dev_vdata *sp_get_of_version(struct platform_device *pdev)
{
#ifdef CONFIG_OF
	const struct of_device_id *match;

	match = of_match_node(sp_of_match, pdev->dev.of_node);
	if (match && match->data)
		return (struct sp_dev_vdata *)match->data;
#endif
	return NULL;
}

static struct sp_dev_vdata *sp_get_acpi_version(struct platform_device *pdev)
{
#ifdef CONFIG_ACPI
	const struct acpi_device_id *match;

	match = acpi_match_device(sp_acpi_match, &pdev->dev);
	if (match && match->driver_data)
		return (struct sp_dev_vdata *)match->driver_data;
#endif
	return NULL;
}

static int sp_get_irqs(struct sp_device *sp)
{
	struct sp_platform *sp_platform = sp->dev_specific;
	struct device *dev = sp->dev;
	struct platform_device *pdev = to_platform_device(dev);
	int ret;
	if (sp_platform->is_vpsp)
		return 0;

	sp_platform->irq_count = platform_irq_count(pdev);

	ret = platform_get_irq(pdev, 0);
	if (ret < 0) {
		dev_notice(dev, "unable to get IRQ (%d)\n", ret);
		return ret;
	}

	sp->psp_irq = ret;
	if (sp_platform->irq_count == 1) {
		sp->ccp_irq = ret;
	} else {
		ret = platform_get_irq(pdev, 1);
		if (ret < 0) {
			dev_notice(dev, "unable to get IRQ (%d)\n", ret);
			return ret;
		}

		sp->ccp_irq = ret;
	}

	return 0;
}

static int sp_platform_probe(struct platform_device *pdev)
{
	struct sp_device *sp;
	struct sp_platform *sp_platform;
	struct device *dev = &pdev->dev;
	enum dev_dma_attr attr;
	int ret;

	ret = -ENOMEM;
	sp = sp_alloc_struct(dev);
	if (!sp)
		goto e_err;

	sp_platform = devm_kzalloc(dev, sizeof(*sp_platform), GFP_KERNEL);
	if (!sp_platform)
		goto e_err;

	sp->dev_specific = sp_platform;
	sp->dev_vdata = pdev->dev.of_node ? sp_get_of_version(pdev)
					 : sp_get_acpi_version(pdev);
	if (!sp->dev_vdata && pdev->id_entry) {
		sp->dev_vdata = (struct sp_dev_vdata *)pdev->id_entry->driver_data;
		if (sp->dev_vdata == &vpsp_vdata[0]) {
			ret = vsps_parse_aspt(dev);
			if (ret)
				goto e_err;
			sp_platform->is_vpsp = true;
		}

	}
	if (!sp->dev_vdata) {
		ret = -ENODEV;
		dev_err(dev, "missing driver data\n");
		goto e_err;
	}

	if (sp_platform->is_vpsp) {
		sp->io_map = devm_ioremap_resource(dev, vpsp_resources);
	} else {
		sp->io_map = devm_platform_ioremap_resource(pdev, 0);
	}
	if (IS_ERR(sp->io_map)) {
		ret = PTR_ERR(sp->io_map);
		goto e_err;
	}

	attr = sp_platform->is_vpsp ? DEV_DMA_COHERENT : device_get_dma_attr(dev);
	if (attr == DEV_DMA_NOT_SUPPORTED) {
		dev_err(dev, "DMA is not supported");
		ret = -EINVAL;
		goto e_err;
	}

	sp_platform->coherent = (attr == DEV_DMA_COHERENT);
	if (sp_platform->coherent)
		sp->axcache = CACHE_WB_NO_ALLOC;
	else
		sp->axcache = CACHE_NONE;

	if (sp_platform->is_vpsp) {
		sp->set_psp_master_device = psp_set_master;
		sp->get_psp_master_device = psp_get_master;
		sp->clear_psp_master_device = psp_clear_master;
	}

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(48));
	if (ret) {
		dev_err(dev, "dma_set_mask_and_coherent failed (%d)\n", ret);
		goto e_err;
	}

	ret = sp_get_irqs(sp);
	if (ret)
		goto e_err;

	dev_set_drvdata(dev, sp);

	ret = sp_init(sp);
	if (ret)
		goto e_err;

	{
		struct psp_device *psp = sp->psp_data;
		struct sev_device *sev = psp->sev_data;
		sev_set_poll_handler(sev, vpsp_wait_event);
	}

	dev_notice(dev, "enabled\n");

	return 0;

e_err:
	dev_notice(dev, "initialization failed\n");
	return ret;
}

static int sp_platform_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sp_device *sp = dev_get_drvdata(dev);
	struct sp_platform *sp_platform = sp->dev_specific;

	sp_destroy(sp);

	dev_notice(dev, "disabled\n");

	return 0;
}

#ifdef CONFIG_PM
static int sp_platform_suspend(struct platform_device *pdev,
				pm_message_t state)
{
	struct device *dev = &pdev->dev;
	struct sp_device *sp = dev_get_drvdata(dev);

	return sp_suspend(sp);
}

static int sp_platform_resume(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sp_device *sp = dev_get_drvdata(dev);

	return sp_resume(sp);
}
#endif

static struct platform_driver sp_platform_driver = {
	.id_table = sp_plat_match,
	.driver = {
		.name = "ccp",
#ifdef CONFIG_ACPI
		.acpi_match_table = sp_acpi_match,
#endif
#ifdef CONFIG_OF
		.of_match_table = sp_of_match,
#endif
	},
	.probe = sp_platform_probe,
	.remove = sp_platform_remove,
#ifdef CONFIG_PM
	.suspend = sp_platform_suspend,
	.resume = sp_platform_resume,
#endif
};

int sp_platform_init(void)
{
	return platform_driver_register(&sp_platform_driver);
}

void sp_platform_exit(void)
{
	platform_driver_unregister(&sp_platform_driver);
}
