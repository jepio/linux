// SPDX-License-Identifier: GPL-2.0-only
#define pr_fmt(fmt) "psp: " fmt
#include <linux/platform_data/psp.h>
#include <linux/platform_device.h>
#include <linux/iopoll.h>
#include <linux/irq.h>
#include <asm/apic.h>
#include <asm/hypervisor.h>
#include <asm/irqdomain.h>

#define PSP_ACPI_CMDID_SHIFT 16
#define PSP_ACPI_STATUS_SHIFT 26
#define PSP_ACPI_STATUS_MASK GENMASK(30, 26)
#define PSP_ACPI_RESPONSE_BIT BIT(31)
#define PSP_ACPI_VECTOR_MASK GENMASK(7, 0)
#define PSP_ACPI_DEST_MODE_SHIFT 9
#define PSP_ACPI_MBOX_IRQID_SHIFT 10
#define PSP_ACPI_IRQ_EN_BIT BIT(0)
#define PSP_ACPI_IRQ_EN_MBOX_IRQID_SHIFT 10

#define PSP_CMD_DELAY_US 2
#define PSP_CMD_TIMEOUT_US 10000

enum ASP_CMDID {
	ASP_CMDID_PART1  = 0x82,
	ASP_CMDID_PART2  = 0x83,
	ASP_CMDID_PART3  = 0x84,
	ASP_CMDID_IRQ_EN = 0x85,
};

enum ASP_CMD_STATUS {
	ASP_CMD_STATUS_SUCCESS = 0x0,
	ASP_CMD_STATUS_INVALID_CMD = 0x1,
	ASP_CMD_STATUS_INVALID_PARAM = 0x2,
	ASP_CMD_STATUS_INVALID_FW_STATE = 0x3,
	ASP_CMD_STATUS_FAILURE = 0x1F,
};

struct psp_irq_data {
	void __iomem *base;
	u8 mbox_irq_id;
	int acpi_cmd_resp_reg;
	struct irq_domain *domain;
};
static struct psp_irq_data pspirqd;

static int psp_sync_cmd(void __iomem *reg, u8 cmd, u16 data)
{
	u32 val;
	int err;

	val  = data;
	val |= cmd << PSP_ACPI_CMDID_SHIFT;
	writel(val, reg);
	err = readl_poll_timeout_atomic(reg, val, val & PSP_ACPI_RESPONSE_BIT, PSP_CMD_DELAY_US, PSP_CMD_TIMEOUT_US);
	if (err)
		return err;

	return (val & PSP_ACPI_STATUS_MASK) >> PSP_ACPI_STATUS_SHIFT;
}

static int psp_set_irq_enable(struct psp_irq_data *data, bool irq_en)
{
	void __iomem *reg = data->base + data->acpi_cmd_resp_reg;
	u16 val = 0;
	int err;

	if (data->mbox_irq_id > 63)
		return -EINVAL;

	val  = irq_en ? PSP_ACPI_IRQ_EN_BIT : 0;
	val |= data->mbox_irq_id << PSP_ACPI_IRQ_EN_MBOX_IRQID_SHIFT;
	err = psp_sync_cmd(reg, ASP_CMDID_IRQ_EN, val);
	if (err != ASP_CMD_STATUS_SUCCESS) {
		pr_err("ASP_CMDID_IRQ_EN failed: %d\n", err);
		return -EIO;
	}

	return 0;
}

static int psp_configure_irq(struct psp_irq_data *data, unsigned int vector, unsigned int apicid)
{
	void __iomem *reg = data->base + data->acpi_cmd_resp_reg;
	u16 part1, part2, part3;
	int err;

	if (data->mbox_irq_id > 63)
		return -EINVAL;

	part1  = apicid;
	part2  = apicid >> 16;
	part3  = vector & PSP_ACPI_VECTOR_MASK;
	part3 |= apic->dest_mode_logical << PSP_ACPI_DEST_MODE_SHIFT;
	part3 |= data->mbox_irq_id << PSP_ACPI_MBOX_IRQID_SHIFT;

	err = psp_sync_cmd(reg, ASP_CMDID_PART1, part1);
	if (err != ASP_CMD_STATUS_SUCCESS) {
		pr_err("ASP_CMDID_PART1 failed: %d\n", err);
		return -EIO;
	}
	err = psp_sync_cmd(reg, ASP_CMDID_PART2, part2);
	if (err != ASP_CMD_STATUS_SUCCESS) {
		pr_err("ASP_CMDID_PART2 failed: %d\n", err);
		return -EIO;
	}
	err = psp_sync_cmd(reg, ASP_CMDID_PART3, part3);
	if (err != ASP_CMD_STATUS_SUCCESS) {
		pr_err("ASP_CMDID_PART3 failed: %d\n", err);
		return -EIO;
	}

	return 0;
}

static int psp_irq_set_affinity(struct irq_data *data, const struct cpumask *mask, bool force)
{
	struct psp_irq_data *pspirqd = irq_data_get_irq_chip_data(data);
	struct irq_cfg *cfg;
	int err;

	err = irq_chip_set_affinity_parent(data, mask, force);
	if (err < 0 || err == IRQ_SET_MASK_OK_DONE)
		return err;

	cfg = irqd_cfg(data);
	err = psp_configure_irq(pspirqd, cfg->vector, cfg->dest_apicid);
	if (err)
		return err;

	return 0;
}

static void psp_irq_unmask(struct irq_data *data)
{
	struct psp_irq_data *pspirqd = irq_data_get_irq_chip_data(data);

	psp_set_irq_enable(pspirqd, true);
}

static void psp_irq_mask(struct irq_data *data)
{
	struct psp_irq_data *pspirqd = irq_data_get_irq_chip_data(data);

	psp_set_irq_enable(pspirqd, false);
}

static const struct irq_chip psp_irq_chip = {
	.name			= "PSP-IRQ",
	.irq_set_affinity	= psp_irq_set_affinity,
	.irq_ack		= irq_chip_ack_parent,
	.irq_retrigger  	= irq_chip_retrigger_hierarchy,
	.irq_mask	     	= psp_irq_mask,
	.irq_unmask	    	= psp_irq_unmask,
	.flags			= IRQCHIP_AFFINITY_PRE_STARTUP,
};

static int psp_irq_domain_alloc(struct irq_domain *domain, unsigned int virq,
				unsigned int nr_irqs, void *args)
{
	int err;
	int i;

	err = irq_domain_alloc_irqs_parent(domain, virq, nr_irqs, args);
	if (err)
		return err;

	for (i = virq; i < virq + nr_irqs; i++) {
		irq_set_chip_and_handler_name(i, &psp_irq_chip, handle_edge_irq, "edge");
		irq_set_chip_data(i, domain->host_data);
	}
	return 0;
}

static const struct irq_domain_ops psp_irq_domain_ops = {
	.alloc	= psp_irq_domain_alloc,
	.free	= irq_domain_free_irqs_top,
};

static int psp_init_irq(const struct psp_platform_data *pdata, const struct resource *reg,
			struct resource *irq)
{
	struct irq_alloc_info info;
	struct fwnode_handle *fn;
	void __iomem *base;
	int virq;
	int err;

	base = ioremap(reg->start, resource_size(reg));
	if (!base)
		return -ENOMEM;

	pspirqd.mbox_irq_id = pdata->mbox_irq_id;
	pspirqd.acpi_cmd_resp_reg = pdata->acpi_cmd_resp_reg;
	pspirqd.base = base;

	fn = irq_domain_alloc_named_fwnode("AMD-PSP-IRQ");
	if (!fn) {
		err = -ENOMEM;
		goto unmap;
	}

	pspirqd.domain = irq_domain_create_hierarchy(x86_vector_domain, 0, 1,
							fn,
							&psp_irq_domain_ops,
							&pspirqd);
	if (!pspirqd.domain) {
		err = -ENOMEM;
		goto freefwnode;
	}

	init_irq_alloc_info(&info, NULL);
	virq = irq_domain_alloc_irqs(pspirqd.domain, 1, NUMA_NO_NODE, &info);
	if (virq < 0) {
		err = virq;
		goto freedomain;
	}
	*irq = (struct resource)DEFINE_RES_IRQ(virq);

	return 0;

freedomain:
	irq_domain_remove(pspirqd.domain);
	pspirqd.domain = NULL;
freefwnode:
	irq_domain_free_fwnode(fn);
unmap:
	iounmap(base);

	return err;
}

static struct platform_device psp_device = {
	.name           = "psp",
	.id             = PLATFORM_DEVID_NONE,
};

static int __init psp_init_platform_device(void)
{
	struct psp_platform_data pdata = {};
	struct resource res[2];
	int err;

	/*
	 * The ACPI PSP interface is mutually exclusive with the PCIe interface,
	 * but there is no reason to use the ACPI interface over the PCIe one.
	 * Restrict probing ACPI PSP to platforms known to only expose the ACPI
	 * interface, which at this time is SNP-host capable Hyper-V VMs.
	 */
	if (!hypervisor_is_type(X86_HYPER_MS_HYPERV))
		return -ENODEV;

	err = acpi_parse_aspt(&res[0], &pdata);
	if (err)
		return err;
	err = psp_init_irq(&pdata, &res[0], &res[1]);
	if (err)
		return err;
	err = platform_device_add_resources(&psp_device, res, 2);
	if (err)
		return err;
	err = platform_device_add_data(&psp_device, &pdata, sizeof(pdata));
	if (err)
		return err;

	err = platform_device_register(&psp_device);
	if (err)
		return err;
	return 0;
}
device_initcall(psp_init_platform_device);
