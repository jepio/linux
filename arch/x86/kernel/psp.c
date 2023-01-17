// SPDX-License-Identifier: GPL-2.0-only
#define pr_fmt(fmt) "psp: " fmt
#include <linux/platform_data/psp.h>
#include <linux/platform_device.h>
#include <linux/iopoll.h>
#include <linux/irq.h>
#include <asm/hypervisor.h>
#include <asm/irqdomain.h>

#define PSP_ACPI_DATA_SHIFT 0
#define PSP_ACPI_DATA_MASK GENMASK(15, 0)
#define PSP_ACPI_CMDID_SHIFT 16
#define PSP_ACPI_CMDID_MASK GENMASK(25, 16)
#define PSP_ACPI_STATUS_SHIFT 26
#define PSP_ACPI_STATUS_MASK GENMASK(30, 26)
#define PSP_ACPI_RESPONSE_BIT BIT(31)

#define PSP_ACPI_VECTOR_SHIFT 0
#define PSP_ACPI_VECTOR_MASK GENMASK(7, 0)
#define PSP_ACPI_MBOX_IRQID_SHIFT 10
#define PSP_ACPI_MBOX_IRQID_MASK GENMASK(15, 10)

#define PSP_ACPI_IRQ_EN_BIT BIT(0)
#define PSP_ACPI_IRQ_EN_MBOX_IRQID_SHIFT 10
#define PSP_ACPI_IRQ_EN_MBOX_IRQID_MASK GENMASK(15, 10)

// AMD Secure Processor
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
	int mbox_irq_id;
	int acpi_cmd_resp_reg;
};

static int psp_sync_cmd(void __iomem *reg, u8 cmd, u16 data)
{
	u32 val = 0;
	int err;

	val |= data & PSP_ACPI_DATA_MASK;
	val |= (cmd << PSP_ACPI_CMDID_SHIFT) & PSP_ACPI_CMDID_MASK;
	writel(val, reg);
	err = readl_poll_timeout_atomic(reg, val, val & PSP_ACPI_RESPONSE_BIT, 2, 10000);
	if (err < 0)
		return err;
	return (val & PSP_ACPI_STATUS_MASK) >> PSP_ACPI_STATUS_SHIFT;
}

static int psp_set_irq_enable(struct psp_irq_data *data, bool irq_en)
{
	void __iomem *reg = data->base + data->acpi_cmd_resp_reg;
	u8 mbox_irq_id = data->mbox_irq_id;
	u16 val = 0;
	int err;

	val |= irq_en ? PSP_ACPI_IRQ_EN_BIT : 0;
	val |= (mbox_irq_id << PSP_ACPI_IRQ_EN_MBOX_IRQID_SHIFT) & PSP_ACPI_IRQ_EN_MBOX_IRQID_MASK;
	err = psp_sync_cmd(reg, ASP_CMDID_IRQ_EN, val);
	if (err != ASP_CMD_STATUS_SUCCESS) {
		pr_err("ASP_CMDID_IRQ_EN failed: %d\n", err);
		return -EIO;
	}
	return 0;
}

static int psp_configure_irq(struct psp_irq_data *data, int vector, unsigned int cpu)
{
	void __iomem *reg = data->base + data->acpi_cmd_resp_reg;
	unsigned int dest_cpu = cpu_physical_id(cpu);
	u8 mbox_irq_id = data->mbox_irq_id;
	u16 part1, part2, part3;
	int err;

	part1 = dest_cpu;
	part2 = dest_cpu >> 16;
	part3 = vector & PSP_ACPI_VECTOR_MASK;
	part3 |= (mbox_irq_id << PSP_ACPI_MBOX_IRQID_SHIFT) & PSP_ACPI_MBOX_IRQID_MASK;

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


static int psp_init_irq(const struct psp_platform_data *pdata,
	const struct resource *reg, struct resource *irq)
{
	struct psp_irq_data pspirqd;
	struct irq_alloc_info info;
	struct irq_data *data = NULL;
	struct irq_cfg *cfg = NULL;
	void __iomem *base = NULL;
	int virq;
	int err;

	base = ioremap(reg->start, resource_size(reg));
	if (IS_ERR(base))
		return PTR_ERR(base);
	if (!base)
		return -ENOMEM;
	pspirqd.mbox_irq_id = pdata->mbox_irq_id;
	pspirqd.acpi_cmd_resp_reg = pdata->acpi_cmd_resp_reg;
	pspirqd.base = base;
	init_irq_alloc_info(&info, cpumask_of(0));
	virq = irq_domain_alloc_irqs(NULL, 1, NUMA_NO_NODE, &info);
	if (virq <= 0) {
		pr_err("failed to allocate vector: %d\n", virq);
		err = -ENOMEM;
		goto unmap;
	}
	irq_set_handler(virq, handle_edge_irq);
	data = irq_get_irq_data(virq);
	if (!data) {
		pr_err("no irq data\n");
		err = -ENODEV;
		goto freeirq;

	}
	cfg = irqd_cfg(data);
	if (!data) {
		pr_err("no irq cfg\n");
		err = -ENODEV;
		goto freeirq;
	}
	err = psp_configure_irq(&pspirqd, cfg->vector, 0);
	if (err) {
		pr_err("failed to configure irq: %d\n", err);
		goto freeirq;
	}
	err = psp_set_irq_enable(&pspirqd, true);
	if (err) {
		pr_err("failed to enable irq: %d\n", err);
		goto freeirq;
	}

	*irq = (struct resource)DEFINE_RES_IRQ(virq);
	iounmap(base);
	return 0;

freeirq:
	irq_domain_free_irqs(virq, 1);
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
