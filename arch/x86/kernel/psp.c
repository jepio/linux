// SPDX-License-Identifier: GPL-2.0-only

#include <linux/platform_data/psp.h>
#include <linux/platform_device.h>
#include <asm/hypervisor.h>

static struct platform_device psp_device = {
	.name           = "psp",
	.id             = PLATFORM_DEVID_NONE,
};

static int __init psp_init_platform_device(void)
{
	struct psp_platform_data pdata = {};
	struct resource res[1];
	int err;

	/*
	 * The ACPI PSP interface is mutually exclusive with the PCIe interface,
	 * but there is no reason to use the ACPI interface over the PCIe one.
	 * Restrict probing ACPI PSP to platforms known to only expose the ACPI
	 * interface, which at this time is SNP-host capable Hyper-V VMs.
	 */
	if (!hypervisor_is_type(X86_HYPER_MS_HYPERV))
		return -ENODEV;

	err = acpi_parse_aspt(res, &pdata);
	if (err)
		return err;
	err = platform_device_add_resources(&psp_device, res, 1);
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
