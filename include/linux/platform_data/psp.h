/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * psp.h - PSP register offsets parsed from ASPT ACPI table
 */

#ifndef __LINUX_PSP_H
#define __LINUX_PSP_H

#include <linux/types.h>

struct psp_platform_data {
	int sev_cmd_resp_reg;
	int sev_cmd_buf_lo_reg;
	int sev_cmd_buf_hi_reg;
	int feature_reg;
	int irq_en_reg;
	int irq_st_reg;
	int mbox_irq_id;
	int acpi_cmd_resp_reg;
};

#endif /* __LINUX_PSP_H */
