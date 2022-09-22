#undef TRACE_SYSTEM
#define TRACE_SYSTEM sev

/* SPDX-License-Identifier: GPL-2.0 */
#if !defined(_TRACE_SEV_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_SEV_H

#include <linux/tracepoint.h>
#include <asm/sev.h>

TRACE_EVENT(sev_psmash,
	TP_PROTO(u64 paddr, int ret),
	TP_ARGS(paddr, ret),
	TP_STRUCT__entry(
		__field(u64, paddr)
		__field(int, ret)
	),
	TP_fast_assign(
		__entry->paddr = paddr;
		__entry->ret   = ret;
	),
	TP_printk("paddr 0x%llx, ret %d", __entry->paddr, __entry->ret)
);

TRACE_EVENT(sev_rmpupdate,
	TP_PROTO(u64 paddr, struct rmpupdate *val, int ret),
	TP_ARGS(paddr, val, ret),
	TP_STRUCT__entry(
		__field(u64, paddr)
		__field(unsigned long, gpa)
		__field(u8, assigned)
		__field(u8, pagesize)
		__field(u8, immutable)
		__field(u32, asid)
		__field(int, ret)
	),
	TP_fast_assign(
		__entry->paddr = paddr;
		__entry->gpa = (unsigned long)val->gpa;
		__entry->assigned = val->assigned;
		__entry->pagesize = val->pagesize;
		__entry->asid = val->asid;
		__entry->ret = ret;
	),
	TP_printk("paddr 0x%llx, ret %d [assigned=%d immutable=%d pagesize=%d asid=%d gpa=0x%lx]", __entry->paddr, __entry->ret, __entry->assigned, __entry->immutable, __entry->pagesize, __entry->asid, __entry->gpa)
);

#endif /* _TRACE_SEV_H */

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH asm/trace
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE sev

/* This part must be outside protection */
#include <trace/define_trace.h>
