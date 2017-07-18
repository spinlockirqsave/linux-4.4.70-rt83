#undef TRACE_SYSTEM
#define TRACE_SYSTEM pcie215

#if !defined(_TRACE_PCIE215_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_PCIE215_H

#include <linux/tracepoint.h>

TRACE_EVENT(pcie215_event,
	TP_PROTO(__u8 id),
	TP_ARGS(id),
	TP_STRUCT__entry(
			 __field(__u8, id)
			),
	TP_fast_assign(
		       __entry->id = id;
		       ),
	TP_printk("[%u]", __entry->id)
);

TRACE_EVENT(pcie215_isr,

	TP_PROTO(__u8 irq_status, unsigned long long irq_n,
		 unsigned long long irq_spurious_n),

	TP_ARGS(irq_status, irq_n, irq_spurious_n),

	TP_STRUCT__entry(
			 __field(__u8, irq_status)
			 __field(unsigned long long, irq_n)
			 __field(unsigned long long, irq_spurious_n)
	),

	TP_fast_assign(
		       __entry->irq_status = irq_status;
		       __entry->irq_n = irq_n;
		       __entry->irq_spurious_n = irq_spurious_n;
	),

	TP_printk("IRQ# [%llu], IRQ spurious# [%llu], IRQ status [%02x]",
		  __entry->irq_n, __entry->irq_spurious_n, __entry->irq_status)
);

#endif /* _TRACE_PCIE215_H */

/* This part must be outside protection */
#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#define TRACE_INCLUDE_FILE pcie215-trace
#include <trace/define_trace.h>
