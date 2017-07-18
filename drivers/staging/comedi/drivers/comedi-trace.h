#undef TRACE_SYSTEM
#define TRACE_SYSTEM comedi

#if !defined(_TRACE_COMEDI_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_COMEDI_H

#include <linux/tracepoint.h>

TRACE_EVENT(comedi_event,
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

#endif /* _TRACE_COMEDI_H */

/* This part must be outside protection */
#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#define TRACE_INCLUDE_FILE comedi-trace
#include <trace/define_trace.h>
