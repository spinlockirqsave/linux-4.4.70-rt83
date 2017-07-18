#undef TRACE_SYSTEM
#define TRACE_SYSTEM comedi

#if !defined(_TRACE_COMEDI_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_COMEDI_H

#include <linux/tracepoint.h>

/*
 * Tracepoint for calling from various places inside Comedi.
 * Takes simple id and prints it to the trace log if trace
 * events from Comedi are enabled.
 *
 * Currently following tracepoints are in use:
 *
 * Id : Description
 * 0	Enter the interrupt of amplc_dio200_common
 * 1	Called from same ISR of amplc_dio200_common if IRQ is handled
 *
 * If you would like to add new tracepoint just add a call to
 *	trace_comedi_event(id)
 * with id incremented.
 */
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
