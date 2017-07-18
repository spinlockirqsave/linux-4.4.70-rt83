#ifndef PCIE215_IOCTL_H
#define PCIE215_IOCTL_H


#ifndef __KERNEL__
	#include <asm/ioctl.h>
#endif


/*
 * Ioctl interface.
 */
#define PCIE215_IOCTL_MAGIC		'4'

/* Enable/Disable interrupts triggering by Altera fpga on PCIe215 */
#define PCIE215_IOCTL_IRQ_ENABLE		_IOW(PCIE215_IOCTL_MAGIC, 0, \
						     unsigned long)
/* Enable/Disable interrupts triggers on PCIe215 */
#define PCIE215_IOCTL_IRQ_TRIGGERS_ENABLE	_IOW(PCIE215_IOCTL_MAGIC, 1, \
						     unsigned long)
#define PCIE215_IOCTL_IRQ_TRIGGERS_DISABLE	_IOW(PCIE215_IOCTL_MAGIC, 2, \
						     unsigned long)

#endif // PCIE215_IOCTL_H
