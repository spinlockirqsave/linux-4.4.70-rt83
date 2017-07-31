/*
 * This is a driver for Amplicon's PCIe215 signal processing
 * controller. It enables user space process to react
 * to the external signals asserted on interrupt pins
 * of Amplicon's PCIe215 board.
 *
 * The simplest use case is:
 *
 *	1. Application opens driver
 *
 *		fd = open(PCIE215, O_RDWR);
 *
 *	2. And specifies pins allowed to trigger IRQ
 *
 *		ioctl(fd, PCIE215_IOCTL_IRQ_TRIGGERS_ENABLE, triggers);
 *
 *	3. Enables interrupt generation by PCIe215's Altera Cyclone IV fpga
 *
 *		ioctl(fd, PCIE215_IOCTL_IRQ_ENABLE, 1);
 *
 *	4. Application executes read() call and is blocked in that call
 *	waiting for external signal being asserted on one of configured
 *	(in step 2) triggers (likely in a loop)
 *
 *		ret = read(fd, NULL, 0);
 *
 *	5. When signal on any of the enabled pins has been asserted,
 *	CPU is interrupted and driver wakes up the user space process.
 *	Sleep is interruptible so signal queued for the process will
 *	wake up the process too. Process can understand the reason
 *	it has been awoken by the value returned from the read() call:
 *		it is 0 if the wake up is due to signal asserted
 *		and -1 in other cases, errno is set to IEINTR
 *		if the wake up is due to signal delivered to the process.
 */



#include <linux/device.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/fs.h>

#include <linux/wait.h>
#include <linux/sched.h>

#include <linux/kdev_t.h>
#include <linux/cdev.h>

#include <linux/miscdevice.h>

#define CREATE_TRACE_POINTS
#include "pcie215-trace.h"

#include "pcie215_ioctl.h"


#define PCIE215_DRV_VERSION	"v0.1"
#define PCIE215_DRV_AUTHOR	"Piotr Gregor <pgregor@quadrant-systems.co.uk>"
#define PCIE215_DRV_DESC	"Driver for Amplicon's PCIe215 Digital I/O Board"

#define PCIE215				"pcie215"
#define PCI_DEVICE_ID_AMPLICON_PCIE215	0x12


/*
 * Registers
 * PCIe215 is allocated two base address registers. We call them
 * BA (Base Address Configuration Register, Bar 1) and LCR (Local Bus
 * Configuration Register, Bar 0). LCR is used only to access LLIER
 * (Low Level Interrupt Enable Register). PCI Bar 1 (BA) is the main
 * register space. Interrupt sources are classified by the position
 * in the interrupt source register and each source is maskable
 * by a bit in an interrupt mask register.
 */

/*
 * PCIe215 registers (expressed as offset from base memory address)
 */
#define PCIE215_PCIE_IO_SIZE	0x4000
								/* direction	| size in bits	| description */
#define PCIE215_PORT_A_PPI_X	0x00				/* In/Out	| 8	| data to/from port A of PPI X */
#define PCIE215_PORT_B_PPI_X	0x08				/* In/Out	| 8	| data to/from port B of PPI X */
#define PCIE215_PORT_C_PPI_X	0x10				/* In/Out	| 8	| data to/from port C of PPI X */
#define PCIE215_PORT_CTRL_PPI_X	0x18				/* Out		| 8	| control word for PPI X */

#define PCIE215_PORT_A_PPI_Y	0x20				/* In/Out	| 8	| data to/from port A of PPI Y */
#define PCIE215_PORT_B_PPI_Y	0x28				/* In/Out	| 8	| data to/from port B of PPI Y */
#define PCIE215_PORT_C_PPI_Y	0x30				/* In/Out	| 8	| data to/from port C of PPI Y */
#define PCIE215_PORT_CTRL_PPI_Y	0x38				/* Out		| 8	| control word for PPI Y */

#define PCIE215_IER		0xF0				/* In/Out	| 8	| Interrupt Enable/status Register */

#define PCIE215_EFE		0x100				/* In/Out	| 8	| Enhanced Features Enable register (on == 1) */
#define PCIE215_VERSION	0x120					/* In		| 8	| Hardware Version register */
#define PCIE215_LLIER	0x50					/* In/Out	| 32	| Low-level Interrupt Enable Register, accessed from LCR */

/*
 * IRQ sources (triggers of interrupts)
 */
#define PCIE215_IRQ_SRC_N	(6)
#define PCIE215_IRQ_SRC_MASK	(0x3f)				/* bitmask of valid IRQ triggers */

/*
 * 8255 PPI Registers
 * The control register is composed of 7-bit latch circuit (D0-D6)
 * and 1-bit flag (D7)
 *	| D7	| D6 | D5 | D4 | D3 | D2 | D1 | D0	|
 *	| flag	| l a t c h				|
 */
#define PCIE215_8255_CTRL_PORTC_LO_IO	BIT(0)		/* D0 */
#define PCIE215_8255_CTRL_PORTB_IO	BIT(1)		/* D1 (0 = Output, 1 = Input) */
#define PCIE215_8255_CTRL_PORTB_MODE(x)	((x & 0x1) << 2)/* D2 */
#define PCIE215_8255_CTRL_PORTC_HI_IO	BIT(3)		/* D3 */
#define PCIE215_8255_CTRL_PORTA_IO	BIT(4)		/* D4 (0 = Output, 1 = Input) */
#define PCIE215_8255_CTRL_PORTA_MODE(x)	((x & 0x3) << 5)/* D5 and D6 */
#define PCIE215_8255_CTRL_CW_FLAG	BIT(7)		/* D7 */

/*
 * IRQ triggers
 */
#define PCIE215_IRQ_SRC_PPI_X_C0	BIT(0)		/* for external signals,pin 44, INTRB */
#define PCIE215_IRQ_SRC_PPI_X_C3	BIT(1)		/* for external signals,pin 24, INTRA */
#define PCIE215_IRQ_SRC_PPI_Y_C0	BIT(2)		/* for external signals,pin 70, INTRB */
#define PCIE215_IRQ_SRC_PPI_Y_C3	BIT(3)		/* for external signals,pin 11, INTRA */
#define PCIE215_IRQ_SRC_CTRZ1_OUT1_OP	BIT(4)		/* for onboard timers,	pin 55 */
#define PCIE215_IRQ_SRC_CTRZ2_OUT1_OP	BIT(5)		/* for onboard timers,	pin 58 */

/*
 * 8254 Timer/Counter registers
 */
#define PCIE215_8254_CTRL_Z1_0			0x80
#define PCIE215_8254_CTRL_Z1_1			0x88
#define PCIE215_8254_CTRL_Z1_2			0x90
#define PCIE215_8254_CTRL_Z1_CTRL		0x98
#define PCIE215_8254_CTRL_Z2_0			0xA0
#define PCIE215_8254_CTRL_Z2_1			0xA8
#define PCIE215_8254_CTRL_Z2_2			0xB0
#define PCIE215_8254_CTRL_Z2_CTRL		0xB8
#define PCIE215_8254_CTRL_MODE_0		0			/* mode 0 of 8254 */
#define PCIE215_8254_CTRL_COUNTER_SELECT(x)	((x) << 6)		/* select counter 0,1,2 */
#define PCIE215_8254_CTRL_RW(x)			(((x) & 0x3) << 4)	/* read/write */
#define PCIE215_8254_CTRL_RW_LATCH		PCIE215_8254_CTRL_RW(0)
#define PCIE215_8254_CTRL_RW_LSB_ONLY		PCIE215_8254_CTRL_RW(1)
#define PCIE215_8254_CTRL_RW_MSB_ONLY		PCIE215_8254_CTRL_RW(2)
#define PCIE215_8254_CTRL_RW_LSB_MSB		PCIE215_8254_CTRL_RW(3)
#define PCIE215_8254_CTRL_BCD_16_BIT		0
#define PCIE215_8254_CTRL_BCD_BCD		1


/*
 * Bit flags
 */
#define	PCIE215_FLAGS_IRQ			1			/* 1 if IRQ has been triggered on one of the pins the TASK is waitnig for, 0 otherwise */

/*
 * This parameter may be passed to insmod and/or modprobe
 * to set enabled IRQ triggers at module load time. Interruption
 * triggering is also started in that case (if triggers != 0).
 */
static unsigned long triggers;
module_param(triggers, ulong, S_IRUGO);
MODULE_PARM_DESC(triggers, "This bit mask may be passed to insmod "
		 "and/or modprobe to set enabled IRQ triggers at "
		 "module load time (interruption triggering is "
		 "also started in that case). Value to trigger|pin "
		 "mapping is (value:trigger|pin): "
		 "1:PPI-X-C0|44, 2:PPI-X-C3|24, 4:PPI-Y-C0|70, "
		 "8:PPI-Y-C3|11, 16:CTRZ1-OUT1-O/P|55, "
		 "32:CTRZ2-OUT1-O/P|58");


struct pcie215 {
	struct pci_dev	*pdev;
	spinlock_t	lock;			/* synchronizes access to driver's data between ISR and ioctl */
	unsigned int	major;
	long		irq;			/* assigned IRQ vector number */
	unsigned long	irq_flags;
	unsigned long long irq_n;
	unsigned long long irq_spurious_n;
	unsigned long long irq_missed_n;
	unsigned long	ba;			/* Base Address, main register space */
	unsigned long	lcr;			/* Local Bus configuration register space */
	void __iomem	*ba_iomem;		/* I/O remapped memory */
	void __iomem	*lcr_iomem;		/* I/O remapped memory */
	wait_queue_head_t	wait_queue;	/* List of the blocked processes awaiting for external interrupt */
	unsigned long		flags;
	__u8			triggers;
	u8	ppi_8255_config;
};

struct pcie215 pcie215_device;

/**
 *	pcie215_ioread8 - read device memory of size 8 bits.
 *	@ba:		Switch between Base Address and Local Address
 *			Configuration registers.
 *	@offset:	Offset to chosen register space.
 *
 *	If @ba is set to 1 then reading is performed at @offset
 *	from the Base Address, if ba is set to 0 then reading
 *	is at @offset from Local Bus.
 *
 *	Return: The value read.
 */
static u8 pcie215_ioread8(struct pcie215 *pcie215dev,
			  u32 offset, u8 ba)
{
	if (ba) {
		if (pcie215dev->ba_iomem)
			return ioread8(pcie215dev->ba_iomem + offset);
		else
			return inb(pcie215dev->ba + offset);
	} else {
		if (pcie215dev->lcr_iomem)
			return ioread8(pcie215dev->lcr_iomem + offset);
		else
			return inb(pcie215dev->lcr + offset);
	}
}

/**
 *	pcie215_iowrite8 - write device memory of size 8 bits.
 *	@ba:		Switch between Base Address and Local Address
 *			Configuration registers.
 *	@offset:	Offset to chosen register space.
 *
 *	If @ba is set to 1 then write is performed at @offset
 *	from the Base Address, if ba is set to 0 then write
 *	is at @offset from Local Bus.
 */
static void pcie215_iowrite8(struct pcie215 *pcie215dev, u8 val,
			     u32 offset, u8 ba)
{
	if (ba) {
		if (pcie215dev->ba_iomem)
			return iowrite8(val, pcie215dev->ba_iomem + offset);
		else
			return outb(val, pcie215dev->ba + offset);
	} else {
		if (pcie215dev->lcr_iomem)
			return iowrite8(val, pcie215dev->lcr_iomem + offset);
		else
			return outb(val, pcie215dev->lcr + offset);
	}
}


/**
 *	pcie215_iowrite32 - write device memory of size 32 bits.
 *	@ba:		Switch between Base Address and Local Address
 *			Configuration registers.
 *	@offset:	Offset to chosen register space.
 *
 *	If @ba is set to 1 then write is performed at @offset
 *	from the Base Address, if ba is set to 0 then write
 *	is at @offset from Local Bus.
 */
static void pcie215_iowrite32(struct pcie215 *pcie215dev, u32 val,
			     u32 offset, u8 ba)
{
	if (ba) {
		if (pcie215dev->ba_iomem)
			return iowrite32(val, pcie215dev->ba_iomem + offset);
		else
			return outl(val, pcie215dev->ba + offset);
	} else {
		if (pcie215dev->lcr_iomem)
			return iowrite32(val, pcie215dev->lcr_iomem + offset);
		else
			return outl(val, pcie215dev->lcr + offset);
	}
}

/**
 *	pcie215_init_8254 - Initialise 82c54 CMOS Programmable
 *				Interval Timer/Counter.
 *
 *	After power-up, the state of the 82c54 is undefined. The mode,
 *	count value and output of all counters are undefined. Each counter
 *	must be programmed before it is used. Unused counters need not
 *	to be programmed.
 */
static void pcie215_init_8254(struct pcie215 *pcie215dev)
{
	u8	mode = 0;

	int i = 0;

	for (; i < 3; ++i) {
		mode = PCIE215_8254_CTRL_COUNTER_SELECT(i) |
			PCIE215_8254_CTRL_RW_LATCH |
			PCIE215_8254_CTRL_MODE_0 |
			PCIE215_8254_CTRL_BCD_16_BIT;

		pcie215_iowrite8(pcie215dev, mode,
				 PCIE215_8254_CTRL_Z1_CTRL, 1);
		pcie215_iowrite8(pcie215dev, mode,
				 PCIE215_8254_CTRL_Z2_CTRL, 1);
	}
}

/**
 *	pcie215_init_8255 - Initialise 82c55a CMOS Programmable
 *				Peripheral Interface.
 *
 *	Initialise 8255 to Mode 0 for both PORT A and PORT B.
 *	Mode 0 is Basic Input/Output mode. No handshaking is required,
 *	data is simply written to or read from a specific port.
 *	Mode 0:
 *		- Two 8-bit (A,B) and two 4-bit (C) ports for each PPI
 *		(i.e. 4 8-bit ports in total and 4 4-bits ports in total)
 *		- Each of them can be individually programmed as Input/Output
 *			- Outputs are latched
 *			- Inputs are NOT latched
 *	This is also a default mode of PCIe215 after reset.
 */
static void pcie215_init_8255(struct pcie215 *pcie215dev)
{
	/*
	 * Configure PORT A and PORT B as inputs in MODE 0 (IRQ)
	 * (control word 0x9B)
	 */
	u8	config = (PCIE215_8255_CTRL_CW_FLAG |
			  PCIE215_8255_CTRL_PORTA_MODE(0) |
			  PCIE215_8255_CTRL_PORTA_IO |
			  PCIE215_8255_CTRL_PORTC_HI_IO |
			  PCIE215_8255_CTRL_PORTB_MODE(0) |
			  PCIE215_8255_CTRL_PORTB_IO |
			  PCIE215_8255_CTRL_PORTC_LO_IO);

	pcie215_iowrite8(pcie215dev, config, PCIE215_PORT_CTRL_PPI_X, 1);
	pcie215_iowrite8(pcie215dev, config, PCIE215_PORT_CTRL_PPI_Y, 1);
	pcie215dev->ppi_8255_config = config;
}

/**
 *	pcie215_irq_enable - Enable PCIe interrupts generation.
 *
 *	Enables generation of PCIe interrupts by Altera Cyclone IV fpga
 *	of PCIe215.
 *
 *	Lock must be taken.
 */
static void pcie215_irq_enable(struct pcie215 *pcie215dev)
{
	pcie215_iowrite32(pcie215dev, 0x80, PCIE215_LLIER, 0);
}

/**
 *	pcie215_irq_disable - Disable PCIe interrupts generation.
 *
 *	Disables generation of the interrupts by Altera Cyclone IV fpga
 *	of PCIe215.
 *
 *	Lock must be taken.
 */
static void pcie215_irq_disable(struct pcie215 *pcie215dev)
{
	pcie215_iowrite32(pcie215dev, 0x00, PCIE215_LLIER, 0);
}

/**
 *	pcie215_irq_triggers_enable - Enables pins to trigger IRQ.
 *	@enabled:	Mask of pins allowed to trigger interrupt.
 *			This gets written to interrupt Enable/Status register.
 *
 *	Lock must be taken.
 */
static void pcie215_irq_triggers_enable(struct pcie215 *pcie215dev,
					__u8 enabled)
{
	pcie215_iowrite8(pcie215dev, enabled, PCIE215_IER, 1);
}

/**
 *	pcie215_isr - Interrupt Service Routine.
 *
 *	ISR checks for spuroius IRQ by reading the Interrupt Status/Enable
 *	register. It exits if none of the configured pins is in state HIGH.
 *	It pulls LOW asserted pins by writing 0 to corresponding bit in IER
 *	in loop, until all pins are LOW. Finally, it enables interrupt
 *	triggering on all configured pins and wakes up all processes blocked
 *	in a read() call to this driver.
 *
 *	Return : IRQ_NONE if none of the configured pins is asserted,
 *	IRQ_HANDLED otherwise.
 */
static irqreturn_t pcie215_isr(int irq, void *dev_id)
{
	unsigned long	flags;
	u8		irq_status, triggered = 0;
	__u8		enabled;
	struct pcie215 *pcie215dev = (struct pcie215 *) dev_id;

	trace_pcie215_isr(0, pcie215dev->irq_n, pcie215dev->irq_spurious_n);

	spin_lock_irqsave(&pcie215dev->lock, flags);

	++pcie215dev->irq_n;		/* count IRQ */

	enabled = pcie215dev->triggers;
	irq_status = pcie215_ioread8(pcie215dev, PCIE215_IER, 1);

	if (!(irq_status & enabled)) {
		++pcie215dev->irq_spurious_n;

		trace_pcie215_isr(irq_status, pcie215dev->irq_n,
				pcie215dev->irq_spurious_n);

#if defined(DEBUG)
		dev_info(&pcie215dev->pdev->dev, "%s: IRQ spurious# [%llu] "
			"IRQ# [%llu] status [%02x]\n", __func__,
			pcie215dev->irq_spurious_n,
			pcie215dev->irq_n, irq_status);
#endif

		/*
		 * Reconfigure.
		 */
		pcie215_iowrite8(pcie215dev, pcie215dev->triggers,
				 PCIE215_IER, 1);

		spin_unlock_irqrestore(&pcie215dev->lock, flags);

		return IRQ_NONE;
	}

	trace_pcie215_isr(irq_status, pcie215dev->irq_n,
			pcie215dev->irq_spurious_n);

	/*
	 * Some of the configured IRQ pins have triggered.
	 * Collect configured pins which have been asserted.
	 *
	 * Read IRQ status until all interrupt pins have been
	 * cleared in IRQ Enable/Status register.
	 * Disable already seen triggers from @enabled mask.
	 */
	triggered = irq_status;
	enabled &= ~triggered;
	pcie215_iowrite8(pcie215dev, enabled, PCIE215_IER, 1);

#if defined(DEBUG)
	dev_info(&pcie215dev->pdev->dev, "%s: IRQ# [%llu] IRQ spurious# [%llu] "
		 "status [%02x] trig [%02x] enabled [%02x]\n", __func__,
		 pcie215dev->irq_n, pcie215dev->irq_spurious_n,
		 irq_status, triggered, enabled);
#endif

	while ((irq_status = (pcie215_ioread8(pcie215dev, PCIE215_IER, 1)
			     & enabled)) != 0) {
		triggered |= irq_status;
		enabled &= ~triggered;
		pcie215_iowrite8(pcie215dev, enabled, PCIE215_IER, 1);

#if defined(DEBUG)
		dev_info(&pcie215dev->pdev->dev, "%s: IRQ# [%llu] IRQ spurious# [%llu] "
			 "status [%02x] trig [%02x] enabled [%02x]\n",
			 __func__, pcie215dev->irq_n,
			 pcie215dev->irq_spurious_n, irq_status, triggered,
			 enabled);
#endif

	}

	/*
	 * Enable triggering.
	 */
	pcie215_iowrite8(pcie215dev, pcie215dev->triggers, PCIE215_IER, 1);

	spin_unlock_irqrestore(&pcie215dev->lock, flags);

	/*
	 * Set IRQ pending flag and wake up all processes waiting for external interrupt
	 * in interruptible state. Sync with user space thread to avoid
	 * reschedule and CPU bouncing (this is useful on UP too).
	 */
	set_bit(PCIE215_FLAGS_IRQ, &pcie215dev->flags);
	__wake_up_sync(&pcie215dev->wait_queue, TASK_INTERRUPTIBLE, 0);

	return IRQ_HANDLED;
}

/**
 * pcie215_fops_open - Opens the driver.
 */
static int pcie215_fops_open(struct inode *inode, struct file *filp)
{
	filp->private_data = &pcie215_device;
	return 0;
}

/**
 * pcie215_fops_release - Closes the driver.
 */
static int pcie215_fops_release(struct inode *inode, struct file *filp)
{
	filp->private_data = NULL;
	return 0;
}

/**
 * pcie215_fops_unlocked_ioctl - Executes ioctl operations on the driver.
 * @filp:	file handle
 * @cmd:	ioctl identifier. This identifier may be one of:
 *
 *		PCIE215_IOCTL_IRQ_ENABLE
 *		Turns on/off trigggring of the interrupts on the global basis,
 *		i.e. by the Altera Cyclone IV fpga chip.
 *
 *		PCIE215_IOCTL_IRQ_TRIGGERS_ENABLE
 *		Configures pins for interrupt triggering. Only pins configured
 *		this way will wake up the blocked processes. Signals asserted
 *		on other pins are ignored. By default all pins are considered
 *		"disabled" (unless triggers=X has been passed to the insmod,
 *		in which case pins given in X bit mask are immediately enabled).
 *
 *		PCIE215_IOCTL_IRQ_TRIGGERS_DISABLE
 *		Disables the pins from the mask of pins allowed to wake up
 *		the processes.
 *
 * @arg:	ioctl parameter. The value of this argument to ioctl
 *		is considered in context of the ioctl identifier.
 *
 *		PCIE215_IOCTL_IRQ_ENABLE
 *		@arg can be 0  for global disable, or 1 for global enable.
 *
 *		PCIE215_IOCTL_IRQ_TRIGGERS_ENABLE
 *		@arg is a bit mask of pins allowed for interrupt triggering.
 *		Enabling new pins adds them to the previous mask (history is
 *		kept).
 *		Bit 0 is PPI-X-C0 (pin 44)
 *		Bit 1 is PPI-X-C3 (pin 24)
 *		Bit 2 is PPI-Y-C0 (pin 70)
 *		Bit 3 is PPI-Y-C3 (pin 11)
 *		Bit 4 is CTRZ1-OUT1-O/P (pin 55) - not used
 *		Bit 5 is CTRZ2-OUT1-O/P (pin 58) - not used
 *
 *		PCIE215_IOCTL_IRQ_TRIGGERS_DISABLE
 *		@arg is a bit mask of pins removed from the bit mask of pins
 *		allowed for interrupt triggering. Previously enabled pins
 *		which are not disabled in the mask passed to this call
 *		stays enabled.
 *
 * Return: 0 if ioctl successfull, error code otherwise.
 */
static long pcie215_fops_unlocked_ioctl(struct file *filp, unsigned int cmd,
					unsigned long arg)
{
	unsigned long	flags;
	__u8	triggers = 0;
	struct pcie215 *pcie215dev = filp->private_data;

	if (!pcie215dev)
		return -ENODEV;

	if (_IOC_TYPE(cmd) != PCIE215_IOCTL_MAGIC)
		return -ENOTTY;

	switch (cmd) {

		case PCIE215_IOCTL_IRQ_ENABLE:

			if (arg) {

				/*
				 * Enable interrupt triggering by Altera fpga.
				 */
				spin_lock_irqsave(&pcie215dev->lock, flags);

				pcie215_irq_enable(pcie215dev);

				spin_unlock_irqrestore(&pcie215dev->lock, flags);
			} else {

				/*
				 * Disable interrupt triggering by Altera fpga.
				 */
				spin_lock_irqsave(&pcie215dev->lock, flags);

				pcie215_irq_disable(pcie215dev);
				clear_bit(PCIE215_FLAGS_IRQ, &pcie215dev->flags);

				spin_unlock_irqrestore(&pcie215dev->lock, flags);
			}
			break;

		case PCIE215_IOCTL_IRQ_TRIGGERS_ENABLE:

			/*
			 * Enable interrupt triggering for specified IRQ sources.
			 */
			spin_lock_irqsave(&pcie215dev->lock, flags);

			triggers = arg & PCIE215_IRQ_SRC_MASK;
			pcie215dev->triggers |= triggers;
			pcie215_irq_triggers_enable(pcie215dev, triggers);

			spin_unlock_irqrestore(&pcie215dev->lock, flags);
			break;

		case PCIE215_IOCTL_IRQ_TRIGGERS_DISABLE:

			/*
			 * Disable interrupt triggering for specified IRQ sources.
			 */
			spin_lock_irqsave(&pcie215dev->lock, flags);

			/*
			 * Update enabled triggers.
			 */
			triggers = arg & PCIE215_IRQ_SRC_MASK;
			pcie215dev->triggers &= ~triggers;

			/*
			 * And reconfigure hardware.
			 */
			pcie215_irq_triggers_enable(pcie215dev,
						    pcie215dev->triggers);

			spin_unlock_irqrestore(&pcie215dev->lock, flags);
			break;

		default:
			return -ENOTTY;
	}

	return 0;
}

static long pcie215_fops_compat_ioctl(struct file *filp, unsigned int cmd,
				      unsigned long arg)
{
	return pcie215_fops_unlocked_ioctl(filp, cmd, arg);
}

/**
 * pcie215_fops_read - Put process to sleep until signal is asserted.
 *
 * Return: 0 if interrupted due to physical signal being asserted
 * on configured pin(s), -ERESTARTSYS if interrupted due to software
 * signal delivered to the process.
 */
static ssize_t pcie215_fops_read(struct file *filp, char __user *buf,
				 size_t bytes_n, loff_t *offset)
{
	struct pcie215 *pcie215dev = filp->private_data;

	trace_pcie215_event(0);

	/*
	 * Clear IRQ pending flag
	 */
	clear_bit(PCIE215_FLAGS_IRQ, &pcie215dev->flags);

	/*
	 * Put process to sleep until any interrupt is triggered
	 * due to the signal being asserted on configured pin(s).
	 * Allow to be interrupted by signals.
	 */
	return wait_event_interruptible(pcie215dev->wait_queue,
				 test_bit(PCIE215_FLAGS_IRQ,
					  &pcie215dev->flags));
}

static const struct file_operations pcie215_fops =  {
	.owner = THIS_MODULE,
	.open = pcie215_fops_open,
	.release = pcie215_fops_release,
	.unlocked_ioctl = pcie215_fops_unlocked_ioctl,
	.compat_ioctl = pcie215_fops_compat_ioctl,
	.read = pcie215_fops_read,
};

static struct miscdevice pcie215_misc_dev_t = {
	.name = PCIE215,
	.fops = &pcie215_fops,
	.parent = NULL,
	.this_device = NULL,
	.nodename = NULL,
	.mode = (S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH),
};

static void pcie215_config_print(struct pcie215 *pcie215dev)
{
	struct pci_dev *pdev = pcie215dev->pdev;

	dev_info(&pdev->dev, "%s: Attached pcie215 to PCI [%s]: interrupt [%ld]"
		 " major [%u]", __func__, pci_name(pdev),
		 pcie215dev->irq, pcie215dev->major);

	if (pcie215dev->ba_iomem)
		dev_info(&pdev->dev, "%s: Base Address Configuration Register "
			 "[%p] (I/O memory)", __func__, pcie215dev->ba_iomem);
	else
		dev_info(&pdev->dev, "%s: Base Address Configuration Register "
			 "[%08lx]", __func__, pcie215dev->ba);

	if (pcie215dev->lcr_iomem)
		dev_info(&pdev->dev, "%s: Local Bus Configuration Register "
			 "[%p] (I/O memory)", __func__, pcie215dev->lcr_iomem);
	else
		dev_info(&pdev->dev, "%s: Local Bus Configuration Register "
			 "[%08lx]", __func__, pcie215dev->lcr);

	dev_info(&pdev->dev, "%s: 8255 configured as [%02x]",
		 __func__, pcie215dev->ppi_8255_config);
}

/**
 * pcie215_probe - Driver's and hardware initialisation.
 *
 * This function performs initialisation of the PCIe215 board
 * and of the pcie215 driver. Apart from standard steps required
 * to initialise PCIe hardware driver does these additional steps:
 *
 *	- Initialise 8255 PPI
 *	- Initialise 8254 Timer/Counter
 *	- Disable PCIe interrupts generation by Altera
 *	  Cyclone IV fpga of PCIe215
 *	- If triggers=X was passed to the insmod, then:
 *		1. Enable interrupt triggering on pins
 *		   given in a bit mask X
 *		2. If X > 0 then enable IRQ generation
 *		   on a global basis
 *
 * Return : 0 on success, negative error code otherwise.
 */
static int pcie215_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	int err = 0;
	struct pcie215 *pcie215dev = &pcie215_device;
	unsigned int bar = 0;

	dev_info(&pdev->dev, "%s: Probing...\n", __func__);

	if (!pdev)
		return -ENODEV;

	init_waitqueue_head(&pcie215dev->wait_queue);
	spin_lock_init(&pcie215dev->lock);

	pci_set_drvdata(pdev, pcie215dev);

	err = pci_enable_device(pdev);
	if (err) {
		dev_err(&pdev->dev, "%s: pci_enable_device failed: %d\n",
			__func__, err);
		return err;
	}

	err = pci_request_regions(pdev, "pcie215");
	if (err) {
		dev_err(&pdev->dev, "%s: pci_request_regions failed: %d\n",
			__func__, err);
		goto err_disable;
	}

	/*
	 * Map Local Bus register space.
	 */
	bar = 0;
	if (pci_resource_flags(pdev, bar) & IORESOURCE_MEM) {
		pcie215dev->lcr_iomem = pci_ioremap_bar(pdev, bar);
		if (!pcie215dev->lcr_iomem) {
			dev_err(&pdev->dev, "%s: pci_ioremap failed, can't remap "
				"Local Bus configuration register space: %d\n",
				__func__, err);
			err = -ENOMEM;
			goto err_release_regions;
		}
	} else {
		pcie215dev->lcr = pci_resource_start(pdev, bar);
	}

	/*
	 * Map Base Address register space.
	 */
	bar = 1;
	if (pci_resource_flags(pdev, bar) & IORESOURCE_MEM) {
		pcie215dev->ba_iomem = pci_ioremap_bar(pdev, bar);
		if (!pcie215dev->ba_iomem) {
			dev_err(&pdev->dev, "%s: pci_ioremap failed, can't remap "
				"Base Address configuration register space: %d\n",
				__func__, err);
			err = -ENOMEM;
			goto err_release_regions;
		}
	} else {
		pcie215dev->ba = pci_resource_start(pdev, bar);
	}

	/*
	 * Initialise 8255 PPI.
	 */
	pcie215_init_8255(pcie215dev);

	/*
	 * Initialise 8254 Timer/Counter.
	 */
	pcie215_init_8254(pcie215dev);

	if (!pdev->irq) {
		dev_err(&pdev->dev, "%s: No IRQ assigned to the device\n",
			__func__);
		err = -ENODEV;
		goto err_release_regions;
	}

	if (!pci_intx_mask_supported(pdev)) {
		dev_err(&pdev->dev, "%s: No support for INTx masking of "
			"interrupts on this device\n", __func__);
		err = -ENODEV;
		goto err_release_regions;
	}

	pcie215dev->irq = pdev->irq;
	pcie215dev->pdev = pdev;

	/*
	 * Disable PCIe interrupts generation by Altera Cyclone IV fpga of PCIe215.
	 */
	pcie215_irq_disable(pcie215dev);

	err = request_irq(pcie215dev->irq, pcie215_isr, IRQF_SHARED, PCIE215,
			  pcie215dev);
	if (err) {
		dev_err(&pdev->dev, "%s: Failed to register ISR\n",
			__func__);
		err = -ENODEV;
		goto err_release_regions;
	}

	/*
	 * Request dynamic major number.
	 */
	err = misc_register(&pcie215_misc_dev_t);
	if (err) {
		dev_err(&pdev->dev, "%s: Err, can't register character device",
			__func__);
		err = -ENODEV;
		goto err_release_regions;
	}

	pcie215dev->major = MAJOR(pcie215_misc_dev_t.this_device->devt);

	/*
	 * Dump the config.
	 */
	pcie215_config_print(pcie215dev);

	if (triggers) {
		/*
		 * If the triggers parameter has been given on the insmod
		 * commandline or in the modprobe config, then enable triggers.
		 */
		triggers &= PCIE215_IRQ_SRC_MASK;
		pcie215dev->triggers = triggers;
		pcie215_irq_triggers_enable(pcie215dev, triggers);
		pcie215_irq_enable(pcie215dev);

		dev_info(&pdev->dev, "%s: Enabled IRQ generation and triggers "
			 "[%02x]", __func__, pcie215dev->triggers);
	}

	return err;

err_release_regions:

	if (pcie215dev->ba_iomem) {
		pci_iounmap(pdev, pcie215dev->ba_iomem);
		pcie215dev->ba_iomem = NULL;
	}

	if (pcie215dev->lcr_iomem) {
		pci_iounmap(pdev, pcie215dev->lcr_iomem);
		pcie215dev->lcr_iomem = NULL;
	}

	pci_release_regions(pdev);

err_disable:

	pci_disable_device(pdev);

	return err;
}

static void pcie215_remove(struct pci_dev *pdev)
{
	struct pcie215 *pcie215dev = pci_get_drvdata(pdev);

	dev_info(&pdev->dev, "%s: Removing... IRQ# [%llu] IRQ spurious# [%llu]"
		 " major [%u]\n", __func__, pcie215dev->irq_n,
		 pcie215dev->irq_spurious_n, pcie215dev->major);

	if (!pdev || !pcie215dev)
		return;

	/*
	 * Disable all IRQ sources and PCIe interrupts generation by Altera
	 * Cyclone IV fpga of PCIe215.
	 */
	pcie215_irq_disable(pcie215dev);

	if (pcie215dev && pcie215dev->irq) {
		free_irq(pcie215dev->irq, pcie215dev);
		pcie215dev->irq = 0;
	}

	if (pcie215dev->ba_iomem) {
		pci_iounmap(pdev, pcie215dev->ba_iomem);
		pcie215dev->ba_iomem = NULL;
	}

	if (pcie215dev->lcr_iomem) {
		pci_iounmap(pdev, pcie215dev->lcr_iomem);
		pcie215dev->lcr_iomem = NULL;
	}

	pci_release_regions(pdev);
	pci_disable_device(pdev);

	misc_deregister(&pcie215_misc_dev_t);
}

static int pcie215_suspend(struct pci_dev *pdev, pm_message_t state)
{
	unsigned long flags;

	struct pcie215 *pcie215dev = pci_get_drvdata(pdev);

	if (!pcie215dev)
		return 0;

	spin_lock_irqsave(&pcie215dev->lock, flags);

	/* Disable interrupt triggering before we go to power save state */
	pcie215_irq_disable(pcie215dev);

	if (pcie215dev->irq >= 0) {
		synchronize_irq(pcie215dev->irq);
		free_irq(pcie215dev->irq, pcie215dev);
		pcie215dev->irq = -1;
	}

	/* Save PCI configuration space */
	pci_save_state(pdev);

	pci_disable_device(pdev);

	pci_set_power_state(pdev, pci_choose_state(pdev, state));

	spin_unlock_irqrestore(&pcie215dev->lock, flags);

	return 0;
}

static int pcie215_resume(struct pci_dev *pdev)
{
	struct pcie215 *pcie215dev = pci_get_drvdata(pdev);

	if (!pcie215dev)
		return 0;

	pci_set_power_state(pdev, PCI_D0);
	pci_restore_state(pdev);

	if (pci_enable_device(pdev) < 0) {
		dev_err(&pdev->dev, "%s: Enabling device failed\n", __func__);
		return -EIO;
	}

	/* Do not enable bus-mastering */

	/* Enable interrupt triggering */
	pcie215_irq_enable(pcie215dev);

	return 0;
}

static const struct pci_device_id pcie215_supported_devices [] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_AMPLICON, PCI_DEVICE_ID_AMPLICON_PCIE215) },
	{ }
};

static struct pci_driver pcie215_driver = {
	.name = PCIE215,
	.id_table = pcie215_supported_devices,
	.probe = pcie215_probe,
	.remove = pcie215_remove,
	.suspend = pcie215_suspend,
	.resume = pcie215_resume,
};

module_pci_driver(pcie215_driver);
MODULE_VERSION(PCIE215_DRV_VERSION);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR(PCIE215_DRV_AUTHOR);
MODULE_DESCRIPTION(PCIE215_DRV_DESC);
