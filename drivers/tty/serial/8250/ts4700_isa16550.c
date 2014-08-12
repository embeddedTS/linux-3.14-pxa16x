#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/serial_8250.h>
#include <linux/reboot.h>
#include <linux/init.h>
#include <asm/uaccess.h>
#include <asm/bitops.h>
#include <asm/io.h>

#define FPGA_MUXBUS_BASE  0x81008000
#define IRQ_BASE 62

static struct uart_8250_port req;

static char iobase[16];
module_param_string(iobase, iobase, sizeof(iobase), 0644);
MODULE_PARM_DESC(iobase, "Base 8-bit I/O address.  Assumes 0x81008000 which is used on the "
	             "81xx series, but board with other PC104 I/O bases such as the "
                     "TS-8900 will need an argument.  eg, iobase=0x81008800");

static char irq[64];
module_param_string(irq, irq, sizeof(irq), 0644);
MODULE_PARM_DESC(irq, "IRQ Number which corresponds with the COM port. eg irq=5,6,6,7");

static char com[96];
module_param_string(com, com, sizeof(com), 0644);
MODULE_PARM_DESC(com, "Com address, eg (com=0x3f8,2f8");

static int __init ts4700_isa16550_init(void)
{
	int line = 0;
	unsigned int uart_irq[16];
	unsigned int uart_com[16];
	unsigned int uart_adr = FPGA_MUXBUS_BASE;
	int nr_ports = 0;
	int i, retVal;
	
	retVal = 0;
	if (strlen(iobase)) {
		uart_adr = simple_strtoul(iobase, NULL, 0);
	}

	if (strlen(com)) {
		char *cp = &com[0];
		for(i = 0; i < 16; i++) {
			uart_com[i] = simple_strtoul(cp, &cp, 0);
			cp++; // skip ","

			if(uart_com[i] == 0) break;
			nr_ports++;
		}
	}

	if (strlen(irq)) {
		char *cp = &irq[0];
		for(i = 0; i < 16; i++) {
			uart_irq[i] = simple_strtoul(cp, &cp, 0);
			cp++; // skip ","
			if(uart_irq[i] == 0) break;
		}
	}

	for(i = 0; i < nr_ports; i++) {
	   req.port.dev = NULL;
		req.port.type = PORT_16550A;
		req.port.iotype = UPIO_MEM;
		req.port.iobase = 0;
		req.port.fifosize = 16;
		req.port.flags = UPF_IOREMAP | UPF_SHARE_IRQ;
		req.port.regshift = 0;
		req.port.irq = IRQ_BASE + uart_irq[i];
		req.port.mapbase = uart_adr + uart_com[i];
		req.port.membase = (char *)req.port.mapbase;
		req.port.uartclk = 1843200;
		if((retVal=serial8250_register_8250_port(&req)) >= 0) {		
			line++;
		} 
	}
	
	return 0;
}

static void __exit ts4700_isa16550_exit(void)
{
   /* really ought to be calling serial8250_unregister_port() here */
}


module_init(ts4700_isa16550_init);
module_exit(ts4700_isa16550_exit);

MODULE_DESCRIPTION("TS-4700 PC/104 16550 support.  This driver should only be loaded as a module.");
MODULE_LICENSE("GPL");
