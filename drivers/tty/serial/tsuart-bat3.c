/*
 *  linux/drivers/serial/tsuart-bat3.c
 *
 *  TS-UART loader for embeddedTS TS-BAT3
 *
 *  (c) 2007 embeddedTS
 *
 * This source code is hereby licensed to the Linux 2.6 kernel under the
 * GPL, and to embeddedTS customers under the BSD license.
 *
 * 2007-11-28:mos:initial 2.6 version ported from 2.4 version
 *
 */
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/serial.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <linux/serial_core.h>
#include <asm/uaccess.h>
#include "tsuart1.h"

static unsigned int io=0,irq5=0,irq6=0;
#define BOARD_ID_BAT3 0xBA13

static struct uart_driver *dr3=0;

static int tryInitPort(int iobase) {
  struct tsuart_port *ts;
  u8 val;
  volatile unsigned char *sh;

  if (!io) {
    io = 0x81008000;
  }
  ts = kmalloc(sizeof(struct tsuart_port),GFP_KERNEL);
  if (!ts) {
    printk("Unable to allocate memory for tsuart port\n");
    return 0;
  }

  memset(ts,0,sizeof(struct tsuart_port));
  init_tsuart_port(ts);
  ts->u.iotype = SERIAL_IO_MEM;
  ts->get = tsuartGetMEM8;
  ts->put = tsuartPutMEM8;
  ts->debugFlags = portDebug;
  ts->u.mapbase = io + iobase + 2;
  ts->boardId = BOARD_ID_BAT3;
  ts->mdmctrl = 0;

  sh = ((volatile unsigned char *) ioremap((io & 0xFFFFF000), 4096)) + (io & 0xFFF);

  sh += iobase;
  val = sh[0];
  if (val == 0x28) {
    val = sh[1];
    if ((val & 0x80) == 0x80) {
      ts->u.irq = irq6; // IRQ 6
      if ((val & 0x40) == 0x40) {
	ts->localStatus |= PORT_STATUS_SHIRQ; // IRQ6 with sharing...
      }
    } else {
      ts->u.irq = irq5; // IRQ5
    }
    printk("TS-UART/BAT3 found at port 0x%04X\n",iobase);
    if (tsuart_register_port(dr3,ts) == 0) {
  printk(KERN_INFO "ts-bat3 here6\n");
      iounmap(sh);
      return 1;
    }
  }
  iounmap(sh);
  return 0;
}

static int __init tsuart_init(void)
{
  int ports=0;

  dr3 = 0;

  if (!irq5) {
    irq5 = 67;
  }
  if (!irq6) {
    irq6 = 68;
  }
  if (tryInitPort(0x100)) {
    ports++;
  }
  if (tryInitPort(0x110)) {
    ports++;
  }
  if (tryInitPort(0x200)) {
    ports++;
  }
  if (tryInitPort(0x210)) {
    ports++;
  }

  if (ports == 0) {
    printk("TS-UART/BAT3 did not detect a TS-BAT3 board\n");
  }
  return (ports > 0) ? 0 : -1;
}

static void __exit tsuart_exit(void)
{
  // This module can be safely unloaded after initialization is complete.
  // We allow module to load only so error conditions can be detected.
}

module_init(tsuart_init);
module_exit(tsuart_exit);

module_param(io, uint, 0644);
MODULE_PARM_DESC(io, "Base address for TS-BAT3 UARTS");
module_param(irq6, uint, 0644);
MODULE_PARM_DESC(io, "ARM IRQ for ISA IRQ 6");
module_param(irq5, uint, 0644);
MODULE_PARM_DESC(io, "ARM IRQ for ISA IRQ 5");
