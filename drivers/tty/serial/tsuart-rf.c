/*
 *  linux/drivers/serial/tsuart-rf.c
 *
 *  TS-UART loader for Technologic Systems TS-RF2
 *
 *  (c) 2006 Technologic Systems
 *
 * This source code is hereby licensed to the Linux 2.4 kernel under the
 * GPL, and to Technologic Systems customers under the BSD license.
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

static unsigned long io=0;
static int irq6=0,irq7=0;
#define BOARD_ID_TSRF 0x5162

static int mdmctrl(struct tsuart_port *ts,int mctrl) {
  int dtr=1;

  if (mctrl & 0x80000000) {
    tsuartPutMEM8(ts,-1,
		 (tsuartGetMEM8(ts,-1) & 0xFE) | 
		 ((mctrl & TIOCM_DTR) ? 1 : 0));
    dtr =  mctrl & TIOCM_DTR;
  } else {
    dtr = (tsuartGetMEM8(ts,-1) & 0x01) ? TIOCM_DTR : 0;
  }
  return (TIOCM_CAR | TIOCM_DSR | dtr);
}

static int tryInitPort(int iobase) {
  struct tsuart_port *ts;
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
  ts->boardId = BOARD_ID_TSRF;
  ts->mdmctrl = mdmctrl;

  sh = ((volatile unsigned char *) ioremap((io & 0xFFFFF000), 4096)) + (io & 0xFFF);
 if (sh == 0) {
   printk(KERN_ERR "tsuart-rf: cannot map the 8-bit IO + iobase 0x%X\n", iobase);
   return -ENOMEM;
  }
  sh += iobase;
  if (sh[0] == 0x8e) {
    ts->localStatus |= PORT_STATUS_SHIRQ; //share IRQ6 & IRQ7 with IDE
    if ((sh[1] & 0x80) == 0x80) {
      ts->u.irq = irq7; // IRQ 7
    } else {
      ts->u.irq = irq6; // IRQ 6
    }
    printk("TS-UART/RF2 found at port 0x%04X\n",iobase);
    if (tsuart_register_port(0,ts) == 0) {
      __arm_iounmap(sh);
      return 1;
    }
  }
  __arm_iounmap(sh);

  return 0;
}

static int __init tsuart_init(void)
{
  int ports=0;

  if (!irq6) {
    irq6 = 68;
  }
  if (!irq7) {
    irq7 = 69;
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
    printk("TS-UART/RF2 did not detect a TS-RF2 board\n");
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

module_param(io, ulong, 0644);
MODULE_PARM_DESC(io, "Base address for TS-RF2 UARTS");
module_param(irq6, int, 0644);
MODULE_PARM_DESC(io, "ARM IRQ for ISA IRQ 6");
module_param(irq7, int, 0644);
MODULE_PARM_DESC(io, "ARM IRQ for ISA IRQ 7");

