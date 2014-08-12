/*
 *  linux/drivers/serial/tsuart1.c
 *
 *  Linux 2.6 Driver for Technologic Systems UART
 *
 *  (c) 2007 Technologic Systems
 *
 * This source code is hereby licensed to the Linux 2.4 kernel under the
 * GPL, and to Technologic Systems customers under the BSD license.
 *
 * 2007-09-27:mos:initial version completed
 * 2007-12-21:mos:now support "Slow" baud rates for boards that can do that
 *               :fixed parity typo bug
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

unsigned int invert_rts = 0;
unsigned int invert_cts = 0;
unsigned int invert_rts_cts = 0;

//---------------------------------------------------------------------------
static void tsuart_stop_tx(struct uart_port *port);
static unsigned int tsuart_get_mctrl(struct uart_port *port);
//---------------------------------------------------------------------------

static struct timer_list timer;
static struct timer_list timer2 = { .function = NULL };
static int timer_added = 0;
static int started = 0;
static struct tsuart_port *TS = 0;

//---------------------------------------------------------------------------
// HARDWARE ACCESS
//---------------------------------------------------------------------------

#define TSUART_REG(name,offset) \
static inline unsigned tsuartGet##name(struct tsuart_port *port) {\
  return port->get(port,offset);\
}\
static inline void tsuartPut##name(struct tsuart_port *port,unsigned value) {\
  port->put(port,offset,value);\
}

// TS7XXX_IO8_BASE must be added to iobase
unsigned tsuartGetIO8(struct tsuart_port *port,int offset) {
  return inb(port->u.iobase + offset);
}

unsigned tsuartGetMEM8(struct tsuart_port *port,int offset) {
  return *((volatile unsigned char *)port->u.membase + offset);
}

unsigned tsuartGetMEM16(struct tsuart_port *port,int offset) {
  return *((volatile unsigned short *)port->u.membase + offset);
  /*
  unsigned value;
  value = *((volatile unsigned short *)port->u.membase + offset);
  if (1 || offset == 1) {
    printk("RX:%X @%d.%d\n",value,port->u.line,offset);
  }
  return value;
  */
}

unsigned tsuartGetMEM16_9(struct tsuart_port *port,int offset) {
  unsigned val = *((volatile unsigned short *)port->u.membase + offset);

  if (offset == 1) {
    return val & 0x1FF;
  } else {
    return val;
  }
}

void tsuartPutIO8(struct tsuart_port *port,int offset,unsigned value) {
  outb(value,port->u.iobase + offset);
}

void tsuartPutMEM8(struct tsuart_port *port,int offset,unsigned value) {
  *((volatile unsigned char *)port->u.membase + offset) = value;
}

void tsuartPutMEM16(struct tsuart_port *port,int offset,unsigned value) {
  /*
  if (1 || offset == 1) {
    printk("TX:%X (@%d)\n",value,port->u.line);
  }
  */
  *((volatile unsigned short *)port->u.membase + offset) = value;
}

static void tsuart_try_handleTx(struct tsuart_port *port);

void tsuartPutMEM16_9(struct tsuart_port *port,int offset,unsigned value) {
  if (offset != 1) {
    *((volatile unsigned short *)port->u.membase + offset) = value;
  } else if (port->pstatus & PORT_STATUS_TXMSB) {
    if ((value & 0xFE) == 0) {
      port->pstatus = (port->pstatus & 0xFFFF00FF) | ((value & 0xFF) << 8);
      port->pstatus &= ~PORT_STATUS_TXMSB;
    } else {
      if (port->debugFlags & DEBUG_DATA) {      
	printk("***TX Rejected (sync)\n");
      }
    }
    //tsuart_try_handleTx(port); // avoid stuckness
  } else {
    value += (port->pstatus & 0xFF00);
    *((volatile unsigned short *)port->u.membase + offset) = value;
    port->pstatus |= PORT_STATUS_TXMSB;
  }
}

TSUART_REG(ID,-2)
TSUART_REG(PLDREV,-1)
TSUART_REG(STAT,0)
TSUART_REG(DATA,1)

static inline void setMODE9(struct uart_port *port,int on) {
  tsuartPutSTAT(TSPORT(port),assign_bit_MODE9(tsuartGetSTAT(TSPORT(port)),on));
}

//---------------------------------------------------------------------------
// OS TX/RX INTERFACE
//---------------------------------------------------------------------------

// process the byte in the lsb of data by sending it to the
// appropriate OS buffers for the given TS UART UART
static void tsuart_handleRx(struct tsuart_port *port,int data) {
  struct tty_port *tty;
  unsigned int ch = data;
  int flag = 0;

  //printk("%02X ",ch);
  if (port->debugFlags & DEBUG_DATA) {
    unsigned int ttyinf = port->u.state ? (port->u.state->port.tty ? 3:1) : 0;
    printk("tsuart Rx(%d):%02X:%d\n",port->u.line,ch,ttyinf);
  }
  if (!port->u.state || !port->u.state->port.tty) {
    return; 
  }
  //tty = port->u.state->port.tty;
  tty = &port->u.state->port;
  /*
  if (tty->flip.count >= TTY_FLIPBUF_SIZE) {
    tty->flip.tqueue.routine((void *)tty);
    //printk("->%X ",tty->flip.char_buf_ptr);
    if (tty->flip.count >= TTY_FLIPBUF_SIZE) {
      printk("TSUART0 - TTY FLIP BUF FAILED!\n");
      return;
    }
  }
  */
  if (port->localStatus & PORT_STATUS_PARITY) {
    flag |= TTY_PARITY;
  }

  if (port->rxsize > 3) {
    tty_insert_flip_char(tty, (ch >> 24) & 0xFF, flag);
  }
  if (port->rxsize > 2) {
    tty_insert_flip_char(tty, (ch >> 16) & 0xFF, flag);
  }
  if (port->rxsize > 1) {
    tty_insert_flip_char(tty, (ch >> 8) & 0xFF, flag);
  }
  /*
  if (tty->flip.count >= TTY_FLIPBUF_SIZE) {
    tty->flip.tqueue.routine((void *)tty);
    if (tty->flip.count >= TTY_FLIPBUF_SIZE) {
      printk("TSUART0 - TTY FLIP BUF FAILED!\n");
      return;
    }
  }
  */
  tty_insert_flip_char(tty, (ch & 0xFF), flag);
  port->gotBytes++;
  return;
}

static inline void tsuart_parity_Tx(struct tsuart_port *port,unsigned data) {
  if (port->parity) {
    //    printk("Tx parity: %X -> ",data);
    data = port->parity[data & port->dataMask2];
    //    printk("%X\n",data);
  }
  tsuartPutDATA(port,data);
}

// If there is data ready to be transmitted for the given UART,
// then feed to the byte to the UART
static void tsuart_try_handleTx(struct tsuart_port *port) {
  struct circ_buf *xmit;
  unsigned int ch=0,i=0;

  if (!port->u.state || !port->u.state->port.tty) {
    if (port->debugFlags & DEBUG_DATA) {
      unsigned int ttyinf = port->u.state ? (port->u.state->port.tty ? 3:1) : 0;
      printk("tsuart Tx(%d) <%d>\n",port->u.line,ttyinf);
    }
    return;
  }

  if (port->u.flags & ASYNC_CTS_FLOW) {
    // if we have HWCTS we can't also implement
    // it in software or else we will deadlock and not send any data!
    if (!(port->localStatus & PORT_STATUS_HWCTS)) {
      if (invert_cts != bit_CTS(tsuartGetSTAT(port))) {
	if (port->debugFlags & DEBUG_INTERRUPT) {
	  if (port->u.state->port.tty->hw_stopped) printk("CTS start\n");
	}
	port->u.state->port.tty->hw_stopped = 0;
      } else {
	if (port->debugFlags & DEBUG_INTERRUPT) {
	  if (!port->u.state->port.tty->hw_stopped) printk("CTS stop\n");
	}
	port->u.state->port.tty->hw_stopped = 1;
	if (!timer_added) {
	  timer_added = 1;
	  if (port->debugFlags & DEBUG_INTERRUPT) {
	    printk("Adding timer (%d)\n",port->u.line);
	  }
	  timer.expires = jiffies + DELAY_TIME;
	  init_timer(&timer);
	  add_timer(&timer);
	}
	return;
      }
    }
  }

  xmit = &port->u.state->xmit;
  if (uart_circ_empty(xmit)) { // || uart_tx_stopped(&(port->u))) {
    return;
  }

  i = 0;
  //printk("%d:%d %p\n",xmit->tail,xmit->head,&i);
  do {
    // handle one character
    ch = xmit->buf[xmit->tail];
    if (port->debugFlags & DEBUG_DATA) {
      unsigned int ttyinf = port->u.state ? (port->u.state->port.tty ? 3:1) : 0;
      printk("tsuart Tx(%d):%02X:%d\n",port->u.line,ch,ttyinf);
    }
    xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
    port->u.icount.tx++;
    tsuart_parity_Tx(port,ch);
    // done handling one character
    i++;
    if (uart_circ_empty(xmit)) { // || uart_tx_stopped(&(port->u))) {
      break;
    }
  } while (bit_TBRE(tsuartGetSTAT(port)) && i < 64);
  //printk("%d %X\n",i,tsuartGetSTAT(port));

  if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS) {
    uart_write_wakeup(&port->u);
  }
  if (uart_circ_empty(xmit)) {
    if (port->debugFlags & DEBUG_TXRX) {
      printk("tsuart stopping xmit at end\n");
    }
    tsuart_stop_tx(&port->u);
  }
}

//---------------------------------------------------------------------------
// Begin FPGA dependant code
//---------------------------------------------------------------------------

// accepts the value read from the TS UART STAT register,
// and the encoded baud rate to set, and returns the value to be written
// back to the STAT register.
static inline unsigned encode_BAUD(unsigned reg_value,int bits) {
  return (reg_value & 0xFFFFFF1F) | (bits << 5);
}

// accepts the value read from the TS UART STAT register,
// encodes the given baud rate, and then returns the value to be written
// back to the STAT register.  If the passed baud rate is not supported
// the passed value is returned unchanged.
static unsigned assign_bits_BAUD(unsigned reg_value,int baud_rate) {
  int val;
  switch (baud_rate) {
  case 115200: val = encode_BAUD(reg_value,0); break;
  case 57600:  val = encode_BAUD(reg_value,1); break;
  case 38400:  val = encode_BAUD(reg_value,2); break;
  case 19200:  val = encode_BAUD(reg_value,3); break;
  case 9600:   val = encode_BAUD(reg_value,4); break;
  case 4800:   val = encode_BAUD(reg_value,5); break;
  case 2400:   val = encode_BAUD(reg_value,6); break;
  case 0:      val = encode_BAUD(reg_value,7); break;
  default:     val = reg_value; break;
  }
  return val;
}

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

static inline unsigned tsuart_parity_Rx(struct tsuart_port *port) {
  unsigned rxdata0 = tsuartGetDATA(port);
  unsigned data,rxdata = rxdata0 & port->dataMask;

  //  if (port->localStatus & PORT_STATUS_PAR) {
  data = rxdata & port->dataMask2;
  //    if (port->debugFlags & DEBUG_PARITY) {
  //      printk("Rx parity: %X -> ",rxdata);
  //      printk("%X (%X)\n",data,port->parity[data]);
  //    }
  //  } else {
  //    data = rxdata;
  //  }

  if (!port->parity || (port->parity[data] & port->dataMask) == rxdata) {
    port->localStatus &= ~PORT_STATUS_PARITY;
    return data;
  } else {
    port->localStatus |= PORT_STATUS_PARITY;
    port->u.icount.parity++;
    return data;  // was -1
  }
}
// Check if data is ready to be received from the specified UART
// and if so, read it into the data location provided.
static inline void tsuart_checkRx(struct tsuart_port *port) {
  int stat = tsuartGetSTAT(port);

  if ((port->localStatus & PORT_STATUS_OPEN) || !port->portStatus
      || !(*(port->portStatus) & PORT_STATUS_OPEN)) {
    if (bit_DR(stat)) {
      if (bit_OERR(stat)) {
	port->u.icount.overrun++;
      }
      port->data = tsuart_parity_Rx(port);
    }
  } else {
  }

}


// See if the given UART wants to transmit data, and if so, 
// transmit data if any is ready to be sent.
static inline void tsuart_checkHandleTx(struct tsuart_port *port) {
  int stat;

  // Check the STAT register, and if it says the transmit
  // buffer is empty, then check if we have any data to send.
  // We do it in this order for two reasons. First, because if 
  // the transmit register became empty after our last access 
  // to the UART, we may wind up getting another interrupt for 
  // which we may have nothing to do (except clear the IRQ), so
  // we might as well do that here.  Second, it probably less
  // expensive to check STAT then it is to see if we have more
  // data to send, so if we don't have to do both we might as
  // well not do the more expensive operation.
  
  if (!port->u.state || !port->u.state->port.tty) {
    return;
  }
  stat = tsuartGetSTAT(port);
  if (bit_TBRE(stat)) { // if buffer empty, then try to send
    tsuart_try_handleTx(port);
  }
}

// See if we read any data, and if so, pass it to the upper layers
static inline void tsuart_try_handleRx(struct tsuart_port *port) {
  if (port->data >= 0) {
    tsuart_handleRx(port,port->data);
  }
}

//---------------------------------------------------------------------------
// PARITY
//---------------------------------------------------------------------------

static int calcpar(int val,int parodd) {
    int ones = 0;

    while (val) {
      if (val & 1) {
        ones++;
      }
      val >>= 1;
    }
    return (ones % 2 == parodd) ? 0 : 1;
}

struct parity_cache {
  int dbits,par,parodd,use;
  unsigned *parity;
  struct parity_cache *next;
};

struct parity_cache *parc = 0;

unsigned *alloc_parity_buffer(int dbits,int par,int parodd) {
  struct parity_cache *pc = parc;
  int i,entries;

  while (pc) {
    if (pc->dbits == dbits && pc->par == par && pc->parodd == parodd) {
      pc->use++;
      return pc->parity;
    }
    pc = pc->next;
  }
  entries = 1 << dbits;
  //size = (dbits + par) / 8 + ((dbits + par) % 8) ? 1 : 0;
  pc = kmalloc(sizeof(struct parity_cache),GFP_KERNEL);
  pc->parity = kmalloc(entries * sizeof(unsigned),GFP_KERNEL);
  for (i=0;i<entries;i++) {
    pc->parity[i] = i + (par ? (calcpar(i,parodd) << dbits) : 0) + (1 << (dbits+par));
  }
  pc->dbits = dbits;
  pc->par = par;
  pc->parodd = parodd;
  pc->use = 1;
  pc->next = parc;
  parc = pc;
  return pc->parity;
}

void dealloc_parity_buffer(unsigned *buffer) {
  struct parity_cache *pc = parc,*last = 0;

  if (!buffer) {
    return;
  }
  while (pc) {
    if (pc->parity == buffer) {
      if (--pc->use == 0) {
	if (last) {
	  last->next = pc->next;
	} else {
	  parc = pc->next;
	}
	kfree(pc->parity);
	kfree(pc);
        return; 
      }
    }
    last = pc;
    pc = pc->next;
  }
}

static void initpar(struct tsuart_port *port,int dbits,int par,int parodd,int stop) {

    dealloc_parity_buffer(port->parity);
    /*
    if (par) {
      port->localStatus |= PORT_STATUS_PAR;
    } else {
      port->localStatus &= ~PORT_STATUS_PAR;
    }
    */
    //    printk("initpar: dbits=%d, par=%d\n",dbits,par);
    port->parity = alloc_parity_buffer(dbits,par,parodd);
    //    printk("\n");
}

//---------------------------------------------------------------------------
// TIMERS
//---------------------------------------------------------------------------

static void tsuart_mdm_timer(unsigned long data) {
  struct tsuart_port *root = TS,*next;
  unsigned int status, delta;

  //printk("tsuart_mdm_timer()\n");
  while (root) {
    next = root;
    while (next) {
      status = tsuart_get_mctrl(&next->u);
      delta = status ^ next->old_status;
      next->old_status = status;
      if (delta) {
	if (delta & TIOCM_CAR) {
	  //printk("tsuart_mdm_timer: DCD\n");
	  //uart_handle_dcd_change(&next->u, status & TIOCM_CAR);
	}
	if (delta & TIOCM_DSR) {
	  //printk("tsuart_mdm_timer: DSR\n");
	  next->u.icount.dsr++;
	}
	if (delta & TIOCM_CTS) {
	  //printk("tsuart_mdm_timer: CTS\n");
	  //uart_handle_dcd_change(&next->u, status & TIOCM_CTS);
	}
	//wake_up_interruptible(&next->u.info->delta_msr_wait);
      }
      next = next->next;
    }
    root = root->nextd;
  }
  timer2.expires = jiffies + HZ;
  //add_timer (&timer2);
}

static void tsuart_cts_timer(unsigned long data) {
  struct uart_port *port = (struct uart_port *)data;
  struct tsuart_port *tsport,*next;
  timer_added = 0;
  if (TSPORT(port)->debugFlags & DEBUG_INTERRUPT) {  
    printk("tsuart_cts_timer()\n");
  }
  tsport = TSPORT(port)->root;
  while (tsport) {
    next = tsport->nextd;
    while (tsport) {
      tsuart_checkHandleTx(tsport);
      //if (TSPORT(port)->debugFlags & DEBUG_INTERRUPT) {  
      //printk("cts check: %d\n",tsport->u.line);
      //}
      tsport = tsport->next;
    }
    tsport = next;
  }
}

//---------------------------------------------------------------------------
// LINUX INTERFACE
//---------------------------------------------------------------------------

static void tsuart_stop_tx(struct uart_port *port) {
  if (TSPORT(port)->debugFlags & DEBUG_INTERFACE) {  
    printk("tsuart_stop_tx(%d)\n",port->line);
  }
}

static void tsuart_start_tx(struct uart_port *port) {
  unsigned long flags;

  if (TSPORT(port)->debugFlags & DEBUG_INTERFACE) {  
    printk("tsuart_start_tx(%d)\n",port->line);
  }
  local_irq_save(flags);
  tsuart_checkHandleTx(TSPORT(port));
  local_irq_restore(flags);
}

static void tsuart_stop_rx(struct uart_port *port)
{
  if (TSPORT(port)->debugFlags & DEBUG_INTERFACE) {
    printk("tsuart_stop_rx(%d)\n",port->line);
  }
}

static void tsuart_enable_ms(struct uart_port *port)
{
  if (TSPORT(port)->debugFlags & DEBUG_INTERFACE) {
    printk("tsuart_enable_ms(%d)\n",port->line);
  }
}

static unsigned int tsuart_tx_empty(struct uart_port *port)
{
  int bit = bit_TBRE(tsuartGetSTAT(TSPORT(port)));
  if (TSPORT(port)->debugFlags & DEBUG_INTERFACE) {
    printk("tsuart_tx_empty called(%d) = %d\n",port->line,bit);
  }
  return bit;
}

static unsigned int tsuart_get_mctrl(struct uart_port *port) {
  struct tsuart_port *p = TSPORT(port);
  unsigned int result = 0;

  if (p->mdmctrl) {
    result |= p->mdmctrl(p,0);
  } else {
    result |= TIOCM_CAR;  // pretend DCD is always set
    result |= TIOCM_DSR;  // pretend DSR is always set
  }
  if (invert_cts != bit_CTS(tsuartGetSTAT(TSPORT(port)))) {
    result |= TIOCM_CTS;
  }
  if (TSPORT(port)->debugFlags & DEBUG_INTERFACE) {
    printk("tsuart_get_mctrl(%d) CTS=%d\n",port->line,
	   (result & TIOCM_CTS) ? 1 : 0);
  }
  return result;
}


static void tsuart_set_mctrl(struct uart_port *port, unsigned int mctrl) {
  struct tsuart_port *p = TSPORT(port);

  if (TSPORT(port)->debugFlags & DEBUG_INTERFACE) {
    printk("tsuart_set_mctrl(%d,RTS=%d,invert=%d)\n",port->line,(mctrl&TIOCM_RTS) ? 1:0 ,invert_rts);
  }
  tsuartPutSTAT(p,assign_bit_RTS(tsuartGetSTAT(p),((mctrl&TIOCM_RTS)?1:0)!=invert_rts));
  if (p->mdmctrl) {
    p->mdmctrl(p,0x80000000 | mctrl);
  } else {
    // No support for TIOCM_DTR, so ignore it
  }
}

static void tsuart_break_ctl(struct uart_port *port, int break_state)
{
  // some UARTs do not support BREAK
  if (break_state == -1) break_state = 1;
  tsuartPutSTAT((struct tsuart_port *)port,assign_bit_TXBRK(tsuartGetSTAT((struct tsuart_port *)port),break_state));
  if (TSPORT(port)->debugFlags & DEBUG_INTERFACE) {
    printk("--- tsuart_break_ctl(%d,%d)\n",port->line,break_state);
  }
}

static int tsuart_startup(struct uart_port *port)
{
  if (TSPORT(port)->debugFlags & DEBUG_INTERFACE) {
    printk("tsuart_startup(%d), started=%d\n",port->line,started+1);
  }
  if (TSPORT(port)->portStatus) {
    if (*(TSPORT(port)->portStatus) & PORT_STATUS_OPEN) {
      printk("Port busy, can't open!\n");
      return -EBUSY;
    } else {
      *(TSPORT(port)->portStatus) |= PORT_STATUS_OPEN;
    }
  }
  TSPORT(port)->localStatus |= PORT_STATUS_OPEN;

  setMODE9(port,TSPORT(port)->rxsize > 1 || (TSPORT(port)->localStatus & PORT_STATUS_8PLUS1));
  TSPORT(port)->old_status = tsuart_get_mctrl(port);
  if (started++) { // not needed?
    return 0;
  }

  /* create our timer but don't submit it just yet */
  timer.data = (unsigned long )port;
  timer.function = tsuart_cts_timer;

  if (timer2.function == NULL) {
    if (TSPORT(port)->debugFlags & DEBUG_INTERRUPT) {
      printk("(NOT) Starting MDM timer\n");
    }
    timer2.expires = jiffies + HZ;
    timer2.function = tsuart_mdm_timer;
    init_timer(&timer2);
    //add_timer(&timer2);
  } else {
    printk("MDM timer can't start!\n");
  }
  return 0;
}

static void tsuart_shutdown(struct uart_port *port)
{
  if (TSPORT(port)->debugFlags & DEBUG_INTERFACE) {
    printk("tsuart_shutdown(%d), started=%d,overflows=%d\n",port->line,started-1,port->icount.overrun);
  }
  if (TSPORT(port)->portStatus) {
    if (*(TSPORT(port)->portStatus) & PORT_STATUS_OPEN) {
      *(TSPORT(port)->portStatus) &= ~PORT_STATUS_OPEN;
    } else {
      return; // should not happen???
    }
  }
  TSPORT(port)->localStatus &= ~PORT_STATUS_OPEN;

  if (--started) { // not needed?
    return;
  }
  if (timer_added) {
    del_timer (&timer);
    timer_added = 0;
  }
  if (timer2.function) {
    del_timer(&timer2);
    timer2.function = NULL;
  }
  /*
  tsuartPutSTAT(TSPORT(port),assign_bits_BAUD(tsuartGetSTAT(TSPORT(port)),0));
  port->info->xmit.tail = port->info->xmit.head; // avoid stuckness
  */
  /*
  if (port->info->xmit.tail != port->info->xmit.head) {
    printk("shutdown called with non-empty transmit queue\n");
  }
  */
}

#define TSUART_BAUD_DIVIDE_BY_8 0x2000
/*
uart_get_baud_rate
uart_update_timeout
 */
/*
  7M1 = 8N1 with MSB=1
  7S1 = 8N1 with MSB=0
  8M1 = 8N2

  7: 7N1
  8: 7E1
  8: 7O1
  8: 7N2
  9: 7E2
  9: 7O2
  8: 8N1
  9: 8E1
  9: 8O1
  9: 8N2

  SPACE parity:
  tio.c_cflag |= PARENB | CMSPAR;
  tio.c_cflag &= ~PARODD;
  MARK parity:
  tio.c_cflag |= PARENB | CMSPAR | PARODD;

  - ok, to implement 2 stop bits, what must we do?
  - we already have a special method to implement parity
  - we basically create a lookup table for parity
  - if the index is the value, then the lookup would be the value with parity
  - i forget how the reverse is true; when we receive we must go the other way
  - ok we mask out the parity bit and see if the lookup matches the value we got
  - ok back to stop bits
  - we might have a stop bit that is our 8th bit or that is our 9th bit
  - we might also have parity enabled so long as we don't exceed 9 bits
  - can we handle stop bits the same as parity?
  - it seems like maybe not, since receiving isn't symmetrical
  - in other words, when we send, we have to tack on an extra stop bit
  - however are we going to receive that stop bit???
  - i think if we are in 9-bit mode then we have to!
  - well it certainly couldn't hurt to try!
  - first we need to look at how many data bits are being requested:
  - data bits + parity bits + stop bits
  - the result must be 9 or less.
  - if it is 9, we must set 9-bit mode, otherwise we must set 8-bit mode
  - we must also create a translation table of some kind
    - for example, for 8N1 there are 256 entries corresponding 1:1
    - but for 7N2 there are only 128 entries; the MSB is set on each of them
  - we need to set up some kind of test in order to make sure that we are really doing this right!
    - first, we need to be able to set various termios on both sides (7N1, 7N2, 8N1, 8N2, 8E1, etc.)
    - next we need to send characters back and forth and see if it works
    - can we identify kinds of failure we would expect if we haven't implemented everything right?
      - perhaps we can do this by considering mis-matched termios and what would happen
      - for example, suppose we have 7N2 on one side but not on the other
      - for some termios this would still work
      - would 8N1 work? it would seem like we should rx the MSB set all the time...
      - TEST: 7N2 -> 8N1, we should set the MSB bit set
      - TEST: 8N1 -> 7N2 will probably work even though the stop bit is wrong, because we mask it off

mount -o nolock,vers=2 192.168.0.11:/u/x/home/michael /mnt/host
cd /mnt/host/ct
insmod ./tsuart1.ko
insmod ./tsuart7800.ko
mdev -s
dmesg -c | tail
cd ..
./se-arm5 -d ttts4



cp drivers/serial/tsuart1.ko /u/x/home/michael/ct

  ---
 */
static void tsuart_change_speed(struct uart_port *port, 
				struct ktermios *new_termios, 
				struct ktermios *old_termios)
{
  int encoded_baud,baud,par=0,parodd=0,dbits=8,stop=1;
  int cflag = new_termios->c_cflag;
  int iflag = new_termios->c_iflag;
  int oflag = new_termios->c_oflag;
  int maxflush;

  if (TSPORT(port)->debugFlags & DEBUG_INTERFACE) {
    printk("tsuart_change_speed called,cflag=0x%X,iflag=0x%X,oflag=0x%X\n",cflag,iflag,oflag);
  }

  if (cflag & CRTSCTS) {
    tsuartPutSTAT(TSPORT(port),assign_bit_HWCTS(tsuartGetSTAT(TSPORT(port)),1));
  } else {
    tsuartPutSTAT(TSPORT(port),assign_bit_HWCTS(tsuartGetSTAT(TSPORT(port)),0));
  }

  encoded_baud = cflag & CBAUD;
  switch (encoded_baud) {
  //case B150:    baud = 150; break;
  case B300:    baud = 300; break;
  case B600:    baud = 600; break;
  case B1200:   baud = 1200; break;
  case B2400:   baud = 2400; break;
  case B4800:   baud = 4800; break;
  case B9600:   baud = 9600; break;
  case B19200:  baud = 19200; break;
  case B38400:  baud = 38400; break;
  case B57600:  baud = 57600; break;
  case B115200: baud = 115200; break;
  default: baud=0;
  }
  uart_update_timeout(port,cflag,baud);

  if (baud && (baud >=2400 || (TSPORT(port)->localStatus & PORT_STATUS_SLOWOK))) {
    if (TSPORT(port)->debugFlags & DEBUG_CONTROL) {
      printk("tsuart:change_speed, port %d: %d baud\n",port->line,baud);
    }
    if (baud < 2400) {
      baud = baud * 8;
      tsuartPutSTAT(TSPORT(port),tsuartGetSTAT(TSPORT(port)) | TSUART_BAUD_DIVIDE_BY_8);
    } else { 
      tsuartPutSTAT(TSPORT(port),tsuartGetSTAT(TSPORT(port)) & ~TSUART_BAUD_DIVIDE_BY_8);
    }
    tsuartPutSTAT(TSPORT(port),assign_bits_BAUD(tsuartGetSTAT(TSPORT(port)),baud));
  } else {
    if (TSPORT(port)->debugFlags & DEBUG_CONTROL) {
      printk("tsuart:unknown change_speed, port %d: %X\n",port->line,encoded_baud);
    }
  }

  if (cflag & PARENB) {
    par = 1;
    parodd = (cflag & PARODD) != 0;
  }
  // byte size and parity
  switch (cflag & CSIZE) {
  case CS7:
    dbits = 7;
    break;
  case CS8:
    dbits = 8;
    break;
  default: // not CS8
    printk("tsuart:unsupported byte size, using CS8\n");
    break;
  }
  if (cflag & CSTOPB) {
    stop = 2;
  }
  if (dbits + par + stop - 1 > 9) { // wait, what if we can't do 9-bit mode?
    printk("data bits + parity + extra stop bits = %d > %d; unsupported!\n",
	   dbits+par+stop-1, 9);
    printk("Expect strange things to happen, as we don't know which bits to get rid of...\n");
  }
  TSPORT(port)->dataMask = (1 << (dbits)) - 1;
  TSPORT(port)->dataMask = (1 << (dbits+par)) - 1;
  if((dbits == 7) && (par == 1)) TSPORT(port)->dataMask = 0x7F;
  //7bit parity bug fix...
  if (dbits + par + stop - 1 > 8) {
    TSPORT(port)->localStatus |= PORT_STATUS_8PLUS1;
  } else {
    TSPORT(port)->localStatus &= ~PORT_STATUS_8PLUS1;
  }
  /*
- we have a serious problem with our data masks
- first, we only look at dataMask if we have parity
- otherwise, we just mask against dataMask2, which we always do first in either case
- it looks like 7-bit data mode just doesn't work, because of this!

- when we Tx, we AND our data against dataMask and use that value to look up the value to send
- when we Rx, we AND the received data against dataMask2 to get our received data value, 
  and then if we have parity we AND it against dataMask to look up the value for parity comparison
- if am not sure why we couldn't ALWAYS  AND against dataMask, since we always do the look up
  even if parity isn't enabled!
- in fact PORT_STATUS_PAR is set in our localStatus, even though that is the only place it is used!
- well to be fair, if parity isn't set, we don't actually do a lookup
- but that shouldn't be a problem!
- dataMask is the mask we apply to the value used to look up the value to send
- dataMask2 is the mask we apply based on the number of actual bits of data PLUS PARITY
- so dataMask2=dataMask + parity
- dataMask=dbits
- i'm still not sure what to do here
- before, we weren't creating a parity table if we didn't have parity
- but now, to use the table method, we have to create the table if we have parity or an extra stop bit
- but we cache our parity tables and share them between ports with the same data and parity
- this seems to imply that our parity table should always contain an extra stop bit
- then, the datamask for the particular port would shave off the extra stop bit if it wasn't in use
- also i'm not sure why we populate the parity table outside the allocation function

- ok, we now populate the table in the allocation function
- we always set the extra stop bit and rely on our mask to get rid of it if we don't need it
   */
  /*
  if (TSPORT(port)->rxsize > 1) {
    TSPORT(port)->dataMask = 0x1FF;
  } else if (dbits + par + stop - 1 > 8) {
    TSPORT(port)->localStatus |= PORT_STATUS_8PLUS1;
    TSPORT(port)->dataMask2 = 0x1FF;
  } else {
    TSPORT(port)->localStatus &= ~PORT_STATUS_8PLUS1;
    TSPORT(port)->dataMask2 = 0xFF;
  }
  */
  setMODE9(port,TSPORT(port)->localStatus & PORT_STATUS_8PLUS1);
  //setMODE9(port,TSPORT(port)->rxsize > 1 || (TSPORT(port)->localStatus & PORT_STATUS_8PLUS1));

  //if (TSPORT(port)->rxsize < 2) {
  initpar(TSPORT(port),dbits,par,parodd,stop);  
  //}
  // clear the receive FIFO
  maxflush = 256;
  while (--maxflush>0 && bit_DR(tsuartGetSTAT(TSPORT(port)))) {
    tsuartGetDATA(TSPORT(port));
  }
  if (maxflush < 0) { printk("Warning: Couldn't clear Rx FIFO\n"); }
}

static void tsuart_set_termios(struct uart_port *port,
		struct ktermios *termios, struct ktermios *old_termios)
{
  unsigned long port_flags;

  if (TSPORT(port)->debugFlags & DEBUG_INTERFACE) {
    printk("set_termios called\n");
  }

  spin_lock_irqsave(&port->lock, port_flags);
  tsuart_change_speed(port, termios, old_termios);
  spin_unlock_irqrestore(&port->lock, port_flags);

}

static const char *tsuart_type(struct uart_port *port)
{
  return port->type == PORT_TSUART ? "TSUART" : NULL;
}

/*
 * Release the memory region(s) being used by 'port'
 */
static void tsuart_release_port(struct uart_port *port)
{
  if (TSPORT(port)->debugFlags & DEBUG_INTERFACE) {
    printk("tsuart_release_port(%d)\n",port->line);
  }
  if (port->iotype == SERIAL_IO_MEM) {
    //__iounmap(port->membase);
    __arm_iounmap(port->membase);
    port->membase = 0;
  }
}

/*
 * Request the memory region(s) being used by 'port'
 */
static int tsuart_request_port(struct uart_port *port) {
  int max;
  if (TSPORT(port)->debugFlags & DEBUG_INTERFACE) {
    printk("tsuart_request_port(%d)\n",port->line);
  }
  if (port->iotype == SERIAL_IO_MEM) {
    if (port->membase == 0) {
      port->membase = (void *) ioremap(port->mapbase & 0xFFFFF000, 4096)
	+ (port->mapbase & 0xFFF);
      //printk("port %p:mapbase=%X, membase=%X\n",port,port->mapbase,port->membase);
      if (port->membase == (void *)(port->mapbase & 0xFFF)) {
	port->membase = 0;
	printk(KERN_ERR "tsuart: cannot map io memory\n");
	return -ENOMEM;
      }
    }
  }
  // printk("Clearing RxFIFO\n");
  // clear the receive FIFO
  max = 256;
  while (--max > 0 && bit_DR(tsuartGetSTAT(TSPORT(port)))) tsuartGetDATA(TSPORT(port));
  if (max < 0) { printk("Warning: Couldn't clear Rx FIFO\n"); } //else { printk("max=%d\n",max); }
  return 0;
}

/*
 * Configure/autoconfigure the port.
 */
static void tsuart_config_port(struct uart_port *port, int flags)
{
  if (TSPORT(port)->debugFlags & DEBUG_INTERFACE) {
    printk("tsuart_config_port(%d,%X)\n",port->line,flags);
  }
  if (flags & UART_CONFIG_TYPE) {
    port->type = PORT_TSUART;
    tsuart_request_port(port);
  }
}

/*
 * verify the new serial_struct (for TIOCSSERIAL).
 */
static int tsuart_verify_port(struct uart_port *port, struct serial_struct *ser)
{
  int ret = 0;

  if (port == NULL) return -EINVAL; // avoid warning
  if (ser->type != PORT_UNKNOWN && ser->type != PORT_TSUART)
    ret = -EINVAL;
  if (ser->irq < 0 || ser->irq >= NR_IRQS)
    ret = -EINVAL;
  if (ser->baud_base < 9600) {
    ser->baud_base = 115200; // required to allow ioctl to set low_latency
  }
  if (TSPORT(port)->debugFlags & DEBUG_INTERFACE) {
    printk("tsuart_verify_port(%d,%d)=%d\n",port->line,ser->irq,ret);
  }
  return ret;
}

static int tsuart_ioctls (struct uart_port *port, unsigned int cmd, unsigned long arg)
{
  return -ENOIOCTLCMD;
}


//---------------------------------------------------------------------------
// Interrupt Routine
//---------------------------------------------------------------------------
static irqreturn_t tsuart_interrupt(int irq, void *dev_id) {
  struct tsuart_port *root,*port = (struct tsuart_port *)dev_id;
  unsigned counter=256;

  if (port->debugFlags & DEBUG_INTERRUPT) {  
    printk("tsuart_interrupt()\n");
  }
  port = root = port->root;
  while (port) {
    port->gotBytes = 0;
    port = port->next;
  }
  while (--counter) {
    port = root;
    while (port) {
      port->data = -1;
      tsuart_checkRx(port);
      port = port->next;
    }
    port = root;
    while (port) {
      if (port->data != -1) {
	break;
      }
      port = port->next;
    }
    if (!port) {
      break;
    }
    port = root;
    while (port) {
      tsuart_try_handleRx(port);
      port = port->next;
    }
  }
  port = root;
  while (port) {
    tsuart_checkHandleTx(port);
    if (port->gotBytes) {
      // stats[gotbytes[i]-1]++;
      if (port->debugFlags & DEBUG_DATA) {
	printk("tsuart_interrupt: pushing buffer %d\n",port->u.line);
      }
      //tty_flip_buffer_push(port->u.state->port.tty);
      tty_flip_buffer_push(&port->u.state->port);
    }
    port = port->next;
  }
/*
  if (root->debugFlags & DEBUG_INTERRUPT) {  
    printk("end tsuart_interrupt()\n");
  }
*/
  return IRQ_HANDLED;
}

//---------------------------------------------------------------------------
// CORE LOGIC
//---------------------------------------------------------------------------
static struct uart_ops tsuart_ops = {
	.stop_tx	= tsuart_stop_tx,
	.start_tx	= tsuart_start_tx,
	.stop_rx	= tsuart_stop_rx,
	.enable_ms	= tsuart_enable_ms,
 	.tx_empty	= tsuart_tx_empty,
	.get_mctrl	= tsuart_get_mctrl,
	.set_mctrl	= tsuart_set_mctrl,
	.break_ctl	= tsuart_break_ctl,
	.startup	= tsuart_startup,
	.shutdown	= tsuart_shutdown,
	.set_termios	= tsuart_set_termios,
	.type		= tsuart_type,
	.release_port	= tsuart_release_port,
	.request_port	= tsuart_request_port,
	.config_port	= tsuart_config_port,
	.verify_port	= tsuart_verify_port,
	.ioctl		= tsuart_ioctls,
};

static struct uart_driver tsuart_reg;
struct uart_driver_list *drivers = 0;

int tsuart_configure_driver(struct uart_driver *dr,int major,int minor,
			    char *tty,int maxports) {
  struct uart_driver_list *udl;

  memset(dr,0,sizeof(struct uart_driver));
  dr->owner = THIS_MODULE;
  dr->major = major;
  dr->dev_name = tty;
  dr->minor = minor;
  dr->nr = maxports;
  dr->cons = NULL;
  //  printk("Registering UART driver %p with minor %d\n",dr,dr->minor);
  if (uart_register_driver(dr)) {
    printk("Unable to register driver\n");
    return 0;
  }
  udl = kmalloc(sizeof(struct uart_driver_list),GFP_KERNEL);
  if (!udl) {
    printk("Unable to kmalloc memory for uart driver\n");
    return 0;
  }
  udl->car = dr;
  udl->lines = 0;
  udl->cdr = drivers;
  drivers = udl;
  return 1;
}

static int portsEqual(struct tsuart_port *p1,struct tsuart_port *p2) {
  if (p1->u.iotype != p2->u.iotype) {
    return 0;
  }
  if (p1->u.iotype == SERIAL_IO_PORT) {
    return (p1->u.iobase == p2->u.iobase) && (p1->portStatus!=p2->portStatus);
  } else {
    return p1->u.mapbase == p2->u.mapbase && (p1->portStatus!=p2->portStatus);
  }
  return 1; // unreachable
}

static int portFind(struct tsuart_port *port,portCompF comp) {
  struct tsuart_port *nextd,*next;

  nextd = TS;
  while (nextd) {
    next = nextd;
    while (next) {
      if (comp(next,port)) {
	return 1;
      }
      next = next->next;
    }
    nextd = nextd->nextd;
  }
  return 0;
}

static inline int portExists(struct tsuart_port *port) {
  return portFind(port,portsEqual);
}

static inline int shares(struct tsuart_port *port) {
  return (port->localStatus & PORT_STATUS_SHIRQ) == PORT_STATUS_SHIRQ;
}

static int irqConflict(struct tsuart_port *port) {
  struct tsuart_port *next;

  if (!TS) {
    return 0;
  }
  next = TS;
  while (next) {
    if (next->u.irq == port->u.irq) {
      // allow only if all ports already registered are either
      // 1. the same boardId as the new port or
      // 2. allowing sharing, and the new port allows sharing
      while (next) {
	if (next->boardId == port->boardId) {
	  return 0;
	}
	if (!shares(port) || !shares(next)) {
	  return 1;
	}
	next = next->nextd;
      }
      return 0;
    }
    next = next->next;
  }
  return 0;
}

// Returns TRUE if an IRQ is already allocated for the given port
static int addPort(struct tsuart_port *port) {
  struct tsuart_port *next;

  if (!TS) {
    TS = port;
    port->root = port;
    return 0;
  }
  next = TS;
  while (next) {
    if (next->u.irq == port->u.irq) {
      port->root = next;
      while (next->next) {
	next = next->next;
      }
      next->next = port;
      return 1;
    }
    if (next->nextd) {
      next = next->nextd;
    } else {
      next->nextd = port;
      port->root = port;
      return 0;
    }
  }
  return 0; // unreachable
}

/*
printk("debug: %p %p %p %p %p\n",
               port->dev ? port->dev->bus_id : "",
               port->dev ? ": " : "",
               drv->dev_name, address, uart_type(port));
 */
int tsuart_register_port(struct uart_driver *driver,
				struct tsuart_port *port) {
  struct uart_port *p = &port->u;
  int ret=0;
  struct uart_driver_list *d = drivers;

  if (!driver) {
    driver = &tsuart_reg;
  }
  while (d) {
    if (d->car == driver) {
      break;
    }
    d = d->cdr;
  }
  if (!d) {
    printk("Trying to register port on an unregistered driver\n");
    return -1;
  }
  port->driver = driver;
  if (irqConflict(port)) {
    printk("IRQ in use\n");
    return -1;
  }
  if (portExists(port)) {
    printk("Port already registered\n");
    return -1;
  }
  port->u.ops = &tsuart_ops;
  port->u.line = d->lines;
  if ((ret = uart_add_one_port(driver,p)) == 0) {
    if (port->debugFlags & DEBUG_ENDS) {
      printk("registered port %p\n",port);
    }    
    if (!addPort(port)) {
      ret=request_irq(port->u.irq, tsuart_interrupt, 
		      port->localStatus & PORT_STATUS_SHIRQ ? IRQF_SHARED : 0, 
		      "tsuart_uarts", p);
      if (ret != 0) {
	printk("Unable to allocate irq %d\n",port->u.irq);
      } else {
	if (port->debugFlags & DEBUG_INTERRUPT) {
	  printk("Allocated IRQ %d\n",port->u.irq);
	}
      }
    } else {
	if (port->debugFlags & DEBUG_INTERRUPT) {
	  printk("IRQ %d already allocated\n",port->u.irq);
	}
    }
  } else {
    if (port->debugFlags & DEBUG_ENDS) {
      printk("Failed to register port %p with driver %p, ret=%d\n",port,driver,ret);
    }
    return -1;
  }
  if (ret == 0) {
    d->lines++;
    /*
    if (!tsuart_sanity_check_STAT(port)) {
      printk("WARNING: port %d failed STAT sanity check!\n",port->u.line);
      // Note: This will fail in a number of cases which are fatal:
      // 1. If the FPGA bitstream isn't loaded 
      // 2. We are loaded on hardware that doesn't have a TS-UART present (e.g. wrong arch)
      // It will also fail in a number of cases which are benign:
      // 1. Loopback is installed on the flow control lines (e.g. RTS/CTS)
      // 2. We are a specialized TS-UART that doesn't have all the bits in STAT defined
    }
    */
  }
  port->localStatus &= ~PORT_STATUS_HWCTS;
  tsuartPutSTAT(port,assign_bit_HWCTS(tsuartGetSTAT(port),0));
  if (!bit_HWCTS(tsuartGetSTAT(port))) {
    tsuartPutSTAT(port,assign_bit_HWCTS(tsuartGetSTAT(port),1));
    if (bit_HWCTS(tsuartGetSTAT(port))) {
      port->localStatus |= PORT_STATUS_HWCTS;
    }
  }
  //printk("HWCTS=%d\n",port->localStatus & PORT_STATUS_HWCTS);
  return ret;
}


//---------------------------------------------------------------------------
// KERNEL MODULE
//---------------------------------------------------------------------------

#define SERIAL_TSUART_MAJOR	234	// in unassigned range (231-239)
#define SERIAL_TSUART_MINOR	1
#define TTY_NAME "ttytsuart"

static int __init tsuart_init(void)
{
  tsuart_configure_driver(&tsuart_reg,SERIAL_TSUART_MAJOR,SERIAL_TSUART_MINOR,
                          TTY_NAME,64);
  if (invert_rts_cts) {
    invert_rts = invert_cts = 1;
  }
  return 0;
}

static void __exit tsuart_exit(void)
{
}

module_init(tsuart_init);
module_exit(tsuart_exit);

MODULE_AUTHOR("Technologic Systems");
MODULE_DESCRIPTION("Technologic Systems TS-UART driver " __DATE__ " " __TIME__);
MODULE_LICENSE("GPL");  // or BSD.  Take your pick!

EXPORT_SYMBOL(tsuart_configure_driver);
EXPORT_SYMBOL(tsuart_register_port);
EXPORT_SYMBOL(tsuartGetIO8);
EXPORT_SYMBOL(tsuartGetMEM8);
EXPORT_SYMBOL(tsuartGetMEM16);
EXPORT_SYMBOL(tsuartGetMEM16_9);
EXPORT_SYMBOL(tsuartPutIO8);
EXPORT_SYMBOL(tsuartPutMEM8);
EXPORT_SYMBOL(tsuartPutMEM16);
EXPORT_SYMBOL(tsuartPutMEM16_9);

module_param(invert_rts, uint, 0644);
MODULE_PARM_DESC(invert_rts, "Invert RTS signals");
module_param(invert_cts, uint, 0644);
MODULE_PARM_DESC(invert_cts, "Invert CTS signals");
module_param(invert_rts_cts, uint, 0644);
MODULE_PARM_DESC(invert_cts, "Invert RTS/CTS signals");
