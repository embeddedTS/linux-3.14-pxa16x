
//---------------------------------------------------------------------------
// 
//---------------------------------------------------------------------------

#define DEBUG_ENDS 1
#define DEBUG_CONFIG 2
#define DEBUG_PROBE 4
#define DEBUG_INTERFACE 8
#define DEBUG_CONTROL 16
#define DEBUG_INTERRUPT 32
#define DEBUG_DATA 64
#define DEBUG_TXRX 128
#define DEBUG_PARITY 256

#define PORT_STATUS_OPEN 1
#define PORT_STATUS_TXMSB 2
#define PORT_STATUS_PAR 4
#define PORT_STATUS_PARODD 8
#define PORT_STATUS_8PLUS1 16
#define PORT_STATUS_SHIRQ 32
#define PORT_STATUS_SLOWOK 64
#define PORT_STATUS_PARITY 128
#define PORT_STATUS_HWCTS 256

#define DELAY_TIME      HZ / 100        // 100 Hz
#define PORT_TSUART     55              // see include/linux/serial_core.c

#define TSPORT(port) ((struct tsuart_port *)port)

struct uart_driver_list {
  struct uart_driver *car;
  int lines;
  struct uart_driver_list *cdr;  
};

struct tsuart_port {
  struct uart_port u;
  struct tsuart_port *nextd; // next tsuart_port with different IRQ
                             // must be NULL unless root == this
  struct tsuart_port *next;  // next tsuart_port with same IRQ
  struct tsuart_port *root;  // first tsuart_port with same IRQ
  unsigned (*get)(struct tsuart_port *,int); // get register function
  void (*put)(struct tsuart_port *,int,unsigned); // put register function
  int debugFlags;
  int gotBytes;
  int data;
  int *portStatus,pstatus,rxsize,localStatus;
  struct uart_driver *driver;
  unsigned *parity;
  int dataMask,dataMask2;
  int boardId; // gives us a way to associate a port with a board
  int (*mdmctrl)(struct tsuart_port *,int cmd);
  unsigned int old_status;
};

static inline void init_tsuart_port(struct tsuart_port *p) {
  p->u.irq = 0;
  p->u.uartclk = 0; // unused
  p->u.fifosize = 0; // unused
  p->u.ops = 0;
  p->u.flags = ASYNC_BOOT_AUTOCONF;
  p->u.line = 0;
  p->u.iobase = 0;
  p->u.membase = 0;
  p->u.mapbase = 0;
  p->nextd = p->next = p->root = 0;
  p->get = 0;
  p->put = 0;
  p->debugFlags = 0;
  p->gotBytes = 0;
  p->data = 0;
  p->rxsize = 1;
  p->parity = 0;
  p->dataMask = p->dataMask2 = 0xFF;
  p->localStatus = 0;
  p->portStatus = 0;
  p->boardId = 0;
  p->mdmctrl = 0;
}

typedef int (*portCompF)(struct tsuart_port *,struct tsuart_port *);

// accepts the value read from the TS UART STAT register
// returns TRUE if the TBRE bit is set, and FALSE if it is clear
static inline int bit_TBRE(unsigned char reg_value) {
  return (reg_value & 0x01) == 0x01;
}

// accepts the value read from the TS UART STAT register
// returns TRUE if the DR bit is set, and FALSE if it is clear
static inline int bit_DR(unsigned char reg_value) {
  return (reg_value & 0x02) == 0x02;
}

// accepts the value read from the TS UART STAT register
// returns TRUE if the OERR bit is set, and FALSE if it is clear
static inline int bit_OERR(unsigned char reg_value) {
  return (reg_value & 0x04) == 0x04;
}

// accepts the value read from the TS UART STAT register
// returns TRUE if the CTS bit is set, and FALSE if it is clear
static inline int bit_CTS(unsigned char reg_value) {
  return (reg_value & 0x08) == 0x08;
}

static inline unsigned bit_RXBRK(unsigned reg_value) {
  return (reg_value & 0x0200) == 0x0200;
}

// accepts the value read from the TS UART STAT register
// returns TRUE if the RTS bit is set, and FALSE if it is clear
static inline int bit_RTS(unsigned char reg_value) {
  return (reg_value & 0x10) == 0x10;
}

// accepts the value read from the TS UART STAT register
// If set is TRUE, sets the RTS bit, others clears it,
// and returns the new value to be written back to the STAT register
static inline unsigned assign_bit_RTS(unsigned reg_value,int set) {
  return (reg_value & ~0x10) | (set ? 0x10 : 0);
}

static inline unsigned assign_bit_TXBRK(unsigned reg_value,int set) {
  return (reg_value & ~0x400) | (set ? 0x400 : 0);
}

static inline int bit_TRE(unsigned short reg_value) {
  return (reg_value & 0x1000) == 0x1000;
}

static inline unsigned short assign_bit_MODE9(unsigned reg_value,int set)
{
  return (reg_value & ~0x100) | (set ? 0x100 : 0);
}

static inline unsigned short assign_bit_HWCTS(unsigned reg_value,int set)
{
  return (reg_value & ~0x4000) | (set ? 0x4000 : 0);
}

static inline unsigned short bit_HWCTS(unsigned reg_value)
{
  return (reg_value & 0x4000) == 0x4000;
}

MODULE_AUTHOR("embeddedTS");
MODULE_LICENSE("GPL");  // or BSD.  Take your pick!
static int portDebug=0;
module_param(portDebug,int,0644);

extern int tsuart_register_port(struct uart_driver *,struct tsuart_port *);
extern int tsuart_unregister_port(struct tsuart_port *);
extern unsigned tsuartGetIO8(struct tsuart_port *,int offset);
extern void tsuartPutIO8(struct tsuart_port *,int offset,unsigned value);
extern unsigned tsuartGetMEM8(struct tsuart_port *port,int offset);
extern unsigned tsuartGetMEM16(struct tsuart_port *port,int offset);
extern unsigned tsuartGetMEM16_9(struct tsuart_port *port,int offset);
extern void tsuartPutMEM8(struct tsuart_port *port,int offset,unsigned value);
extern void tsuartPutMEM16(struct tsuart_port *port,int offset,unsigned value);
extern void tsuartPutMEM16_9(struct tsuart_port *port,int offset,unsigned value);
extern int tsuart_configure_driver(struct uart_driver *dr,int major,int minor,
				   char *tty,int maxports);

