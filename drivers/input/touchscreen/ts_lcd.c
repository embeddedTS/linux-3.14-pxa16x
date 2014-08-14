/*
 * TS-LCD touchscreen controller driver
 *
 * adapted from mk712.c
 * Copyright (c) 1999-2002 Transmeta Corporation
 * Copyright (c) 2005 Rick Koch <n1gp@hotmail.com>
 * Copyright (c) 2005 Vojtech Pavlik <vojtech@suse.cz>
 */

/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

/*
 * This driver supports the TS-LCD
 */

/*
 * 1999-12-18: original version, Daniel Quinlan
 * 1999-12-19: added anti-jitter code, report pen-up events, fixed mk712_poll
 *             to use queue_empty, Nathan Laredo
 * 1999-12-20: improved random point rejection, Nathan Laredo
 * 2000-01-05: checked in new anti-jitter code, changed mouse protocol, fixed
 *             queue code, added module options, other fixes, Daniel Quinlan
 * 2002-03-15: Clean up for kernel merge <alan@redhat.com>
 *             Fixed multi open race, fixed memory checks, fixed resource
 *             allocation, fixed close/powerdown bug, switched to new init
 * 2005-01-18: Ported to 2.6 from 2.4.28, Rick Koch
 * 2005-02-05: Rewritten for the input layer, Vojtech Pavlik
 * 2008-02-05: adapted from mk712.c for the TS-LCD, Michael Schmidt
 *
 */


#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <asm/io.h>

MODULE_AUTHOR("Daniel Quinlan <quinlan@pathname.com>, Vojtech Pavlik <vojtech@suse.cz>");
MODULE_DESCRIPTION("TS-LCD");
MODULE_LICENSE("GPL");

extern int tsGetBaseBoard(void);

static struct input_dev *tslcd_dev;
static DEFINE_SPINLOCK(tslcd_lock);
static struct timer_list timer;
static int timer_started = 0;

static int swapxy =
#ifdef TOUCHSCREEN_TSLCD_SWAPXY
			1;
#else
			0;
#endif
static int negx =
#ifdef TOUCHSCREEN_TSLCD_NEGX
			1;
#else
			0;
#endif
static int negy =
#ifdef TOUCHSCREEN_TSLCD_NEGY
			1;
#else
			0;
#endif


#define TSLCD_NOTOUCH 0xFFFF

static int qX(int XAD,int YAD,int XUL1,int YUL1,int XUR1,int YUR1,int XLL1,int YLL1,int XLR1,int YLR1) { return ((((4096 - XAD)*XUL1)/4096 + (XAD*XUR1)/4096)*(4096 - YAD))/4096 + ((((4096 - XAD)*XLL1)/4096 + (XAD*XLR1)/4096)*YAD)/4096; }
static int qY(int XAD,int YAD,int XUL1,int YUL1,int XUR1,int YUR1,int XLL1,int YLL1,int XLR1,int YLR1) { return (YAD*(((4096 - XAD)*YLL1)/4096 + (XAD*YLR1)/4096))/4096 + ((4096 - YAD)*(((4096 - XAD)*YUL1)/4096 + (XAD*YUR1)/4096))/4096; }

static volatile unsigned short *tX, *tY;
static int hardcode=0;
static unsigned short calib_x0 = 87, calib_dx = 3867; // 3954-87;
static unsigned short calib_y0 = 129, calib_dy = 3848; //3977-129;
static unsigned int
  qcalib_xul = -422, qcalib_yul = -923,
  qcalib_xur = -453, qcalib_yur = 4955,
  qcalib_xll = 4730, qcalib_yll = -943,
  qcalib_xlr = 4722, qcalib_ylr = 5065;
static int use_qcalib = 1;
static int qwide=800,qhigh=480;

static inline unsigned short tslcd_readX(void) {
  short x = swapxy ? *tY : *tX;
  short y = swapxy ? *tX : *tY;

  if (x & 1) {
    x = ((~x) >> 4) & 0xfff;
    y = ((~y) >> 4) & 0xfff;
    if (!use_qcalib) {
      x = (x-calib_x0) * 4096 / calib_dx;
    } else {
      x = qX(x,y,qcalib_xul,qcalib_yul,qcalib_xll,qcalib_yll,
	     qcalib_xur,qcalib_yur,qcalib_xlr,qcalib_ylr);
      x = x - (100 * (x - 2048)) / qwide;
    }

    if (x < 0) x = 0; else if (x > 4095) x = 4095;
    if (negx) x = 4095 - x;
    return x;
  } else {
    return TSLCD_NOTOUCH;
  }
}

static inline unsigned short tslcd_readY(void) {
  short x = swapxy ? *tY : *tX;
  short y = swapxy ? *tX : *tY;

  if (!(x & 1)) return TSLCD_NOTOUCH;
  x = ((~x) >> 4) & 0xfff;
  y = ((~y) >> 4) & 0xfff;
  if (!use_qcalib) {
    y = (y-calib_y0) * 4096 / calib_dy;
  } else {
    y = qY(x,y,qcalib_xul,qcalib_yul,qcalib_xll,qcalib_yll,
             qcalib_xur,qcalib_yur,qcalib_xlr,qcalib_ylr);
    y = y - (100 * (y - 2048)) / qhigh;
  }
  if (y < 0) y = 0; else if (y > 4095) y = 4095;
  if (negy) y = 4095 - y;
  return y;
}

static int DEBOUNCE_MS=9;
static void tslcd_sample(unsigned long data)
{
  static unsigned short last_x = TSLCD_NOTOUCH;
  static unsigned short last_y = 0;
  static unsigned short x,y;
  static int bounce=3;

  spin_lock(&tslcd_lock);
  x = tslcd_readX();
  y = tslcd_readY();

  if (x == last_x && y == last_y) {
    goto sample_done;
  }
  if (x == TSLCD_NOTOUCH) {
    bounce = DEBOUNCE_MS;
    //printk("sample none\n");
    input_report_key(tslcd_dev, BTN_TOUCH, 0);
  } else {
    if (bounce > 0) {
      bounce--;
    } else {
      //printk("ts1=%d,%d\n",x,y);
      input_report_abs(tslcd_dev, ABS_X, x);
      input_report_abs(tslcd_dev, ABS_Y, y);
      input_report_key(tslcd_dev, BTN_TOUCH, 1);
    }
  }
  last_x = x;
  last_y = y;

  input_sync(tslcd_dev);

 sample_done:
  timer.expires = jiffies + 1;
  add_timer(&timer);
  spin_unlock(&tslcd_lock);
}

static int tslcd_open(struct input_dev *dev)
{
  unsigned long flags;

  //printk("tslcd_open\n");
  spin_lock_irqsave(&tslcd_lock, flags);
  if (!timer_started) {
    timer_started++;
    timer.function = tslcd_sample;
    timer.data = 0;
    timer.expires = jiffies + 1;
    init_timer(&timer);
    add_timer(&timer);
  }
  spin_unlock_irqrestore(&tslcd_lock, flags);

  return 0;
}

static void tslcd_close(struct input_dev *dev)
{
  unsigned long flags;

  //printk("tslcd_close\n");
  spin_lock_irqsave(&tslcd_lock, flags);
  if (!--timer_started) {
    del_timer(&timer);
  }
  spin_unlock_irqrestore(&tslcd_lock, flags);

}

static void sspi_cmd(volatile unsigned short *gpio,unsigned int cmd) {
  unsigned int i;

  // pulse CS#
  *gpio = (*gpio & 0xFFF0) | 0x2;
  *gpio = (*gpio & 0xFFF0) | 0x0;

  for (i = 0; i < 32; i++, cmd <<= 1) {
    if (cmd & 0x80) {
      *gpio = (*gpio & 0xFFF0) | 0x4;
      *gpio = (*gpio & 0xFFF0) | 0xc;
    } else {
      *gpio = (*gpio & 0xFFF0) | 0x0;
      *gpio = (*gpio & 0xFFF0) | 0x8;
    }
  }
}

#define CALIB1_OFFSET 7
#define CALIB2_OFFSET 8
static int load_calibration(volatile unsigned short *gpio,
			     unsigned short *x,unsigned short *dx,
			     unsigned short *y,unsigned short *dy,
			     int *xul,int *yul,
			     int *xur,int *yur,
			     int *xll,int *yll,
			     int *xlr,int *ylr) {

  int i,j,ret,n[20],_x,_y,_dx,_dy;
  int _xul,_yul,_xur,_yur,_xll,_yll,_xlr,_ylr;

  sspi_cmd(gpio,0xac); // X_PROGRAM_EN
  sspi_cmd(gpio,0x4e); // READ_TAG

  for (j = 0; j < 20; j++) {
    for (ret = 0x0, i = 0; i < 32; i++) {
      *gpio = (*gpio & 0xFFF0) | 0x0;
      *gpio = (*gpio & 0xFFF0) | 0x8;
      ret = ret << 1 | (*gpio & 0x1);
    }
    n[j] = ret;

  }

  sspi_cmd(gpio,0x78); // PROGRAM_DIS

  *gpio = (*gpio & 0xFFF0) | 0x2;
  _x = n[CALIB1_OFFSET] >> 16;
  _dx = n[CALIB1_OFFSET] & 0xFFFF;
  _y = n[CALIB2_OFFSET] >> 16;
  _dy = n[CALIB2_OFFSET] & 0xFFFF;

  _xul = (short)(n[1+CALIB2_OFFSET] >> 16);
  _yul = (short)(n[1+CALIB2_OFFSET] & 0xFFFF);
  _xur = (short)(n[2+CALIB2_OFFSET] >> 16);
  _yur = (short)(n[2+CALIB2_OFFSET] & 0xFFFF);
  _xll = (short)(n[3+CALIB2_OFFSET] >> 16);
  _yll = (short)(n[3+CALIB2_OFFSET] & 0xFFFF);
  _xlr = (short)(n[4+CALIB2_OFFSET] >> 16);
  _ylr = (short)(n[4+CALIB2_OFFSET] & 0xFFFF);

  if (!(_xul > 0 || _yul > 0 || _xur > 0 || _yur < 0
	|| _xll < 0 || _yll > 0 || _xlr < 0 || _ylr < 0)) {
    *xul = _xul;
    *yul = _yul;
    *xur = _xur;
    *yur = _yur;
    *xll = _xll;
    *yll = _yll;
    *xlr = _xlr;
    *ylr = _ylr;
    return 1;
  }

  if ((_dx < 1 || _dx > 32768) || (_dy < 1 || _dy > 32768)) {
    return 0; // use defaults
  }
  if ((_x < 1 || _x > 32768) || (_y < 1 || _y > 32768)) {
    return 0; // use defaults
  }
  if (_dx == 0) _dx = 4096;
  if (_dy == 0) _dy = 4096;

  *x = _x;
  *y = _y;
  *dx = _dx;
  *dy = _dy;
  return 0;
}




static int __init tslcd_init(void)
{
  int err;
  int gpioBase;
  unsigned int baseboard;
  volatile unsigned short *vreg;

  vreg = ioremap(0x80004000,5*4096);
  if (vreg == 0) {
    printk("tslcd: could not get touchscreen regs\n");
    err = -ENOMEM;
    goto fail1;
  }

   baseboard = tsGetBaseBoard();

  if (baseboard == 10 || baseboard == 17) {    /* TS-8900 or TS-8920 */
     printk("Baseboard: TS-89XX\n");
     gpioBase = 0x4004;
     vreg[1] |= BIT(11);      /* Enable the baseboard clock in the 4700's fpga (at 0x80004002) */
     vreg[2] = 0x321;         /* the bus config register */
     tX = vreg + 0x2040;
     tY = vreg + 0x2041;
  }
  else {

     gpioBase = 0x28;
     vreg[1] |= BIT(14);      /* Enable the Touchscreen core in the fpga (at 0x80004002) */

     tX = vreg + 0x800;
     tY = vreg + 0x801;
  }

  if (!hardcode) {
    use_qcalib = load_calibration(vreg+(gpioBase / sizeof(unsigned short)),
				  &calib_x0,&calib_dx,&calib_y0,&calib_dy,
				  &qcalib_xul, &qcalib_yul,
				  &qcalib_xur, &qcalib_yur,
				  &qcalib_xll, &qcalib_yll,
				  &qcalib_xlr, &qcalib_ylr);
  } else use_qcalib = (hardcode == 2);
  if (use_qcalib) {
    printk("ts_lcd.c: using quadrilateral calibration\n");
    printk("ul=%d,%d ur=%d,%d ll=%d,%d lr=%d,%d\n",
	   qcalib_xul, qcalib_yul,
	   qcalib_xur, qcalib_yur,
	   qcalib_xll, qcalib_yll,
	   qcalib_xlr, qcalib_ylr);
  } else {
    printk("ts_lcd.c: using rectangular calibration\n");
    printk("x calibration: offset=%d, width=%d\n",calib_x0,calib_dx);
    printk("y calibration: offset=%d, width=%d\n",calib_y0,calib_dy);
  }

  tslcd_dev = input_allocate_device();
  if (!tslcd_dev) {
    printk(KERN_ERR "tslcd: not enough memory\n");
    err = -ENOMEM;
    goto fail1;
  }

  tslcd_dev->name = "TS-LCD";
  tslcd_dev->phys = "tslcd/input0";
  //tslcd_dev->id.bustype = BUS_ISA;
  //tslcd_dev->id.vendor  = 0x0005;
  //tslcd_dev->id.product = 0x0001;
  //tslcd_dev->id.version = 0x0100;

  tslcd_dev->open    = tslcd_open;
  tslcd_dev->close   = tslcd_close;


  tslcd_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);

  // tslcd_dev->keybit[(long)BTN_TOUCH] = BIT(BTN_TOUCH);   <<<< this is wrong
  tslcd_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

  input_set_abs_params(tslcd_dev, ABS_X, 0, 4096, 0, 0);
  input_set_abs_params(tslcd_dev, ABS_Y, 0, 4096, 0, 0);

  err = input_register_device(tslcd_dev);
  if (err)
    goto fail2;

  return 0;

 fail1:
  input_free_device(tslcd_dev);
 fail2:
  return err;
}

static void __exit tslcd_exit(void)
{
  input_unregister_device(tslcd_dev);
}

module_init(tslcd_init);
module_exit(tslcd_exit);
module_param(swapxy,int, 0644);
module_param(negx,int, 0644);
module_param(negy,int, 0644);
module_param(DEBOUNCE_MS, int, 0644);
MODULE_PARM_DESC(swapxy, "Swap X/Y axes");
MODULE_PARM_DESC(negx, "Reverse X axis");
MODULE_PARM_DESC(negy, "Reverse Y axis");
MODULE_PARM_DESC(DEBOUNCE_MS, "Number of ticks to debounce the touchscreen for");
module_param(hardcode, int, 0644);
MODULE_PARM_DESC(hardcode, "use hardcoded calibration values (1=rect,2=quad)");

module_param(calib_x0, ushort, 0644);
MODULE_PARM_DESC(calib_x0, "calib_x0");
module_param(calib_dx, ushort, 0644);
MODULE_PARM_DESC(calib_dx, "calib_dx");
module_param(calib_y0, ushort, 0644);
MODULE_PARM_DESC(calib_y0, "calib_y0");
module_param(calib_dy, ushort, 0644);
MODULE_PARM_DESC(calib_dy, "calib_dy");
module_param(qcalib_xul, uint, 0644);
MODULE_PARM_DESC(qcalib_xul,"qcalib_xul");
module_param(qcalib_yul, uint, 0644);
MODULE_PARM_DESC(qcalib_yul,"qcalib_yul");
module_param(qcalib_xur , uint, 0644);
MODULE_PARM_DESC(qcalib_xur ,"qcalib_xur");
module_param(qcalib_yur, uint, 0644);
MODULE_PARM_DESC(qcalib_yur,"qcalib_yur");
module_param(qcalib_xll, uint, 0644);
MODULE_PARM_DESC(qcalib_xll,"qcalib_xll");
module_param(qcalib_yll, uint, 0644);
MODULE_PARM_DESC(qcalib_yll,"qcalib_yll");
module_param(qcalib_xlr, uint, 0644);
MODULE_PARM_DESC(qcalib_xlr,"qcalib_xlr");
module_param(qcalib_ylr, uint, 0644);
MODULE_PARM_DESC(qcalib_ylr,"qcalib_ylr");
