#include <linux/irq.h>
#include <linux/gfp.h>
#include <linux/seq_file.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <asm/mach/arch.h>
#include <asm/hw_irq.h>
#include <mach/addr-map.h>
#include <mach/irqs.h>

#define APMASK_OFFSET   0x9C  /* The APMASK register is in the Marvell's GPIO Control Block */

#define FPGA_REGS_BASE TS47XX_FPGA_VIRT_BASE
#define FPGA_REG(x) (*((volatile u16 *)(FPGA_REGS_BASE + (x))))
#define FPGA_IRQ_OFFSET       0x2C     /* IRQ cause */
#define FPGA_IRQ_MASK_OFFSET  0x2E     /* IRQ mask (1=masked)*/

#define MIN_FPGA_IRQ    64
#define MAX_FPGA_IRQ    (MIN_FPGA_IRQ + 16)
#define IS_BAD_FPGA_IRQ(x) ((x) < MIN_FPGA_IRQ || (x) > MAX_FPGA_IRQ)
#define IS_MASTER_IRQ(x)   ((x) == MAX_FPGA_IRQ)


#define ICU_BASE (AXI_VIRT_BASE + 0x82000)
#define ICU_INT_CONF(x)  (*((volatile u32 *)(ICU_BASE + (x))))


static void mask_muxed_irq(struct irq_data *data);
static void unmask_muxed_irq(struct irq_data *data);

static void mux_irq_handler(unsigned int irq, struct irq_desc *desc)
{
   unsigned int bit, n;
   unsigned short fpga_irq_reg;

   /*
    * When the FPGA asserts its IRQ line, we end up in this handler.  Now we need
    * to read the FPGA's IRQ register to determine what peripheral(s) need to be serviced.
    */

   /*    Read the IRQ register from the FPGA...
    *
    *    BIT 0: XUART            (IRQ #64)
    *    BIT 1: CAN              (IRQ #65)
    *    BIT 2: CAN2             (IRQ #66)
    *    BIT 3: PC/104 IRQ 5     (IRQ #67)
    *    BIT 4: PC/104 IRQ 6     (IRQ #68)
    *    BIT 5: PC/104 IRQ 7     (IRQ #69)
    *    BITS 6-15: Reserved     (IRQ #70-79)
    *
    *    There is an additional "irq", #80, which
    *    doesn't really exist, and can never cause
    *    us to end up in this handler.  We only use
    *    it to globally mask/unmask the FPGA interrupt.
    */

    /* Why do we need to do this before handling interrupts?  */
   ICU_INT_CONF(0xfc) &= ~0xF;  /* Mask the SMC_IRQ */
    
   fpga_irq_reg = FPGA_REG(FPGA_IRQ_OFFSET);
      for(n=0,bit=1; bit <= 0x8000; bit <<= 1, n++)  {
         if(fpga_irq_reg & bit)            
            generic_handle_irq(MIN_FPGA_IRQ + n);         
      }
      
   ICU_INT_CONF(0xfc) |= 1 ; /* Unmask SMC_IRQ */
     
}

/**
 *    When the 'master' irq is unmasked, then we want to
 *    mask the fpga irq.  This is the opposite of
 *    what seems intuitive.  The reason is that when the
 *    /proc/irq/80/irq file is opened, the unmask_muxed_irq()
 *    function will be called, and that's when we need
 *    to disable the fpga, so we mask the pin.  The
 *    converse is also true -- when the file is closed,
 *    the mask_muxed_irq() function is called, so we
 *    then unmask the pin.
 *
 *    In other words:  Opening /proc/irq/80/irq will
 *    cause the irq from the fpga to be masked.
 *    Closing it will cause it to be unmasked again.
 *    The /proc/irq/80/irq file is opened by ts4700ctl
 *    before programming the fpga, and closed afterwards.
 */
static void unmask_muxed_irq(struct irq_data *data)
{
   int bit = data->irq - MIN_FPGA_IRQ;

   if (IS_MASTER_IRQ(data->irq)) {
      ICU_INT_CONF(0xfc) &= ~0xF;    /* Mask SMC_IRQ */

   } else if (! IS_BAD_FPGA_IRQ(data->irq))
      {
         FPGA_REG(FPGA_IRQ_MASK_OFFSET) &= ~BIT(bit);
         ICU_INT_CONF(0xfc) |= 1 ;
      }
}

static void mask_muxed_irq(struct irq_data *data)
{
   int bit = data->irq - MIN_FPGA_IRQ;

   if (IS_MASTER_IRQ(data->irq)) {
         ICU_INT_CONF(0xfc) |= 1 ; /* Unmask SMC_IRQ */
   } else if (! IS_BAD_FPGA_IRQ(data->irq)) {

      FPGA_REG(FPGA_IRQ_MASK_OFFSET) |= BIT(bit);
      if (FPGA_REG(FPGA_IRQ_MASK_OFFSET) == 0xFFFF)
         ICU_INT_CONF(0xfc) &= ~0xF;
   }
}

static void ack_muxed_irq(struct irq_data *data)
{
#if (0)
   /* temp hack for test... */
   int bit = data->irq - MIN_FPGA_IRQ;
   if (IS_MASTER_IRQ(data->irq)) {


   } else if (! IS_BAD_FPGA_IRQ(data->irq)) {
      printk("ack %d\n", data->irq);
      FPGA_REG(0x10) = 0;
   }
#endif
}

static struct irq_chip muxed_irq_chip = {
   .name    = "MUXIRQ",
   .irq_mask    = mask_muxed_irq,
   .irq_unmask  = unmask_muxed_irq,
   .irq_ack     = ack_muxed_irq
};


void __init ts4700_init_mux_irq(void)
{
   int n;

   /*
    * On the TS-47xx, the FPGA interrupt is connected to MFP_27.  This pin must
    * be configured as the SMC_IRQ pin (this is done in ts4700.c -- look for "GPIO27_SMC_IRQ").
    */


   FPGA_REG(FPGA_IRQ_MASK_OFFSET) = 0xFFFF;  /* mask all */

   for(n=MIN_FPGA_IRQ; n <= MAX_FPGA_IRQ; n++)
   {
      irq_set_chip(n, &muxed_irq_chip);
      irq_set_handler(n, handle_level_irq);
      set_irq_flags(n, IRQF_VALID);
   }

   /* Install mux handler */
   irq_set_chained_handler(IRQ_PXA168_SM_INT, mux_irq_handler);
}

