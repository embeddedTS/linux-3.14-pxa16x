/*
 * linux/include/asm-arm/arch-pxa/pxa_u2o.h
 *
 * This supports machine-specific differences in how the PXA
 * USB 2.0 Device Controller (U2O) is wired.
 *
 * (C) Copyright 2008 Marvell International Ltd.
 * All Rights Reserved
 */

#ifndef __ASM_ARCH_PXA_U2O_H
#define __ASM_ARCH_PXA_U2O_H

#include <plat/pxausb_common.h>
#include <mach/regs-usb.h>

#if (0)
/* usb otg controller register base */
#define PXA168_U2O_REGBASE 	(0xd4208000)
#define	PXA168_U2O_PHYBASE	(0xd4207000)

#define PXA935_U2O_REGBASE	(0x55502000)
#define	PXA935_U2O_PHYBASE	(0x5550a000)

#define PXA168_U2H_REGBASE      (0xd4209000)
#define PXA168_U2H_PHYBASE      (0xd4206000)
 
#define USB_REG_RANGE		(0x1ff)
#define	USB_PHY_RANGE		(0xff)

/* registers */
#define U2x_CAPREGS_OFFSET       0x100

#define U2xUSBCMD				(0x140)       /* U2O Command */
#define U2xUSBCMD_RST				(1<<1)      /* Reset */
#define U2xUSBCMD_RS				(1<<0)      /* Run/Stop */

#define U2xUSBSTS				(0x144)       /* U2O Status */
#define U2xUSBINTR				(0x148)       /* U2O Interrupt Enable */

#define U2xPORTSC				(0x184)       /* U2O Port Status */
#define U2xPORTSC_PP                            (1<<12)   		  /* Port Power */
#define U2xPORTSC_PTS_MASK                      (3<<30)   		  /* Parallel Transceiver Select */

#define U2xUSBINTR				(0x148)       /* U2O Interrupt Enable */
#define U2xUSBMODE				(0x1A8)       /* U2O Device Mode */
#define U2xUSBMODE_CM_MASK                      (3<<0)   		  /* U2O Controller Mode */

#endif

#define U2xOTGSC				(0x1A4)       /* U2O On-The-Go Status and Control */

/* OTG interrupt enable bit masks */
#define  U2xOTGSC_DPIE                         (0x40000000)   /* Data-line pulsing IE */
#define  U2xOTGSC_1MSIE                        (0x20000000)   /* 1 Millisecond timer IE */
#define  U2xOTGSC_BSEIE                        (0x10000000)   /* B-session end IE */
#define  U2xOTGSC_BSVIE                        (0x08000000)   /* B-session valid IE */
#define  U2xOTGSC_ASVIE                        (0x04000000)   /* A-session valid IE */
#define  U2xOTGSC_AVVIE                        (0x02000000)   /* A-V-bus valid IE */
#define  U2xOTGSC_IDIE                         (0x01000000)   /* OTG ID IE */
#define  U2xOTGSC_IE_MASK   		       (0x7F000000)

/* OTG interrupt status bit masks */
#define  U2xOTGSC_IS_MASK   (0x007F0000)
#define  U2xOTGSC_DPIS                         (0x00400000)   /* Data-line pulsing IS */
#define  U2xOTGSC_1MSIS                        (0x00200000)   /* 1 Millisecond timer IS */
#define  U2xOTGSC_BSEIS                        (0x00100000)   /* B-session end IS */
#define  U2xOTGSC_BSVIS                        (0x00080000)   /* B-session valid IS */
#define  U2xOTGSC_ASVIS                        (0x00040000)   /* A-session valid IS */
#define  U2xOTGSC_AVVIS                        (0x00020000)   /* A-Vbus valid IS */
#define  U2xOTGSC_IDIS                         (0x00010000)   /* OTG ID IS */

/* OTG status bit masks */
#define  U2xOTGSC_DPS                          (0x00004000)   /* Data-line pulsing */
#define  U2xOTGSC_1MST                         (0x00002000)   /* 1 Milliseconf timer toggle */
#define  U2xOTGSC_BSE                          (0x00001000)   /* B-session end */
#define  U2xOTGSC_BSV                          (0x00000800)   /* B-session valid */
#define  U2xOTGSC_ASV                          (0x00000400)   /* A-session valid */
#define  U2xOTGSC_AVV                          (0x00000200)   /* A-Vbus Valid */
#define  U2xOTGSC_ID                           (0x00000100)   /* OTG ID */

/* OTG control bit masks */
#define  U2xOTGSC_CTL_BITS                     (0x2F)
#define  U2xOTGSC_HABA                         (0x00000080)   /* hardware assisted B-Dis to A-Con */
#define  U2xOTGSC_HADP                         (0x00000040)   /* hardware assisted data pulse bits*/
#define  U2xOTGSC_IDPU                         (0x00000020)   /* ID pull-up enable */
#define  U2xOTGSC_DP                           (0x00000010)   /* Data-pulsing */
#define  U2xOTGSC_OT                           (0x00000008)   /* OTG termination */
#define  U2xOTGSC_HAAR                         (0x00000004)   /* Auto reset bit */
#define  U2xOTGSC_VC                           (0x00000002)   /* Vbus charge */
#define  U2xOTGSC_VD                           (0x00000001)   /* Vbus discharge */

#if (0)

#define UTMI_REVISION		0x0
#define UTMI_CTRL		0x4
#define UTMI_PLL		0x8
#define UTMI_TX			0xc
#define UTMI_RX			0x10
#define UTMI_IVREF		0x14
#define UTMI_T0			0x18
#define UTMI_T1			0x1c
#define UTMI_T2			0x20
#define UTMI_T3			0x24
#define UTMI_T4			0x28
#define UTMI_T5			0x2c
#define UTMI_RESERVE		0x30
#define UTMI_USB_INT		0x34
#define UTMI_DBG_CTL		0x38
#define UTMI_OTG_ADDON		0x3c

/* For UTMICTRL Register */
#define UTMI_CTRL_USB_CLK_EN                    (1<<31)
/* pxa168 */
#define UTMI_CTRL_SUSPEND_SET1                  (1<<30)
#define UTMI_CTRL_SUSPEND_SET2                  (1<<29)
#define UTMI_CTRL_RXBUF_PDWN                    (1<<24)
#define UTMI_CTRL_TXBUF_PDWN                    (1<<11)

#define UTMI_CTRL_INPKT_DELAY_SHIFT             30
#define UTMI_CTRL_INPKT_DELAY_SOF_SHIFT 	28
#define UTMI_CTRL_PU_REF_SHIFT			20
#define UTMI_CTRL_ARC_PULLDN_SHIFT              12
#define UTMI_CTRL_PLL_PWR_UP_SHIFT              1
#define UTMI_CTRL_PWR_UP_SHIFT                  0
/* For UTMI_PLL Register */
#define UTMI_PLL_CLK_BLK_EN_SHIFT               24
#define UTMI_PLL_FBDIV_SHIFT                    4
#define UTMI_PLL_REFDIV_SHIFT                   0
#define UTMI_PLL_FBDIV_MASK                     0x00000FF0
#define UTMI_PLL_REFDIV_MASK                    0x0000000F
#define UTMI_PLL_ICP_MASK                       0x00007000
#define UTMI_PLL_KVCO_MASK                      0x00031000
#define UTMI_PLL_PLLCALI12_SHIFT		29
#define UTMI_PLL_PLLCALI12_MASK			(0x3<<29)
#define UTMI_PLL_PLLVDD18_SHIFT			27
#define UTMI_PLL_PLLVDD18_MASK			(0x3<<27)
#define UTMI_PLL_PLLVDD12_SHIFT			25
#define UTMI_PLL_PLLVDD12_MASK			(0x3<<25)
#define UTMI_PLL_KVCO_SHIFT			15
#define UTMI_PLL_ICP_SHIFT			12
/* For UTMI_TX Register */
#define UTMI_TX_LOW_VDD_EN_SHIFT                11
#define UTMI_TX_IMPCAL_VTH_SHIFT                14
#define UTMI_TX_IMPCAL_VTH_MASK                 (0x7<<14)
#define UTMI_TX_CK60_PHSEL_SHIFT                17
#define UTMI_TX_CK60_PHSEL_MASK                 (0xf<<17)
#define UTMI_TX_TXVDD12_SHIFT                   22
#define UTMI_TX_TXVDD12_MASK                    (0x3<<22)
/* For UTMI_RX Register */
#define UTMI_RX_SQ_THRESH_SHIFT                 4
#define UTMI_RX_SQ_THRESH_MASK                  (0xf<<4)
#define UTMI_REG_SQ_LENGTH_SHIFT                15
#define UTMI_REG_SQ_LENGTH_MASK                 (0x3<<15)

#define REG_RCAL_START                          0x00001000
#define VCOCAL_START                            0x00200000
#define KVCO_EXT                                0x00400000
#define PLL_READY                               0x00800000
#define CLK_BLK_EN                              0x01000000

#define UTMI_OTG_ADDON_OTG_ON			(1<<0)

#endif

#define res_size(res)   ((res)->end - (res)->start + 1)

struct device;
struct otg_transceiver;

struct pxa_usb_plat_info {
	int (*phy_init) (void __iomem * base);
	int (*phy_deinit) (void __iomem * base);
	int (*plat_init) (struct device *dev);
	int (*vbus_set) (int);
	int (*vbus_status) (unsigned base);	/* do we see host? */
	int (*vbus_detect) (void *func, int enable);
	int (*usbid_detect) (struct otg_transceiver *otg);
	int (*set_power) (int);
	int clk_gating;
	int rely_on_vbus;
	int in_single;
	int out_single;
	int is_otg;
	unsigned regbase;
	unsigned phybase;
	void * (*init_pmic_ops) (void);
};

extern struct otg_xceiv_ops *init_pxa9xx_otg_xceiv_ops(void);
extern struct otg_ctrl_ops *init_pxa9xx_otg_ctrl_ops(void);

extern int otg_is_client(void);
extern int otg_is_host(void);


extern unsigned u2o_get(unsigned base, unsigned offset);
extern void u2o_set(unsigned base, unsigned offset, unsigned value);
extern void u2o_clear(unsigned base, unsigned offset, unsigned value);
extern void u2o_write(unsigned base, unsigned offset, unsigned value);

extern int rely_on_vbus;
extern int connected;
#endif /* __ASM_ARCH_PXA_U2O_H */

