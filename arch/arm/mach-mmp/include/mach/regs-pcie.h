/*
 * linux/arch/arm/mach-mmp/include/mach/regs-pcie.h
 *
 *  PCIe controler registers
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_MACH_REGS_PCIE_H
#define __ASM_MACH_REGS_PCIE_H

#include <mach/addr-map.h>


#define PCIE_REG(x)		((void __iomem *)(PXA168_PCIE_VIRT_BASE + (x)))

/* PCIe Register Offsets */

#define PCIE_DMA_CTRL_CH0  0x0	/* DMA Control */
#define PCIE_DMA_CTRL_CH1  0x80
#define PCIE_DMA_CTRL_CH2  0x100
#define PCIE_DMA_CTRL_CH3  0x180
#define PCIE_DMA_CTRL_BYTE_COUNT(x)  (x && 0xFFF)

#define PCIE_DMA_PCIE_CTRL_CH0 0x4	/* DMA PCIe Control */
#define PCIE_DMA_PCIE_CTRL_CH1 0x84
#define PCIE_DMA_PCIE_CTRL_CH2 0x104
#define PCIE_DMA_PCIE_CTRL_CH3 0x184
#define PCIE_DMA_PCIE_CTRL_PCIE_TYPE_MASK 0x1F
#define PCIE_DMA_PCIE_CTRL_TLP_MEM_RW 0x0
#define PCIE_DMA_PCIE_CTRL_TLP_MEM_READ_LOCK 0x1
#define PCIE_DMA_PCIE_CTRL_TLP_CFG_TYPE0_RW 0x4
#define PCIE_DMA_PCIE_CTRL_TLP_CFG_TYPE1_RW 0x5
#define PCIE_DMA_PCIE_CTRL_PCIETD_OFFSET 7

#define PCIE_DMA_START_CH0 0xC	/* DMA Start */
#define PCIE_DMA_START_CH1 0x8C
#define PCIE_DMA_START_CH2 0x10C
#define PCIE_DMA_START_CH3 0x18C
#define PCIE_DMA_START_TRANSFER 0x1

#define PCIE_DMA_SRC_ADDR_LS_CH0 0x10	/* DMA Channel Source Address upper */
#define PCIE_DMA_SRC_ADDR_LS_CH1 0x90
#define PCIE_DMA_SRC_ADDR_LS_CH2 0x110
#define PCIE_DMA_SRC_ADDR_LS_CH3 0x190

#define PCIE_DMA_SRC_ADDR_MS_CH0 0x14	/* DMA Channel Source Address lower */
#define PCIE_DMA_SRC_ADDR_MS_CH1 0x94
#define PCIE_DMA_SRC_ADDR_MS_CH2 0x114
#define PCIE_DMA_SRC_ADDR_MS_CH3 0x194

#define PCIE_DMA_DEST_ADDR_LS_CH0 0x18	/* DMA Channel Destination Address
					   upper */
#define PCIE_DMA_DEST_ADDR_LS_CH1 0x98
#define PCIE_DMA_DEST_ADDR_LS_CH2 0x118
#define PCIE_DMA_DEST_ADDR_LS_CH3 0x198

#define PCIE_DMA_DEST_ADDR_MS_CH0 0x1C	/* DMA Channel Destination Address
					   lower */
#define PCIE_DMA_DEST_ADDR_MS_CH1 0x9C
#define PCIE_DMA_DEST_ADDR_MS_CH2 0x11C
#define PCIE_DMA_DEST_ADDR_MS_CH3 0x19C

#define PCIE_DMA_NEXT_DESC_CH0 0x20	/* DMA Next Descriptor Pointer */
#define PCIE_DMA_NEXT_DESC_CH1 0xA0
#define PCIE_DMA_NEXT_DESC_CH2 0x120
#define PCIE_DMA_NEXT_DESC_CH3 0x1A0

#define PCIE_DMA_MODE_CH0 0x24	/* DMA mode */
#define PCIE_DMA_MODE_CH1 0xA4
#define PCIE_DMA_MODE_CH2 0x124
#define PCIE_DMA_MODE_CH3 0x1A4
#define PCIE_DMA_MODE_NONCHAINED (0x0)
#define PCIE_DMA_MODE_CHAINED (0x1)
#define PCIE_DMA_MODE_PCIE_READ (0x1 << 1)
#define PCIE_DMA_MODE_PCIE_WRITE (0x0 << 1)

#define PCIE_PIO_RD_DATA_CH0 0x30	/* PIO Read Data */
#define PCIE_PIO_RD_DATA_CH1 0xB0
#define PCIE_PIO_RD_DATA_CH2 0x130
#define PCIE_PIO_RD_DATA_CH3 0x1B0

#define PCIE_PIO_START_CH0 0x34	/* PIO Start */
#define PCIE_PIO_START_CH1 0xB4
#define PCIE_PIO_START_CH2 0x134
#define PCIE_PIO_START_CH3 0x1B4

#define PCIE_PIO_WR_DATA_CH0 0x38	/* PIO Write Data */
#define PCIE_PIO_WR_DATA_CH1 0xB8
#define PCIE_PIO_WR_DATA_CH2 0x138
#define PCIE_PIO_WR_DATA_CH3 0x1B8

#define PCIE_PIO_WR_STRB_CH0 0x3C	/* PIO Write Data Strobes */
#define PCIE_PIO_WR_STRB_CH1 0xBC
#define PCIE_PIO_WR_STRB_CH2 0x13C
#define PCIE_PIO_WR_STRB_CH3 0x1BC

#define PCIE_PIO_ADDR_LS_CH0 0x40	/* PIO address LSBs */
#define PCIE_PIO_ADDR_LS_CH1 0xC0
#define PCIE_PIO_ADDR_LS_CH2 0x140
#define PCIE_PIO_ADDR_LS_CH3 0x1C0

#define PCIE_PIO_ADDR_MS_CH0 0x44	/* PIO address MSBs */
#define PCIE_PIO_ADDR_MS_CH1 0xC4
#define PCIE_PIO_ADDR_MS_CH2 0x144
#define PCIE_PIO_ADDR_MS_CH3 0x1C4

#define PCIE_DMA_RD_RESP_STAT0 0x60	/* DMA Read Response Status */
#define PCIE_DMA_RD_RESP_STAT1 0xE0
#define PCIE_DMA_RD_RESP_STAT2 0x160
#define PCIE_DMA_RD_RESP_STAT3 0x1E0

#define PCIE_DMA_WR_RESP_STAT0 0x64	/* DMA Write Response Status */
#define PCIE_DMA_WR_RESP_STAT1 0xE4
#define PCIE_DMA_WR_RESP_STAT2 0x164
#define PCIE_DMA_WR_RESP_STAT3 0x1E4

#define PCIE_DMA_ISR 0xf80	/* DMA Channel Interrupt Status Register */
#define PCIE_DMA_ISRM 0xf84	/* DMA Interrupt Status Register Mask */
#define PCIE_DMA_AXI_SLVERR_ISR 0xf90	/* DMA Channel AXI Slave Error
					   Interrupt Status Register */
#define PCIE_DMA_AXI_SLVERR_ISRM 0xf94	/* DMA Channel AXI Slave Error
					   Interrupt Status Register Mask */
#define PCIE_DMA_AXI_DECERR_ISR 0xf98	/* DMA Channel AXI Decode Error
					   Interrupt Status Register */
#define PCIE_DMA_AXI_DECERR_ISRM 0xf9c	/* DMA Channel AXI Decode Error
					   Interrupt Status Register Mask */
#define PCIE_DMA_COMPL_SC_ISR 0xfa0	/* DMA Channel PCIe Successful
					   Completion Interrupt Status
					   Register */
#define PCIE_DMA_COMPL_SC_ISRM 0xfa4	/* DMA Channel PCIe Successful
					   Completion Interrupt Status
					   Register Mask */
#define PCIE_DMA_COMPL_UR_ISR 0xfa8	/* DMA Channel PCIe Unsupported
					   Request Completion Interrupt
					   Status Register */
#define PCIE_DMA_COMPL_UR_ISRM 0xfac	/* DMA Channel PCIe Unsupported
					   REquest Completion Interrupt
					   Status Register Mask */
#define PCIE_DMA_COMPL_CR_S_ISR 0xfb0	/* DMA Channel PCIe Configuration
					   Request Retry Completion Interrupt
					   Status Register */
#define PCIE_DMA_COMPL_CR_S_ISRM 0xfb4	/* DMA Channel PCIe Configuration
					   Request Retry Completion Interrupt
					   Status Register Mask */
#define PCIE_DMA_COMPL_CA_ISR 0xfb8	/* DMA Channel PCIe Completer Abort
					   Interrupt Status Register */
#define PCIE_DMA_COMPL_CA_ISRM 0xfbc	/* DMA Channel PCIe Completer Abort
					   Interrupt Status Register Mask */
#define PCIE_MSI 0x1000		/* PCIe Control */
#define PCIE_MSI_ISR_BASE 0x1400	/* MSI ISR -  0x1400, 0x1408...0x1438 */
#define PCIE_MSI_ISRM_BASE 0x1404	/* MSI ISR Mask - 0x1404,
					   0x140C...0x143C */
#define PCIE_ISR0 0x1800	/* PCIe Control ISR 0 register */
#define PCIE_ISRM0 0x1804	/* PCIe Control ISR 0 mask register */
#define PCIE_ISR1 0x1808	/* PCIe Control ISR 1 register */
#define PCIE_ISRM1 0x180C	/* PCIe Control ISR 1 mask register */
#define PCIE_PCI_LEGACY_ISR0 0x1820	/* PCIe legacy interrupt ISR */
#define PCIE_PCI_LEGACY_ISRM0 0x1824	/* PCI legacy interrupt ISR mask */
#define PCIE_CTRL0 0x1840	/* PCIe Control0 register */
#define PCIE_CTRL1 0x1844	/* PCIe Control1 register */
#define PCIE_STAT0 0x1880	/* PCIe Status0 register */
#define PCIE_STAT1 0x1884	/* PCIe Status1 register */

#define PCIE_PHY_CONTROL 0x2000
#define PCIE_PHY_ANALOG_CTRL    (PCIE_PHY_CONTROL + (0x94 << 2))

/* Configuration capability registers */
#define PCIE_STD_PCI_CFG  0x3000
#define PCIE_CFG_PM_CAP   0x3040
#define PCIE_CFG_MSI_CAP  0x3050
#define PCIE_CFG_PCIE_CAP 0x3070
#define PCIE_CFG_MSIX_CAP 0x30B0
#define PCIE_CFG_SLOT_CAP 0x30C0
#define PCIE_CFG_VPD_CAP  0x30D0

#define PCIE_STD_PCI_CFG_REG(x)  ((void __iomem *)(PCIE_REG(PCIE_STD_PCI_CFG) \
						   + (x)))
#define PCIE_CFG_PM_CAP_REG(x)   ((void __iomem *)(PCIE_REG(PCIE_CFG_PM_CAP) \
						   + (x)))
#define PCIE_CFG_MSI_CAP_REG(x)  ((void __iomem *)(PCIE_REG(PCIE_CFG_MSI_CAP) \
						   + (x)))
#define PCIE_CFG_PCIE_CAP_REG(x) ((void __iomem *)(PCIE_REG(PCIE_CFG_PCIE_CAP) \
						   + (x)))
#define PCIE_CFG_MSIX_CAP_REG(x) ((void __iomem *)(PCIE_REG(PCIE_CFG_MSIX_CAP)\
						   + (x)))
#define PCIE_CFG_SLOT_CAP_REG(x) ((void __iomem *)(PCIE_REG(PCIE_CFG_SLOT_CAP)\
						   + (x)))
#define PCIE_CFG_VPD_CAP_REG(x)  ((void __iomem *)(PCIE_REG(PCIE_CFG_VPD_CAP)\
						   + (x)))
#endif				/* __ASM_MACH_REGS_PCIE_H */
