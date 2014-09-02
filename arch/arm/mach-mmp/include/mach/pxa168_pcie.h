/*
 *pxa168 pcie platform device data definition file.
 */
#ifndef __LINUX_PXA168_PCIE_H
#define __LINUX_PXA168_PCIE_H

struct pxa168_pcie_platform_data {
	int (*init) (void);
};

extern u8 pxa168_pcie_read8(u32 addr);
extern u16 pxa168_pcie_read16(u32 addr);
extern u32 pxa168_pcie_read32(u32 addr);
extern void pxa168_pcie_write8(u8 val, u32 addr);
extern void pxa168_pcie_write16(u16 val, u32 addr);
extern void pxa168_pcie_write32(u32 val, u32 addr);

#endif				/* __LINUX_PXA168_PCIE_H */
