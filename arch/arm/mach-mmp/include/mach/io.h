/*
 * linux/arch/arm/mach-mmp/include/mach/io.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_MACH_IO_H
#define __ASM_MACH_IO_H

#include <mach/hardware.h>
#include <mach/pxa168_pcie.h>
#include <linux/mm.h>
#include <asm/pgtable.h>

#define IO_SPACE_LIMIT 0xffffffff

/*
 * PXA168 does not allow direct mapping of PCIe memory onto
 * CPU memory space. Therefore, all CPU access occur indirectly
 * via DMA or PIO channels specially designed to allow single
 * byte/word/dword read/write.
 *
 */
#if defined(CONFIG_PCI)

#define readl_relaxed(addr)	readl(addr)
#define writel_relaxed(v,c)	__raw_writel((__force u32) cpu_to_le32(v),c)

#define __pxa168_mem_pci(a)      ((u32)a)
#define __mem_ptrio(a) ((void __iomem *)a)
#define __mem_ptr8(a) ((u8 *)a)
#define __mem_ptr16(a) ((u16 *)a)
#define __mem_ptr32(a) ((u32 *)a)

/*
 * We , we simply return the actual PCI
 * address and our read/write implementation use that to drive the
 * access registers. If something outside of PCI is ioremap'd, we
 * fallback to the default.
 */
static inline void __iomem *
__pxa168_ioremap(phys_addr_t addr, size_t size, unsigned int mtype, void *caller)
{   
	if (addr > PXA168_PCIE_MAX_MEM)
		return __arm_ioremap_caller(addr, size, mtype, caller);

	return (void __iomem *)addr;
}

static inline void
__pxa168_iounmap(void __iomem *addr)
{
	if ((__force u32)addr >= VMALLOC_START)
		__iounmap(addr);
}

#define __arch_ioremap(a, s, f)		__pxa168_ioremap(a, s, f)
#define	__arch_iounmap(a)		__pxa168_iounmap(a)

#define	writeb(v, p)			__pxa168_writeb(v, __mem_ptrio(p))
#define	writew(v, p)			__pxa168_writew(v, __mem_ptrio(p))
#define	writel(v, p)			__pxa168_writel(v, __mem_ptrio(p))

#define	writesb(p, v, l)		__pxa168_writesb(p, __mem_ptr8(v), l)
#define	writesw(p, v, l)		__pxa168_writesw(p, __mem_ptr16(v), l)
#define	writesl(p, v, l)		__pxa168_writesl(p, __mem_ptr32(v), l)

#define	readb(p)			__pxa168_readb(__mem_ptrio(p))
#define	readw(p)			__pxa168_readw(__mem_ptrio(p))
#define	readl(p)			__pxa168_readl(__mem_ptrio(p))

#define	readsb(p, v, l)			__pxa168_readsb(p, __mem_ptr8(v), l)
#define	readsw(p, v, l)			__pxa168_readsw(p, __mem_ptr16(v), l)
#define	readsl(p, v, l)			__pxa168_readsl(p, __mem_ptr32(v), l)

static inline void
__pxa168_writeb(u8 value, void __iomem *p)
{
	u32 addr = (u32)p;

	if (addr >= VMALLOC_START) {
		__raw_writeb(value, (void*)addr);
		return;
	}

	pxa168_pcie_write8(value, addr);
}

static inline void
__pxa168_writesb(void __iomem *bus_addr, const u8 *vaddr, int count)
{
	while (count--)
		writeb(*vaddr++, bus_addr);
}

static inline void
__pxa168_writew(u16 value, void __iomem *p)
{
	u32 addr = (u32)p;

	if (addr >= VMALLOC_START) {
		__raw_writew(value, addr);
		return;
	}

	pxa168_pcie_write16(value, addr);
}


static inline void
__pxa168_writesw(void __iomem *bus_addr, const u16 *vaddr, int count)
{
	while (count--)
		writew(*vaddr++, bus_addr);
}

static inline void
__pxa168_writel(u32 value, void __iomem *p)
{
	u32 addr = (__force u32)p;

	if (addr >= VMALLOC_START) {
		__raw_writel(value, p);
		return;
	}

	pxa168_pcie_write32(value, addr);
}

static inline void
__pxa168_writesl(void __iomem *bus_addr, const u32 *vaddr, int count)
{
	while (count--)
		writel(*vaddr++, bus_addr);
}

static inline unsigned char
__pxa168_readb(const void __iomem *p)
{
	u32 addr = (u32)p;

	if (addr >= VMALLOC_START)
		return __raw_readb((void*)addr);

	return pxa168_pcie_read8(addr);
}

static inline void
__pxa168_readsb(const void __iomem *bus_addr, u8 *vaddr, u32 count)
{
	while (count--)
		*vaddr++ = readb(bus_addr);
}

static inline unsigned short
__pxa168_readw(const void __iomem *p)
{
	u32 addr = (u32)p;

	if (addr >= VMALLOC_START)
		return __raw_readw(addr);

	return pxa168_pcie_read16(addr);
}

static inline void
__pxa168_readsw(const void __iomem *bus_addr, u16 *vaddr, u32 count)
{
	while (count--)
		*vaddr++ = readw(bus_addr);
}

static inline unsigned int
__pxa168_readl(const void __iomem *p)
{
	u32 addr = (__force u32)p;

	if (addr >= VMALLOC_START)
		return __raw_readl(p);
	
	return pxa168_pcie_read32(addr);
}

static inline void
__pxa168_readsl(const void __iomem *bus_addr, u32 *vaddr, u32 count)
{
	while (count--)
		*vaddr++ = readl(bus_addr);
}


/*
 * We can use the built-in functions b/c they end up calling writeb/readb
 */
#define memset_io(c, v, l)		_memset_io((c), (v), (l))
#define memcpy_fromio(a, c, l)		_memcpy_fromio((a), (c), (l))
#define memcpy_toio(c, a, l)		_memcpy_toio((c), (a), (l))

/*
 * PXA168 does not have a transparent cpu -> PCIe I/O translation
 * window.  Instead, it has a set of PIOs that can be used to
 * to access a PCIe address.  This means that we need to override
 * the default I/O functions.
 */
#define	outb(p, v)			__pxa168_outb(p, v)
#define	outw(p, v)			__pxa168_outw(p, v)
#define	outl(p, v)			__pxa168_outl(p, v)

#define	outsb(p, v, l)			__pxa168_outsb(p, v, l)
#define	outsw(p, v, l)			__pxa168_outsw(p, v, l)
#define	outsl(p, v, l)			__pxa168_outsl(p, v, l)

#define	inb(p)				__pxa168_inb(p)
#define	inw(p)				__pxa168_inw(p)
#define	inl(p)				__pxa168_inl(p)

#define	insb(p, v, l)			__pxa168_insb(p, v, l)
#define	insw(p, v, l)			__pxa168_insw(p, v, l)
#define	insl(p, v, l)			__pxa168_insl(p, v, l)


static inline void
__pxa168_outb(u8 value, u32 addr)
{
	if (addr >= VMALLOC_START) {
		__raw_writeb(value, (void*)addr);
		return;
	}

	pxa168_pcie_write8(value, addr);
}

static inline void
__pxa168_outsb(u32 io_addr, const u8 *vaddr, u32 count)
{
	while (count--)
		outb(*vaddr++, io_addr);
}

static inline void
__pxa168_outw(u16 value, u32 addr)
{
	if (addr >= VMALLOC_START) {
		__raw_writew(value, addr);
		return;
	}

	pxa168_pcie_write16(value, addr);
}

static inline void
__pxa168_outsw(u32 io_addr, const u16 *vaddr, u32 count)
{
	while (count--)
		outw(cpu_to_le16(*vaddr++), io_addr);
}

static inline void
__pxa168_outl(u32 value, u32 addr)
{
	if (addr >= VMALLOC_START) {
		__raw_writel(value, (void*)addr);
		return;
	}

	pxa168_pcie_write32(value, addr);
}

static inline void
__pxa168_outsl(u32 io_addr, const u32 *vaddr, u32 count)
{
	while (count--)
		outl(*vaddr++, io_addr);
}

static inline u8
__pxa168_inb(u32 addr)
{
	void __iomem *p = (void __iomem *)addr;

	if (addr >= VMALLOC_START)
		return __raw_readb(p);

	return pxa168_pcie_read8(addr);
}

static inline void
__pxa168_insb(u32 io_addr, u8 *vaddr, u32 count)
{
	while (count--)
		*vaddr++ = inb(io_addr);
}

static inline u16
__pxa168_inw(u32 addr)
{
	void __iomem *p = (void __iomem *)addr;

	if (addr >= VMALLOC_START)
		return __raw_readw(p);

	return pxa168_pcie_read16(addr);
}

static inline void
__pxa168_insw(u32 io_addr, u16 *vaddr, u32 count)
{
	while (count--)
		*vaddr++ = le16_to_cpu(inw(io_addr));
}

static inline u32
__pxa168_inl(u32 addr)
{
	void __iomem *p = (void __iomem *)addr;

	if (addr >= VMALLOC_START)
		return __raw_readl(p);

	return pxa168_pcie_read32(addr);
}

static inline void
__pxa168_insl(u32 io_addr, u32 *vaddr, u32 count)
{
	while (count--)
		*vaddr++ = inl(io_addr);
}

static inline unsigned int
__pxa168_ioread8(const void __iomem *addr)
{
	unsigned long port = (unsigned long __force)addr;

	return	(unsigned int)__pxa168_inb(port);
}

static inline void
__pxa168_ioread8_rep(const void __iomem *addr, void *vaddr, u32 count)
{
	unsigned long port = (unsigned long __force)addr;

	__pxa168_insb(port, vaddr, count);
}

static inline unsigned int
__pxa168_ioread16(const void __iomem *addr)
{
	unsigned long port = (unsigned long __force)addr;

	return	(unsigned int)__pxa168_inw(port);
}

static inline void
__pxa168_ioread16_rep(const void __iomem *addr, void *vaddr, u32 count)
{
	unsigned long port = (unsigned long __force)addr;

	__pxa168_insw(port, vaddr, count);
}

static inline unsigned int
__pxa168_ioread32(const void __iomem *addr)
{
	unsigned long port = (unsigned long __force)addr;

	return	(unsigned int)__pxa168_inl(port);
}

static inline void
__pxa168_ioread32_rep(const void __iomem *addr, void *vaddr, u32 count)
{
	unsigned long port = (unsigned long __force)addr;

	__pxa168_insl(port, vaddr, count);
}

static inline void
__pxa168_iowrite8(u8 value, void __iomem *addr)
{
	unsigned long port = (unsigned long __force)addr;

	__pxa168_outb(value, port);
}

static inline void
__pxa168_iowrite8_rep(void __iomem *addr, const void *vaddr, u32 count)
{
	unsigned long port = (unsigned long __force)addr;

	__pxa168_outsb(port, vaddr, count);
}

static inline void
__pxa168_iowrite16(u16 value, void __iomem *addr)
{
	unsigned long port = (unsigned long __force)addr;

	__pxa168_outw(value, port);
}

static inline void
__pxa168_iowrite16_rep(void __iomem *addr, const void *vaddr, u32 count)
{
	unsigned long port = (unsigned long __force)addr;

	__pxa168_outsw(port, vaddr, count);
}

static inline void
__pxa168_iowrite32(u32 value, void __iomem *addr)
{
	unsigned long port = (unsigned long __force)addr;

	__pxa168_outl(value, port);
}

static inline void
__pxa168_iowrite32_rep(void __iomem *addr, const void *vaddr, u32 count)
{
	unsigned long port = (unsigned long __force)addr;

	__pxa168_outsl(port, vaddr, count);
}

#define	ioread8(p)		 __pxa168_ioread8(__mem_ptrio(p))
#define	ioread16(p)		 __pxa168_ioread16(__mem_ptrio(p))
#define	ioread32(p)		 __pxa168_ioread32(__mem_ptrio(p))

#define	ioread8_rep(p, v, c)	 __pxa168_ioread8_rep(__mem_ptrio(p), v, c)
#define	ioread16_rep(p, v, c)	 __pxa168_ioread16_rep(__mem_ptrio(p), v, c)
#define	ioread32_rep(p, v, c)	 __pxa168_ioread32_rep(__mem_ptrio(p), v, c)

#define	iowrite8(v, p)		 __pxa168_iowrite8(v, __mem_ptrio(p))
#define	iowrite16(v, p)		 __pxa168_iowrite16(v, __mem_ptrio(p))
#define	iowrite32(v, p)		 __pxa168_iowrite32(v, __mem_ptrio(p))

#define	iowrite8_rep(p, v, c)    __pxa168_iowrite8_rep(__mem_ptrio(p), v, c)
#define	iowrite16_rep(p, v, c)	 __pxa168_iowrite16_rep(__mem_ptrio(p), v, c)
#define	iowrite32_rep(p, v, c)	 __pxa168_iowrite32_rep(__mem_ptrio(p), v, c)

#define	ioport_map(port, nr)	 (__mem_ptrio(port))
#define	ioport_unmap(addr)

#else	/* CONFIG_PCI */

#define	__io(a)		__typesafe_io(a)
#define __mem_pci(a)	(a)

#endif /* ! CONFIG_PCI */


#endif /* __ASM_MACH_IO_H */
