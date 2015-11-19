/*
 * driver/net/pxa168_eth.c
 *
 * Base on driver/net/mv643xx_eth.c
 *
 * Copyright (C) 2009 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#include <linux/dma-mapping.h>
#include <linux/in.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/udp.h>
#include <linux/etherdevice.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/ethtool.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/workqueue.h>
#include <linux/clk-provider.h>
#include <linux/clk.h>
#include <linux/phy.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <asm/pgtable.h>
#include <asm/cacheflush.h>
#include <linux/pxa168_eth.h>
#include <mach/addr-map.h>

int nophy = 0;

#define NAPI_PMR

#if defined(MFU_TIMER) && defined(NAPI_PMR)
#error only MFU_TIMER or NAPI_PMR can be defined
#endif

#define DRIVER_NAME 	"pxa168-eth"
#define DRIVER_VERSION 	"0.2"

/*
 * Registers
 */
#define PHY_ADDRESS		0x0000
#define SMI			0x0010
#define PORT_CONFIG		0x0400
#define PORT_CONFIG_EXT		0x0408
#define PORT_COMMAND		0x0410
#define PORT_STATUS		0x0418
#define HTPR			0x0428
#define SDMA_CONFIG		0x0440
#define SDMA_CMD		0x0448
#define INT_CAUSE		0x0450
#define INT_W_CLEAR		0x0454
#define INT_MASK		0x0458
#define ETH_F_RX_DESC_0		0x0480
#define ETH_C_RX_DESC_0		0x04A0
#define ETH_C_TX_DESC_1		0x04E4

/* smi register */
#define SMI_BUSY		(1<<28)	/* 0 - Write, 1 - Read	*/
#define SMI_R_VALID		(1<<27)	/* 0 - Write, 1 - Read	*/
#define SMI_OP_W		(0<<26)		/* Write operation	*/
#define SMI_OP_R		(1<<26)	/* Read operation */

#define PHY_WAIT_ITERATIONS	1000	/* 1000 iterations * 10uS = 10mS max */
#define PHY_WAIT_MICRO_SECONDS	10

/* RX & TX descriptor command */
#define BUF_OWNED_BY_DMA	(1<<31)

/* RX descriptor status */
#define RX_EN_INT		(1<<23)
#define RX_FIRST_DESC		(1<<17)
#define RX_LAST_DESC		(1<<16)
#define RX_ERROR		(1<<15)

/* TX descriptor command */
#define TX_EN_INT		(1<<23)
#define TX_GEN_CRC		(1<<22)
#define TX_ZERO_PADDING		(1<<18)
#define TX_FIRST_DESC		(1<<17)
#define TX_LAST_DESC		(1<<16)
#define TX_ERROR		(1<<15)


/* SDMA_CMD */
#define SDMA_CMD_AT 		(1<<31)
#define SDMA_CMD_TXDL		(1<<24)
#define SDMA_CMD_TXDH 		(1<<23)
#define SDMA_CMD_AR 		(1<<15)
#define SDMA_CMD_ERD 		(1<<7)


/* Bit definitions of the Port Config Reg */
#define PCR_HD			(1<<15)
#define PCR_HS			(1<<12)
#define PCR_EN	 		(1<<7)
#define PCR_PM	 		(1<<0)

/* Bit definitions of the Port Config Extend Reg */
#define PCXR_2BSM   		(1<<28)
#define PCXR_DSCP_EN  		(1<<21)
#define PCXR_MFL_1518		(0<<14)
#define PCXR_MFL_1536 		(1<<14)
#define PCXR_MFL_2048		(2<<14)
#define PCXR_MFL_64K		(3<<14)
#define PCXR_FLP		(1<<11)
#define PCXR_FORCE_100M_FD	(3<<18|0xb<<9)
#define PCXR_PRIO_TX_OFF	3
#define PCXR_TX_HIGH_PRI	(7<<PCXR_PRIO_TX_OFF)

/* Bit definitions of the SDMA Config Reg */
#define SDCR_BSZ_OFF		12
#define SDCR_BSZ8 		(3<<SDCR_BSZ_OFF)
#define SDCR_BSZ4		(2<<SDCR_BSZ_OFF)
#define SDCR_BSZ2 		(1<<SDCR_BSZ_OFF)
#define SDCR_BSZ1 		(0<<SDCR_BSZ_OFF)
#define SDCR_BLMR		(1<<6)
#define SDCR_BLMT		(1<<7)
#define SDCR_RIFB		(1<<9)
#define SDCR_RC_OFF		2
#define SDCR_RC_MAX_RETRANS	(0xf << SDCR_RC_OFF)

/*
 * Bit definitions of the Interrupt Cause Reg
 * and Interrupt MASK Reg is the same
 */
#define ICR_RXBUF		(1<<0)
#define ICR_TXBUF_H 		(1<<2)
#define ICR_TXBUF_L 		(1<<3)
#define ICR_TXEND_H 		(1<<6)
#define ICR_TXEND_L 		(1<<7)
#define ICR_RXERR		(1<<8)
#define ICR_TXERR_H 		(1<<10)
#define ICR_TXERR_L 		(1<<11)
#define ICR_TX_UDR 		(1<<13)
#define ICR_MII_CH 		(1<<28)

#define ALL_INTS (ICR_TXBUF_H  | ICR_TXBUF_L  | ICR_TX_UDR |\
				ICR_TXERR_H  | ICR_TXERR_L |\
				ICR_TXEND_H  | ICR_TXEND_L |\
				ICR_RXBUF | ICR_RXERR  | ICR_MII_CH)


#define ETH_HW_IP_ALIGN		2		/* hw aligns IP header */
#define ETH_EXTRA_HEADER	(6+6+2+4)  /* dest+src addr+protocol id+crc */
#define MAX_PKT_SIZE		1536


#define NUM_RX_DESCS		64
#define NUM_TX_DESCS		64
#define TX_DESC_COUNT_LOW	(NUM_TX_DESCS - 60)
#define TX_DESC_COUNT_HIGH	(NUM_TX_DESCS - 4)

#define HASH_ADD		0
#define HASH_DELETE		1
#define HASH_ADDR_TABLE_SIZE	0x4000 /* 16K (1/2K address - PCR_HS == 1) */
#define HOP_NUMBER		12

/* typedefs */

struct rx_desc {
	volatile u32 cmd_sts;	/* Descriptor command status	*/
	u16 byte_cnt;		/* Descriptor buffer byte count		*/
	u16 buf_size;		/* Buffer size				*/
	u32 buf_ptr;		/* Descriptor buffer pointer		*/
	u32 next_desc_ptr;	/* Next descriptor pointer		*/
};

struct tx_desc {
	volatile u32 cmd_sts;	/* Command/status field		*/
	u16 reserved;
	u16 byte_cnt;		/* buffer byte count			*/
	u32 buf_ptr;		/* pointer to buffer for this descriptor*/
	u32 next_desc_ptr;	/* Pointer to next descriptor		*/
};

struct pxa168_private {
	int port_num;			/* User Ethernet port number	*/

	int rx_resource_err;		/* Rx ring resource error flag */

	/* Tx/Rx rings managment indexes fields. For driver use */

	/* Next available and first returning Rx resource */
	int rx_curr_desc_q, rx_used_desc_q;

	/* Next available and first returning Tx resource */
	int tx_curr_desc_q, tx_used_desc_q;

	struct rx_desc *p_rx_desc_area;
	dma_addr_t rx_desc_dma;
	int rx_desc_area_size;
	struct sk_buff **rx_skb;

	struct tx_desc *p_tx_desc_area;
	dma_addr_t tx_desc_dma;
	int tx_desc_area_size;
	struct sk_buff **tx_skb;

	struct work_struct tx_timeout_task;

	struct net_device *dev;
	struct napi_struct napi;
	struct net_device_stats stats;
	spinlock_t lock;
	/* Size of Tx Ring per queue */
	int tx_ring_size;
	/* Number of tx descriptors in use */
	int tx_desc_count;
	/* Size of Rx Ring per queue */
	int rx_ring_size;
	/* Number of rx descriptors in use */
	int rx_desc_count;

	/*
	 * Used in case RX Ring is empty, which can be caused when
	 * system does not have resources (skb's)
	 */
	struct timer_list timeout;
	struct mii_if_info mii;
	u16		phy_addr;

	/* clock */
	struct clk *clk;
	struct pxa168_eth_platform_data *pd;
	/*
	 * Ethernet controller base address.
	 */
	void __iomem *base;

#ifdef MFU_TIMER
	struct timer_list	*mfu_timer;
#endif

	u8	*htpr;			/* hash pointer */
	dma_addr_t htpr_dma;
};

typedef struct {
	u32 lo;
	u32 hi;
} addr_table_entry_t;

/* Bit fields of a Hash Table Entry */
enum hash_table_entry {
	hteValid = 1,
	hteSkip = 2,
	hteRD = 4,
	hteRDBit = 2
};

static char pxa168_mac_str[] = {0x00,0x09,0x11,0x22,0x33,0x45};

//static char MarvellOUI[3] = {0x00, 0x09, 0x11};
static char TSOUI[4] = {0x00, 0xd0, 0x69, 0xf7};


static int pxa168_eth_open(struct net_device *dev);
static int pxa168_eth_stop(struct net_device *dev);

static int ethernet_phy_setup(struct net_device *dev);

static inline u32 rdl(struct pxa168_private *mp, int offset)
{
	return readl(mp->base + offset);
}

static inline void wrl(struct pxa168_private *mp, int offset, u32 data)
{
	writel(data, mp->base + offset);
}

static void abortDMA(struct pxa168_private *mp)
{
	int delay;
	int maxRetries = 40;

	do {
		wrl(mp, SDMA_CMD, SDMA_CMD_AR | SDMA_CMD_AT);
		udelay(100);

		delay = 10;
		while ((rdl(mp, SDMA_CMD) & (SDMA_CMD_AR | SDMA_CMD_AT))
		&& delay-- > 0) {
			udelay(10);
		}
	} while (maxRetries-- > 0 && delay <= 0);

	if (maxRetries <= 0)
		printk(KERN_ERR "%s : DMA Stuck\n", __func__);
}


static int ethernet_phy_get(struct pxa168_private *mp)
{
	unsigned int reg_data;

	/* only support 3 ports */
	BUG_ON(mp->port_num > 2);

	reg_data = rdl(mp, PHY_ADDRESS);

	return (reg_data >> (5 * mp->port_num)) & 0x1f;
}


static int eth_port_read_smi_reg(struct pxa168_private *mp,
		unsigned int phy_reg, unsigned int *value)
{
	int phy_addr = ethernet_phy_get(mp);
	unsigned int val;
	int i = 0;

	/* wait for the SMI register to become available */
	for (i = 0; (val = rdl(mp, SMI)) & SMI_BUSY; i++) {

		if (i == PHY_WAIT_ITERATIONS) {
			printk(KERN_ERR "pxa168 PHY timeout, port %d, val=0x%x\n",
				mp->port_num, val);
			goto out;
		}
		udelay(PHY_WAIT_MICRO_SECONDS);
	}

	wrl(mp, SMI, (phy_addr << 16) | (phy_reg << 21) | SMI_OP_R);

	/* now wait for the data to be valid */
	for (i = 0; !((val = rdl(mp, SMI)) & SMI_R_VALID); i++) {
		if (i == PHY_WAIT_ITERATIONS) {
			printk(KERN_ERR "pxa168 PHY RD timeout, port %d, val=0x%x\n",
				mp->port_num, val);
			goto out;
		}
		udelay(PHY_WAIT_MICRO_SECONDS);
	}
	*value = val & 0xffff;

	return 0;
out:
	return -1;
}


static void eth_port_write_smi_reg(struct pxa168_private *mp,
		unsigned int phy_reg, unsigned int value)
{
	int phy_addr;
	int i;

	phy_addr = ethernet_phy_get(mp);

	/* the SMI register is a shared resource */

	/* wait for the SMI register to become available */
	for (i = 0; rdl(mp, SMI) & SMI_BUSY; i++) {
		if (i == PHY_WAIT_ITERATIONS) {
			printk(KERN_ERR "pxa168 PHY busy timeout, port %d\n",
				mp->port_num);
			goto out;
		}
		udelay(PHY_WAIT_MICRO_SECONDS);
	}

	wrl(mp, SMI, (phy_addr << 16) | (phy_reg << 21) |
			SMI_OP_W | (value & 0xffff));
out:
	return;
}

static void ethernet_phy_set(struct pxa168_private *mp)
{
	u32 reg_data;
	u16 phy_addr = mp->phy_addr;
	int addr_shift = 5 * mp->port_num;

	if(nophy) return;

	/* only support 3 ports */
	BUG_ON(mp->port_num > 2);

	reg_data = rdl(mp, PHY_ADDRESS);
	reg_data &= ~(0x1f << addr_shift);
	reg_data |= (phy_addr & 0x1f) << addr_shift;
	wrl(mp, PHY_ADDRESS, reg_data);
}

static void ethernet_phy_reset(struct pxa168_private *mp)
{
	unsigned int phy_reg_data;

	if(nophy) return;

	/* Reset the PHY */
	eth_port_read_smi_reg(mp, 0, &phy_reg_data);
	phy_reg_data |= 0x8000;	/* Set bit 15 to reset the PHY */
	eth_port_write_smi_reg(mp, 0, phy_reg_data);

	/* wait for PHY to come out of reset */
	do {
		udelay(1);
		eth_port_read_smi_reg(mp, 0, &phy_reg_data);
	} while (phy_reg_data & 0x8000);
}

static int ethernet_phy_detect(struct pxa168_private *mp)
{
	unsigned int val, tmp, mii_status;
	int addr;

	if(nophy) return 0;

	for (addr = 0; addr < 32; addr++) {

		mp->mii.phy_id = ethernet_phy_get(mp);

		if (eth_port_read_smi_reg(mp, MII_BMSR, &mii_status) != 0) {
			/* try next phy */
			mp->phy_addr = (mp->phy_addr + 1) % 32;
			ethernet_phy_set(mp);
			continue;
		}

		/* invalid MII status. More validation required here... */
		if (mii_status == 0 || mii_status == 0xffff) {
			/* try next phy */
			mp->phy_addr = (mp->phy_addr + 1) % 32;
			ethernet_phy_set(mp);
			continue;
		}

		if (eth_port_read_smi_reg(mp, MII_PHYSID1, &tmp) != 0) {
			/* try next phy */
			mp->phy_addr = (mp->phy_addr + 1) % 32;
			ethernet_phy_set(mp);
			continue;
		}

		val = tmp << 16;
		if (eth_port_read_smi_reg(mp, MII_PHYSID2, &tmp) != 0) {
			/* try next phy */
			mp->phy_addr = (mp->phy_addr + 1) % 32;
			ethernet_phy_set(mp);
			continue;
		}

		val |= tmp;

		if ((val & 0xfffffff0) != 0) {
			printk(KERN_INFO "PHY found at addr %x\n",
				mp->phy_addr);
			return 0;
		}

		mp->phy_addr = (mp->phy_addr + 1) % 32;
		ethernet_phy_set(mp);
	}

	return -1;
}

static void rxq_refill(struct net_device *dev)
{
	struct pxa168_private *mp = netdev_priv(dev);
	struct sk_buff *skb;
	struct rx_desc *p_used_rx_desc;
	int used_rx_desc;	/* Where to return Rx resource */
	unsigned long flags;

	while (mp->rx_desc_count < mp->rx_ring_size) {

		skb = dev_alloc_skb(MAX_PKT_SIZE + ETH_HW_IP_ALIGN);
		if (!skb)
			break;

		mp->rx_desc_count++;

		spin_lock_irqsave(&mp->lock, flags);

		/* Get 'used' Rx descriptor */
		used_rx_desc = mp->rx_used_desc_q;
		p_used_rx_desc = &mp->p_rx_desc_area[used_rx_desc];

		p_used_rx_desc->buf_ptr = dma_map_single(NULL,
				skb->data,
				MAX_PKT_SIZE + ETH_HW_IP_ALIGN,
				DMA_FROM_DEVICE);

		p_used_rx_desc->buf_size = MAX_PKT_SIZE + ETH_HW_IP_ALIGN;
		mp->rx_skb[used_rx_desc] = skb;

		/* Return the descriptor to DMA ownership */
		wmb();
		p_used_rx_desc->cmd_sts = BUF_OWNED_BY_DMA | RX_EN_INT;
		wmb();

		/* Move the used descriptor pointer to the next descriptor */
		mp->rx_used_desc_q = (used_rx_desc + 1) % mp->rx_ring_size;

		/* Any Rx return cancels the Rx resource error status */
		mp->rx_resource_err = 0;

		spin_unlock_irqrestore(&mp->lock, flags);

		skb_reserve(skb, ETH_HW_IP_ALIGN);
	}

	/*
	 * If RX ring is empty of SKB, set a timer to try allocating
	 * again at a later time.
	 */
	if (mp->rx_desc_count == 0) {
		printk(KERN_INFO "%s: Rx ring is empty\n", dev->name);
		mp->timeout.expires = jiffies + (HZ / 10);	/* 100 mSec */
		add_timer(&mp->timeout);
	}
}

/*
 * rxq_refill_timer_wrapper
 *
 * Timer routine to wake up RX queue filling task. This function is
 * used only in case the RX queue is empty, and all alloc_skb has
 * failed (due to out of memory event).
 *
 * Input :	pointer to ethernet interface network device structure
 * Output :	N/A
 */
static inline void rxq_refill_timer_wrapper(unsigned long data)
{
	rxq_refill((struct net_device *)data);
}
static inline u32 nibble_swapping_32_bit(u32 x)
{
	return (((x) & 0xf0f0f0f0) >> 4) | (((x) & 0x0f0f0f0f) << 4);
}

static inline u32 nibble_swapping_16_bit(u32 x)
{
	return (((x) & 0x0000f0f0) >> 4) | (((x) & 0x00000f0f) << 4);
}

static inline u32 flip_4_bits(u32 x)
{
	return (((x) & 0x01) << 3) | (((x) & 0x002) << 1)
				| (((x) & 0x04) >> 1) | (((x) & 0x008) >> 3);
}

/*
 * ----------------------------------------------------------------------------
 * This function will calculate the hash function of the address.
 * depends on the hash mode and hash size.
 * Inputs
 * macH             - the 2 most significant bytes of the MAC address.
 * macL             - the 4 least significant bytes of the MAC address.
 * Outputs
 * return the calculated entry.
 */
static u32 hash_function(u32 macH, u32 macL)
{
	u32 hashResult;
	u32 addrH;
	u32 addrL;
	u32 addr0;
	u32 addr1;
	u32 addr2;
	u32 addr3;
	u32 addrHSwapped;
	u32 addrLSwapped;

	addrH = nibble_swapping_16_bit(macH);
	addrL = nibble_swapping_32_bit(macL);

	addrHSwapped = flip_4_bits(addrH & 0xf)
	    + ((flip_4_bits((addrH >> 4) & 0xf)) << 4)
	    + ((flip_4_bits((addrH >> 8) & 0xf)) << 8)
	    + ((flip_4_bits((addrH >> 12) & 0xf)) << 12);

	addrLSwapped = flip_4_bits(addrL & 0xf)
	    + ((flip_4_bits((addrL >> 4) & 0xf)) << 4)
	    + ((flip_4_bits((addrL >> 8) & 0xf)) << 8)
	    + ((flip_4_bits((addrL >> 12) & 0xf)) << 12)
	    + ((flip_4_bits((addrL >> 16) & 0xf)) << 16)
	    + ((flip_4_bits((addrL >> 20) & 0xf)) << 20)
	    + ((flip_4_bits((addrL >> 24) & 0xf)) << 24)
	    + ((flip_4_bits((addrL >> 28) & 0xf)) << 28);

	addrH = addrHSwapped;
	addrL = addrLSwapped;

	addr0 = (addrL >> 2) & 0x03f;
	addr1 = (addrL & 0x003) | ((addrL >> 8) & 0x7f) << 2;
	addr2 = (addrL >> 15) & 0x1ff;
	addr3 = ((addrL >> 24) & 0x0ff) | ((addrH & 1) << 8);

	hashResult = (addr0 << 9) | (addr1 ^ addr2 ^ addr3);
	hashResult = hashResult & 0x07ff;
	return hashResult;
}

/*
 * ----------------------------------------------------------------------------
 * This function will add an entry to the address table.
 * depends on the hash mode and hash size that was initialized.
 * Inputs
 * mp - ETHERNET .
 * macH - the 2 most significant bytes of the MAC address.
 * macL - the 4 least significant bytes of the MAC address.
 * skip - if 1, skip this address.
 * rd   - the RD field in the address table.
 * Outputs
 * address table entry is added.
 * 0 if success.
 * -ENOSPC if table full
 */
static int add_del_hash_entry(struct pxa168_private *mp, u32 macH,
			u32 macL, u32 rd,
			u32 skip, int del)
{
	addr_table_entry_t *entry, *start;
	u32 newHi;
	u32 newLo;
	u32 i;
	u8	*last;

	newLo = (((macH >> 4) & 0xf) << 15)
			| (((macH >> 0) & 0xf) << 11)
			| (((macH >> 12) & 0xf) << 7)
			| (((macH >> 8) & 0xf) << 3)
			| (((macL >> 20) & 0x1) << 31)
			| (((macL >> 16) & 0xf) << 27)
			| (((macL >> 28) & 0xf) << 23)
			| (((macL >> 24) & 0xf) << 19)
			| (skip << hteSkip) | (rd << hteRDBit)
			| hteValid;

	newHi = (((macL >> 4) & 0xf) << 15)
			| (((macL >> 0) & 0xf) << 11)
			| (((macL >> 12) & 0xf) << 7)
			| (((macL >> 8) & 0xf) << 3)
			| (((macL >> 21) & 0x7) << 0);

	/*
	 * Pick the appropriate table, start scanning for free/reusable
	 * entries at the index obtained by hashing the specified MAC address
	 */
	start = (addr_table_entry_t *)(mp->htpr);
	entry = start + hash_function(macH, macL);
	for (i = 0; i < HOP_NUMBER; i++) {
		if (!(entry->lo & hteValid)) {
			break;
		} else {
			/* if same address put in same position */
			if (((entry->lo & 0xfffffff8) == (newLo & 0xfffffff8))
			    && (entry->hi == newHi)) {
				break;
			}
		}
		if (entry == start + 0x7ff)
			entry = start;
		else
			entry++;
	}

	if (((entry->lo & 0xfffffff8) != (newLo & 0xfffffff8)) &&
			(entry->hi != newHi) && del)
		return 0;

	if (i == HOP_NUMBER) {
		if (!del) {
			printk(KERN_INFO "%s: table section is full\n",
				__FILE__);
			return -ENOSPC;
		} else {
			return 0;
		}
	}

	/*
	 * Update the selected entry
	 */
	if (del) {
		entry->hi = 0;
		entry->lo = 0;
	} else {
		entry->hi = newHi;
		entry->lo = newLo;
	}

	last = (u8 *) entry;
	last = last + sizeof(*entry);

	return 0;
}

/*
 * ----------------------------------------------------------------------------
 *  Create an addressTable entry from MAC address info
 *  found in the specifed net_device struct
 *
 *  Input : pointer to ethernet interface network device structure
 *  Output : N/A
 */
static void update_hash_table_mac_address(struct pxa168_private *mp,
		u8 *oaddr, u8 *addr)
{
	u32 macH;
	u32 macL;

	/* Delete old entry */
	if (oaddr) {
		macH = (oaddr[0] << 8) | oaddr[1];
		macL = (oaddr[2] << 24) | (oaddr[3] << 16) |
			(oaddr[4] << 8) | oaddr[5];
		add_del_hash_entry(mp, macH, macL, 1, 0, HASH_DELETE);
	}

	/* Add new entry */
	macH = (addr[0] << 8) | addr[1];
	macL = (addr[2] << 24) | (addr[3] << 16) | (addr[4] << 8) | addr[5];
	add_del_hash_entry(mp, macH, macL, 1, 0, HASH_ADD);
}

/* Address Table functions */
static int init_hashtable(struct pxa168_private *mp)
{
	u8 *addr;
	dma_addr_t reg_dma;

	if (mp->htpr == NULL) {
			mp->htpr = dma_alloc_coherent(NULL,
				HASH_ADDR_TABLE_SIZE + 7,
				&mp->htpr_dma,
				GFP_KERNEL);
		if (mp->htpr == NULL)
			return -ENOMEM;
	}

	/* align to 8 byte boundary */
	addr = (u8 *)(((u32)mp->htpr+7) & ~0x7);
	reg_dma = (dma_addr_t)(((u32)mp->htpr_dma+7) & ~0x7);

	memset(addr, 0, HASH_ADDR_TABLE_SIZE);

	wrl(mp, HTPR, reg_dma);
	return 0;
}

static void pxa168_eth_set_rx_mode(struct net_device *dev)
{
	struct pxa168_private *mp = netdev_priv(dev);
	struct netdev_hw_addr *ha;
	u32 val;

	val = rdl(mp, PORT_CONFIG);
	if (dev->flags & IFF_PROMISC)
		val |= PCR_PM;
	else
		val &= ~PCR_PM;
	wrl(mp, PORT_CONFIG, val);

	netdev_for_each_mc_addr(ha, dev)
		update_hash_table_mac_address(mp, NULL, ha->addr);
}

/*
 * pxa168_eth_set_mac_address
 *
 * Change the interface's mac address.
 * No special hardware thing should be done because interface is always
 * put in promiscuous mode.
 *
 * Input :	pointer to ethernet interface network device structure and
 *		a pointer to the designated entry to be added to the cache.
 * Output :	zero upon success, negative upon failure
 */
static int pxa168_eth_set_mac_address(struct net_device *dev, void *addr)
{
	struct pxa168_private *mp = netdev_priv(dev);
	int i;
	unsigned char oldMac[6];

	memcpy(oldMac, dev->dev_addr, 6);
	for (i = 0; i < 6; i++)
		/* +2 is for the offset of the HW addr type */
		dev->dev_addr[i] = ((unsigned char *)addr)[i + 2];

	update_hash_table_mac_address(mp, oldMac, dev->dev_addr);

	return 0;
}

#ifdef ETH_DUMP_REGS
static void eth_dump_regs(struct pxa168_private *mp)
{
	unsigned int i = 0;

	printk(KERN_INFO "offset: 0x%x, value: 0x%x\n",
		PHY_ADDRESS, rdl(mp, PHY_ADDRESS));
	printk(KERN_INFO "offset: 0x%x, value: 0x%x\n",
		SMI, rdl(mp, SMI));

	for (i = 0x400; i <= 0x4e4; i += 4)
		printk(KERN_INFO "offset: 0x%x, value: 0x%x\n",
			i, rdl(mp, i));
}
#endif

static void eth_port_start(struct net_device *dev)
{
	unsigned int val = 0;
	struct pxa168_private *mp = netdev_priv(dev);
	int tx_curr_desc, rx_curr_desc;

	/* Assignment of Tx CTRP of given queue */
	tx_curr_desc = mp->tx_curr_desc_q;
	wrl(mp, ETH_C_TX_DESC_1,
		(u32)((struct tx_desc *)mp->tx_desc_dma + tx_curr_desc));

	/* Assignment of Rx CRDP of given queue */
	rx_curr_desc = mp->rx_curr_desc_q;
	wrl(mp, ETH_C_RX_DESC_0,
		(u32)((struct rx_desc *)mp->rx_desc_dma + rx_curr_desc));

	wrl(mp, ETH_F_RX_DESC_0,
		(u32)((struct rx_desc *)mp->rx_desc_dma + rx_curr_desc));

	/* Clear all interrupts */
	wrl(mp, INT_CAUSE, 0);

#ifdef MFU_TIMER
	/* no ints if polling */
	wrl(mp, INT_MASK, 0);
#else
	/* Enable all interrupts for receive, transmit and error. */
	wrl(mp, INT_MASK, ALL_INTS);
#endif

	val = rdl(mp, PORT_CONFIG);
	val |= PCR_EN;
	wrl(mp, PORT_CONFIG, val);

	/* Start RX DMA engine */
	val = rdl(mp, SDMA_CMD);
	val |= SDMA_CMD_ERD;
	wrl(mp, SDMA_CMD, val);

#ifdef ETH_DUMP_REGS
	eth_dump_regs(mp);
#endif
}

static void eth_port_reset(struct net_device *dev)
{
	struct pxa168_private *mp = netdev_priv(dev);
	unsigned int val = 0;

	/* Stop RX DMA */
	val = rdl(mp, SDMA_CMD);
	val &= ~SDMA_CMD_ERD;		/* abort dma command */

	/* Abort any transmit and receive operations and put DMA
	 * in idle state.
	 */

	abortDMA(mp);

	/* Stop all interrupts for receive, transmit and error. */
	wrl(mp, INT_MASK, 0);

	/* Clear all interrupts */
	wrl(mp, INT_CAUSE, 0);

	/* Disable port */
	val = rdl(mp, PORT_CONFIG);
	val &= ~PCR_EN ;
	wrl(mp, PORT_CONFIG, val);

}


/**
 * txq_reclaim - Free the tx desc data for completed descriptors
 *
 * If force is non-zero, frees uncompleted descriptors as well
 */
int txq_reclaim(struct net_device *dev, int force)
{
	struct pxa168_private *mp = netdev_priv(dev);
	struct tx_desc *p_tx_desc;
	u32 cmd_sts;
	struct sk_buff *skb;
	unsigned long flags;
	int tx_index;
	dma_addr_t addr;
	int count;
	int released = 0;

	while (mp->tx_desc_count > 0) {
		spin_lock_irqsave(&mp->lock, flags);

		/* tx_desc_count might have changed before acquiring the lock */
		if (mp->tx_desc_count <= 0) {
			spin_unlock_irqrestore(&mp->lock, flags);
			return released;
		}

		tx_index = mp->tx_used_desc_q;
		p_tx_desc = &mp->p_tx_desc_area[tx_index];
		cmd_sts = p_tx_desc->cmd_sts;

		if (!force && (cmd_sts & BUF_OWNED_BY_DMA)) {
			spin_unlock_irqrestore(&mp->lock, flags);
			if (released > 0)
				return released;
			else
				return -1;
		}

		mp->tx_used_desc_q = (tx_index + 1) % mp->tx_ring_size;
		mp->tx_desc_count--;

		addr = p_tx_desc->buf_ptr;
		count = p_tx_desc->byte_cnt;
		skb = mp->tx_skb[tx_index];
		if (skb)
			mp->tx_skb[tx_index] = NULL;

		if (cmd_sts & TX_ERROR) {
			printk(KERN_ERR "%s: Error in TX\n", dev->name);
			dev->stats.tx_errors++;
		}

		spin_unlock_irqrestore(&mp->lock, flags);

		dma_unmap_single(NULL, addr, count, DMA_TO_DEVICE);

		if (skb) {
			dev_kfree_skb_irq(skb);
			pr_debug("dev_kfree tx_descs\n");
		}

		released = 1;
	}

	return released;
}

static void pxa168_eth_tx_timeout(struct net_device *dev)
{
	struct pxa168_private *mp = netdev_priv(dev);

	printk(KERN_INFO "%s: TX timeout  desc_count %d\n",
		dev->name, mp->tx_desc_count);

	/* Do the reset outside of interrupt context */
	schedule_work(&mp->tx_timeout_task);
}

static void pxa168_eth_tx_timeout_task(struct work_struct *work)
{
	struct pxa168_private *mp =
		container_of(work,
			struct pxa168_private, tx_timeout_task);
	struct net_device *dev = mp->dev;

	pxa168_eth_stop(dev);
	pxa168_eth_open(dev);
}

static int rxq_process(struct net_device *dev, int budget)
{
	struct pxa168_private *mp = netdev_priv(dev);
	struct net_device_stats *stats = &dev->stats;
	unsigned int received_packets = 0;
	struct sk_buff *skb;

	while (budget-- > 0) {

		int rx_next_curr_desc, rx_curr_desc, rx_used_desc;
		volatile struct rx_desc *rx_desc;
		unsigned int cmd_sts;
		unsigned long flags;

		/* Do not process Rx ring in case of Rx ring resource error */
		if (mp->rx_resource_err)
			break;

		spin_lock_irqsave(&mp->lock, flags);

		/* Get the Rx Desc ring 'curr and 'used' indexes */
		rx_curr_desc = mp->rx_curr_desc_q;
		rx_used_desc = mp->rx_used_desc_q;

		rx_desc = &mp->p_rx_desc_area[rx_curr_desc];

		cmd_sts = rx_desc->cmd_sts;
		rmb();

		/* Nothing to receive... */

		if (cmd_sts & (BUF_OWNED_BY_DMA)) {
			spin_unlock_irqrestore(&mp->lock, flags);
			break;
		}

		skb = mp->rx_skb[rx_curr_desc];
		mp->rx_skb[rx_curr_desc] = NULL;

		/* Update current index in data structure */
		rx_next_curr_desc = (rx_curr_desc + 1) % mp->rx_ring_size;
		mp->rx_curr_desc_q = rx_next_curr_desc;

		/* Rx descriptors exhausted. */
		/* Set the Rx ring resource error flag */
		if (rx_next_curr_desc == rx_used_desc)
			mp->rx_resource_err = 1;

		mp->rx_desc_count--;
		spin_unlock_irqrestore(&mp->lock, flags);
		dma_unmap_single(NULL, rx_desc->buf_ptr,
			MAX_PKT_SIZE + ETH_HW_IP_ALIGN, DMA_FROM_DEVICE);
		received_packets++;

		/*
		 * Update statistics.
		 * Note byte count includes 4 byte CRC count
		 */
		stats->rx_packets++;
		stats->rx_bytes += rx_desc->byte_cnt;
		/*
		 * In case received a packet without first / last bits on OR
		 * the error summary bit is on, the packets needs to be dropeed.
		 */
		if (((cmd_sts & (RX_FIRST_DESC | RX_LAST_DESC)) !=
			(RX_FIRST_DESC | RX_LAST_DESC))
		|| (cmd_sts & RX_ERROR)) {

			stats->rx_dropped++;
			if ((cmd_sts & (RX_FIRST_DESC | RX_LAST_DESC)) !=
				(RX_FIRST_DESC | RX_LAST_DESC)) {
				if (net_ratelimit())
					printk(KERN_ERR
						"%s: Rx pkt on multiple desc\n",
						dev->name);
			}
			if (cmd_sts & RX_ERROR)
				stats->rx_errors++;

			dev_kfree_skb_irq(skb);

		} else {
			/*
			 * The -4 is for the CRC in the trailer of the
			 * received packet
			 */

			skb_put(skb, rx_desc->byte_cnt - 4);
			skb->protocol = eth_type_trans(skb, dev);
			netif_rx(skb);
		}
		dev->last_rx = jiffies;
	}

	rxq_refill(dev);	/* Fill RX ring with skb's */

	return received_packets;
}

#ifndef MFU_TIMER
static irqreturn_t pxa168_eth_int_handler(int irq, void *dev_id)
{
	struct net_device *dev = (struct net_device *)dev_id;
	struct pxa168_private *mp = netdev_priv(dev);
	u32 icr;

	/* Read interrupt cause registers */
	icr = rdl(mp, INT_CAUSE);
	if (0x00 == icr)
		return IRQ_NONE;

	/* Clear new interrupts */
	wrl(mp, INT_CAUSE, icr ^ 0xffffffff);

	/* PHY status changed */
	if (icr & ICR_MII_CH) {
		int curr_desc_no;
		struct tx_desc *desc;

		/* last tx desc we set */
		curr_desc_no = (mp->tx_curr_desc_q + mp->tx_ring_size - 1)
			% mp->tx_ring_size;
		/* only tx interrupt if we are about to stop the queue */
		/* set tx interrupt enabled */
		desc = &mp->p_tx_desc_area[curr_desc_no];
		/*
			printk("curr_desc_no = %d, desc->cmd_sts = %08X\n",
				curr_desc_no, desc->cmd_sts);
		*/
		desc = &mp->p_tx_desc_area[mp->tx_used_desc_q];
		/*
			printk(KERN_INFO "tx_used_desc_q = %d,"
			"cmd_sts = %08X\n",
				mp->tx_used_desc_q, desc->cmd_sts);
		*/

		if (!mii_link_ok(&mp->mii) && netif_carrier_ok(dev)) {
			netif_stop_queue(dev);
			netif_carrier_off(dev);
		} else if (mii_link_ok(&mp->mii) && !netif_carrier_ok(dev)) {
			netif_carrier_on(dev);
			netif_wake_queue(dev);
		}
		mii_check_link(&mp->mii);
	}

#ifdef NAPI_PMR
	if (icr & (ICR_RXBUF | ICR_RXERR | ICR_TXBUF_H |
			ICR_TXBUF_L | ICR_TX_UDR)) {
		if (napi_schedule_prep(&mp->napi)) {
			/* Disable interrupts */
			wrl(mp, INT_MASK, 0);
			__napi_schedule(&mp->napi);
		}
	}
#else
	if (icr & (ICR_RXBUF | ICR_RXERR))
		rxq_process(dev, mp->rx_ring_size);

	txq_reclaim(dev, 0);
	if (netif_queue_stopped(dev)
	&& mp->tx_desc_count <= TX_DESC_COUNT_LOW)
		netif_wake_queue(dev);

#endif

	return IRQ_HANDLED;
}
#endif

static int setPortConfigExt(struct pxa168_private *mp, int mtu)
{
	int mtuSize;

	/* printk("%s: mtu = %d\n", __func__, mtu); */
	/* 64 should work but does not -- dhcp packets NEVER get transmitted. */
	if ((mtu > MAX_PKT_SIZE) || (mtu < 64))
		return -EINVAL;

	/* add source/dest mac addr (12) + pid (2) + crc (4) */
	mtu += ETH_EXTRA_HEADER;
#ifdef CONFIG_VLAN_8021Q
   mtu += 4;
#endif   
	if  (mtu <= 1518)
		mtuSize = PCXR_MFL_1518;	
	else if (mtu <= 1536)
		mtuSize = PCXR_MFL_1536;
	else if (mtu <= 2048)
		mtuSize = PCXR_MFL_2048;
	else
		mtuSize = PCXR_MFL_64K;

	/* Extended Port Configuration */
	if(nophy) {
		wrl(mp, PORT_CONFIG_EXT,
			PCXR_2BSM | /* Two byte suffix aligns IP hdr */
			PCXR_DSCP_EN |	/* Enable DSCP in IP */
			mtuSize |
			PCXR_FORCE_100M_FD |
			PCXR_TX_HIGH_PRI); /* Transmit - high priority queue */
	} else {
		wrl(mp, PORT_CONFIG_EXT,
			PCXR_2BSM | /* Two byte suffix aligns IP hdr */
			PCXR_DSCP_EN |	/* Enable DSCP in IP */
			mtuSize |
			PCXR_FLP |	/* do not force link pass */
			PCXR_TX_HIGH_PRI); /* Transmit - high priority queue */
	}

	/* subtract source/dest mac addr (12) + pid (2) + crc (4) */
	mtu -= ETH_EXTRA_HEADER;
	(mp->dev)->mtu = mtu;
	/* printk("%s: mtu = %d, mtuSize = %x\n", __func__, mtu, mtuSize); */
	return 0;
}

static void pxa168_init_hw(struct pxa168_private *mp)
{
	if (mp->pd != NULL) {
		mp->pd->init();
		if(!nophy)
			ethernet_phy_set(mp);
	}

	/* Disable interrupts */
	wrl(mp, INT_MASK, 0);
	wrl(mp, INT_CAUSE, 0);

	/* Write to ICR to clear interrupts. */
	wrl(mp, INT_W_CLEAR, 0);

	/* Abort any transmit and receive operations and put DMA
	 * in idle state.
	 */

	abortDMA(mp);
	/* Initialize address hash table */
	init_hashtable(mp);

	/* SDMA configuration */
	wrl(mp, SDMA_CONFIG,
			SDCR_BSZ8 | 	/* Burst size = 32 bytes */
			SDCR_RIFB |		/* Rx interrupt on frame */
			SDCR_BLMT |		/* Little endian transmit */
			SDCR_BLMR | 	/* Little endian receive */
			SDCR_RC_MAX_RETRANS); /* Max retransmit count */

	/* Port Configuration */
	if(nophy)
		wrl(mp, PORT_CONFIG, PCR_HS|PCR_HD);
	else
		wrl(mp, PORT_CONFIG, PCR_HS);	/* Hash size is 1/2kb */

	setPortConfigExt(mp, (mp->dev)->mtu);
}


/* rx/tx queue initialisation ***********************************************/
static int rxq_init(struct net_device *dev)
{
	struct pxa168_private *mp = netdev_priv(dev);
	volatile struct rx_desc *p_rx_desc;
	int size = 0, i = 0;
	int rx_desc_num = mp->rx_ring_size;

	/* Allocate RX and TX skb rings */
	mp->rx_skb = kmalloc(sizeof(*mp->rx_skb) * mp->rx_ring_size,
			GFP_KERNEL);
	if (!mp->rx_skb) {
		printk(KERN_ERR "%s: Cannot alloc RX skb ring\n", dev->name);
		return -ENOMEM;
	}

	/* Allocate RX ring */
	mp->rx_desc_count = 0;
	size = mp->rx_ring_size * sizeof(struct rx_desc);
	mp->rx_desc_area_size = size;

	mp->p_rx_desc_area = dma_alloc_coherent(NULL, size,
			&mp->rx_desc_dma,
			GFP_KERNEL);
	if (!mp->p_rx_desc_area) {
		printk(KERN_ERR "%s: Cannot alloc RX ring (size %d bytes)\n",
			dev->name, size);
		goto out;
	}
	memset((void *)mp->p_rx_desc_area, 0, size);

	/* initialize the next_desc_ptr links in the Rx descriptors ring */
	p_rx_desc = (struct rx_desc *)mp->p_rx_desc_area;
	for (i = 0; i < rx_desc_num; i++) {
		p_rx_desc[i].next_desc_ptr = mp->rx_desc_dma +
			((i + 1) % rx_desc_num) * sizeof(struct rx_desc);
	}

	/* Save Rx desc pointer to driver struct. */
	mp->rx_curr_desc_q = 0;
	mp->rx_used_desc_q = 0;

	mp->rx_desc_area_size = rx_desc_num * sizeof(struct rx_desc);

	return 0;

out:
	kfree(mp->rx_skb);
	return -ENOMEM;
}

static void rxq_deinit(struct net_device *dev)
{
	struct pxa168_private *mp = netdev_priv(dev);
	int curr;

	/* Free preallocated skb's on RX rings */
	for (curr = 0; mp->rx_desc_count && curr < mp->rx_ring_size; curr++) {
		if (mp->rx_skb[curr]) {
			dev_kfree_skb(mp->rx_skb[curr]);
			mp->rx_desc_count--;
		}
	}

	if (mp->rx_desc_count)
		printk(KERN_ERR
				"Error in freeing Rx Ring. %d skb's still\n",
				mp->rx_desc_count);

	/* Free RX ring */
	if (mp->p_rx_desc_area)
		dma_free_coherent(NULL, mp->rx_desc_area_size,
				mp->p_rx_desc_area, mp->rx_desc_dma);

	kfree(mp->rx_skb);

}

static int txq_init(struct net_device *dev)
{
	struct pxa168_private *mp = netdev_priv(dev);
	struct tx_desc *p_tx_desc;
	int size = 0, i = 0;
	int tx_desc_num = mp->tx_ring_size;

	mp->tx_skb = kmalloc(sizeof(*mp->tx_skb) * mp->tx_ring_size,
			GFP_KERNEL);
	if (!mp->tx_skb) {
		printk(KERN_ERR "%s: Cannot alloc TX skb ring\n", dev->name);
		return -ENOMEM;
	}

	/* Allocate TX ring */
	mp->tx_desc_count = 0;
	size = mp->tx_ring_size * sizeof(struct tx_desc);
	mp->tx_desc_area_size = size;

	mp->p_tx_desc_area = dma_alloc_coherent(NULL, size,
			&mp->tx_desc_dma,
			GFP_KERNEL);
	if (!mp->p_tx_desc_area) {
		printk(KERN_ERR "%s: Cannot allocate Tx Ring (size %d bytes)\n",
				dev->name, size);
		goto out;
	}
	/* check 16-byte alignment */
	BUG_ON((u32) mp->p_tx_desc_area & 0xf);
	memset((void *)mp->p_tx_desc_area, 0, mp->tx_desc_area_size);

	/* Initialize the next_desc_ptr links in the Tx descriptors ring */
	p_tx_desc = (struct tx_desc *)mp->p_tx_desc_area;
	for (i = 0; i < tx_desc_num; i++) {
		p_tx_desc[i].next_desc_ptr = mp->tx_desc_dma +
			((i + 1) % tx_desc_num) * sizeof(struct tx_desc);
	}

	mp->tx_curr_desc_q = 0;
	mp->tx_used_desc_q = 0;

	mp->tx_desc_area_size = tx_desc_num * sizeof(struct tx_desc);

	return 0;

out:
	kfree(mp->tx_skb);
	return -ENOMEM;

}

static void txq_deinit(struct net_device *dev)
{
	struct pxa168_private *mp = netdev_priv(dev);

	/* Free outstanding skb's on TX ring */
	txq_reclaim(dev, 1);

	BUG_ON(mp->tx_used_desc_q != mp->tx_curr_desc_q);

	/* Free TX ring */
	if (mp->p_tx_desc_area)
		dma_free_coherent(NULL, mp->tx_desc_area_size,
				mp->p_tx_desc_area, mp->tx_desc_dma);

	kfree(mp->tx_skb);
}

static int pxa168_eth_open(struct net_device *dev)
{
	struct pxa168_private *mp = netdev_priv(dev);
	int err;

	err = request_irq(dev->irq, pxa168_eth_int_handler, 0, dev->name, dev);
	if (err) {
		dev_err(&dev->dev, "can't assign irq\n");
		return -EAGAIN;
	}

	pxa168_init_hw(mp);
	pxa168_eth_set_rx_mode(dev);
	mp->rx_resource_err = 0;

	err = ethernet_phy_setup(dev);
	if (err)
		return -EAGAIN;


	memset(&mp->timeout, 0, sizeof(struct timer_list));
	mp->timeout.function = rxq_refill_timer_wrapper;
	mp->timeout.data = (unsigned long)dev;

	err = rxq_init(dev);
	if (err != 0)
		goto out_free_irq;

	err = txq_init(dev);
	if (err != 0)
		goto out_free_rx_skb;

	mp->rx_used_desc_q = 0;
	mp->rx_curr_desc_q = 0;
	rxq_refill(dev);	/* Fill RX ring with skb's */
	mp->rx_used_desc_q = 0;
	mp->rx_curr_desc_q = 0;
	eth_port_start(dev);


#ifdef NAPI_PMR
	napi_enable(&mp->napi);
#endif

#ifndef MFU_TIMER
	if (mii_link_ok(&mp->mii)) {
		netif_carrier_on(dev);
		netif_wake_queue(dev);
	} else {
		netif_carrier_off(dev);
		netif_stop_queue(dev);
	}
	mii_check_link(&mp->mii);
#else
	netif_carrier_on(dev);
	netif_wake_queue(dev);
#endif

#ifdef MFU_TIMER
	mp->mfu_timer->expires = jiffies + (HZ / 10);	/* 100 mSec */
	add_timer(mp->mfu_timer);
#endif

	return 0;

out_free_rx_skb:
	rxq_deinit(dev);
out_free_irq:
	free_irq(dev->irq, dev);

	return err;
}

static int pxa168_eth_stop(struct net_device *dev)
{
	struct pxa168_private *mp = netdev_priv(dev);
	eth_port_reset(dev);

#ifdef NAPI_PMR
	napi_disable(&mp->napi);
#endif
	netif_stop_queue(dev);

	del_timer_sync(&mp->timeout);

	/* Disable interrupts */
	wrl(mp, INT_MASK, 0);
	wrl(mp, INT_CAUSE, 0);

	/* Write to ICR to clear interrupts. */
	wrl(mp, INT_W_CLEAR, 0);

#ifndef MFU_TIMER
	free_irq(dev->irq, dev);
#endif

#ifdef MFU_TIMER
	del_timer_sync(mp->mfu_timer);
#endif
	netif_carrier_off(dev);

	rxq_deinit(dev);
	txq_deinit(dev);

	return 0;
}

/*
 * Changes MTU (maximum transfer unit) of the gigabit ethenret port
 *
 * Input :	pointer to ethernet interface network device structure
 *		new mtu size
 * Output :	0 upon success, -EINVAL upon failure
 */
static int pxa168_eth_change_mtu(struct net_device *dev, int mtu)
{
	struct pxa168_private *mp = netdev_priv(dev);
	int retval;

	retval = setPortConfigExt(mp, mtu);
	if (retval != 0)
		return retval;

	/*
	 * Stop then re-open the interface. This will allocate RX skb's with
	 * the new MTU.
	 * There is a possible danger that the open will not successed, due
	 * to memory is full, which might fail the open function.
	 */
	if (netif_running(dev)) {
		pxa168_eth_stop(dev);
		if (pxa168_eth_open(dev))
			printk(KERN_ERR
					"%s: Fatal error on opening device\n",
					dev->name);
	}

	return 0;
}


/**
 * eth_alloc_tx_desc_index - return the index of the next available tx desc
 */
static int eth_alloc_tx_desc_index(struct pxa168_private *mp)
{
	int tx_desc_curr;

	BUG_ON(mp->tx_desc_count >= mp->tx_ring_size);

	tx_desc_curr = mp->tx_curr_desc_q;
	mp->tx_curr_desc_q = (tx_desc_curr + 1) % mp->tx_ring_size;

	pr_debug("mp->tx_curr_desc_q=%d\n", mp->tx_curr_desc_q);
	pr_debug("mp->tx_used_desc_q=%d\n", mp->tx_used_desc_q);

	if (mp->tx_curr_desc_q == mp->tx_used_desc_q)
		pr_debug("mp->tx_curr_desc_q=%d\n", mp->tx_curr_desc_q);

	BUG_ON(mp->tx_curr_desc_q == mp->tx_used_desc_q);

	return tx_desc_curr;
}


/**
 * eth_tx_submit_descs_for_skb - submit data from an skb to the tx hw
 *
 * Ensure the data for an skb to be transmitted is mapped properly,
 * then fill in descriptors in the tx hw queue and start the hardware.
 */
static void eth_tx_submit_descs_for_skb(struct pxa168_private *mp,
		struct sk_buff *skb)
{
	int tx_index;
	struct tx_desc *desc;
	int length;

	tx_index = eth_alloc_tx_desc_index(mp);
	desc = &mp->p_tx_desc_area[tx_index];

	length = skb->len;

#ifdef CONFIG_PXA_SWITCH_WORKAROUND
   {
      unsigned short *usp = (unsigned short *)skb->data;
      if (usp[6] == 0x0081 && length < 64 && length > 60)
         length += (64 - length);
	}
#endif

	mp->tx_skb[tx_index] = skb;

	desc->byte_cnt = length;
	desc->buf_ptr = dma_map_single(NULL, skb->data, length, DMA_TO_DEVICE);

	/* ensure all other descriptors are written before first cmd_sts */
	wmb();

	if (mp->tx_desc_count >= TX_DESC_COUNT_HIGH)
		desc->cmd_sts = BUF_OWNED_BY_DMA | TX_GEN_CRC | TX_FIRST_DESC |
				TX_ZERO_PADDING | TX_LAST_DESC |
				TX_EN_INT;
	else
		desc->cmd_sts = BUF_OWNED_BY_DMA | TX_GEN_CRC | TX_FIRST_DESC |
						TX_ZERO_PADDING | TX_LAST_DESC;
	wmb();

	/* only use high priority */
	wrl(mp, SDMA_CMD, SDMA_CMD_TXDL | SDMA_CMD_TXDH | SDMA_CMD_ERD);

	mp->tx_desc_count++;
}

#ifdef NAPI_PMR
static int pxa168_napi_poll(struct napi_struct *napi, int budget)
{
	struct pxa168_private *mp =
		container_of(napi, struct pxa168_private, napi);
	struct net_device *dev = mp->dev;
	int rxPacketsProcessed;

	txq_reclaim(dev, 0);
	if (netif_queue_stopped(dev)
	&& mp->tx_desc_count <= TX_DESC_COUNT_LOW)
		netif_wake_queue(dev);

	rxPacketsProcessed = rxq_process(dev, budget);

	if (rxPacketsProcessed <= budget) {
		/* Enable interrupts */
		wrl(mp, INT_MASK, ALL_INTS);

		/*
		 * do NOT change this order
		 * if a packet arrived and interrupts were enabled after
		 * netif_rx_complete we might miss this packet
		 * if no other packet arrived
		 * this is why the extra call to rxq_process.
		 */
		rxq_process(dev, budget);
		napi_complete(napi);
		return 0;
	} else {
		return 1;
	}
}

#endif

/**
 * pxa168_eth_start_xmit - queue an skb to the hardware for transmission
 *
 */
static int pxa168_eth_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct pxa168_private *mp = netdev_priv(dev);
	struct net_device_stats *stats = &dev->stats;
	unsigned long flags;
	
#ifdef RESTART_DMA_WORKAROUND
	int w_cnt;
	int timedout;
	u32 mdelaycnt;
#endif

	BUG_ON(netif_queue_stopped(dev));
	BUG_ON(skb == NULL);

#ifndef RESTART_DMA_WORKAROUND
	txq_reclaim(dev, 0);
#endif

	if (mp->tx_desc_count > TX_DESC_COUNT_HIGH) {
		netif_stop_queue(dev);
		return NETDEV_TX_BUSY;
	}

	
	if (skb_shinfo(skb)->nr_frags && __skb_linearize(skb)) {
		spin_lock_irqsave(&mp->lock, flags);
			stats->tx_dropped++;
		spin_unlock_irqrestore(&mp->lock, flags);
		printk(KERN_DEBUG "%s: failed to linearize tiny "
				"unaligned fragment\n", dev->name);
		return NETDEV_TX_OK;
	}

	spin_lock_irqsave(&mp->lock, flags);
		eth_tx_submit_descs_for_skb(mp, skb);
		stats->tx_bytes += skb->len;
		stats->tx_packets++;
		dev->trans_start = jiffies;
	spin_unlock_irqrestore(&mp->lock, flags);

#ifdef RESTART_DMA_WORKAROUND
	w_cnt = 0;
	mdelaycnt = dev->trans_start;
	/* just reclaim */
	timedout = 0;
	while (txq_reclaim(dev, 0) < 0) {
		udelay(100);
		w_cnt++;
		if (w_cnt > 15 && mp->tx_desc_count > 0) {
			/* the DMA did not actually start, so hit it again */
			wrl(mp, SDMA_CMD,
				SDMA_CMD_TXDL | SDMA_CMD_TXDH | SDMA_CMD_ERD);
		}

		if (time_after(jiffies, mdelaycnt + msecs_to_jiffies(1))) {
			/* let other components of linux run */
			mdelay(1);
			mdelaycnt = jiffies;
		}

		if (time_after(jiffies,
				dev->trans_start + msecs_to_jiffies(1000))) {
			timedout = 1;
			break;
		}
	}

	if (timedout) {
		printk(KERN_ERR "%s : Timed Out trans_start = %lu, jiffies = %lu\n",
			__func__, dev->trans_start, jiffies);
		abortDMA(mp);
		txq_reclaim(dev, 1);
		abortDMA(mp);
	}
#endif

	return NETDEV_TX_OK;		/* success */
}

static void pxa168_init_ethtool_cmd(struct net_device *dev, int phy_address,
		int speed, int duplex,
		struct ethtool_cmd *cmd)
{
	memset(cmd, 0, sizeof(*cmd));

	cmd->port = PORT_MII;
	cmd->transceiver = XCVR_INTERNAL;
	cmd->phy_address = phy_address;

	if (speed == 0) {
		cmd->autoneg = AUTONEG_ENABLE;
		/* mii lib checks, but doesn't use speed on AUTONEG_ENABLE */
		cmd->speed = SPEED_10;
		cmd->advertising = ADVERTISED_10baseT_Half |
			ADVERTISED_10baseT_Full  |
			ADVERTISED_100baseT_Half |
			ADVERTISED_100baseT_Full;
	} else {
		cmd->autoneg = AUTONEG_DISABLE;
		cmd->speed = speed;
		cmd->duplex = duplex;
	}
}

/*
 * Wrappers for MII support library.
 */
static int pxa168_mdio_read(struct net_device *dev,
				int phy_id, int location)
{
	int val;
	struct pxa168_private *mp = netdev_priv(dev);

	if(nophy) {
		switch (location) {
			case 0: return 0x3100;
			case 1: return 0x782d;
			case 3: return 0x1e1;
			case 5: return 0xc5e1;
		}
	}

	eth_port_read_smi_reg(mp, location, &val);
	return val;
}

static void pxa168_mdio_write(struct net_device *dev, int phy_id,
			int location, int val)
{
	struct pxa168_private *mp = netdev_priv(dev);

	eth_port_write_smi_reg(mp, location, val);
}


static int pxa168_eth_do_ioctl(struct net_device *dev, struct ifreq *ifr,
					int cmd)
{
	struct pxa168_private *mp = netdev_priv(dev);

	return generic_mii_ioctl(&mp->mii, if_mii(ifr), cmd, NULL);
}

static int pxa168_set_settings(struct net_device *dev,
			struct ethtool_cmd *cmd)
{
	struct pxa168_private *mp = netdev_priv(dev);
	int err;

	spin_lock_irq(&mp->lock);
	err = mii_ethtool_sset(&mp->mii, cmd);
	spin_unlock_irq(&mp->lock);

	return err;
}

static int ethernet_phy_setup(struct net_device *dev)
{
	int err;
	int speed = 0;
	int duplex = DUPLEX_FULL;
	struct pxa168_private *mp = netdev_priv(dev);
	struct ethtool_cmd cmd;
	if(nophy) {
		speed = SPEED_100;
	} else {
		err = ethernet_phy_detect(mp);
		if (err) {
			printk(KERN_INFO "pxa168 ethernet port %d: "
					"No PHY detected at addr %d\n",
					mp->port_num, ethernet_phy_get(mp));
			return err;
		}

		/* if no mac address assigned -- generate one */
		if (memcmp (dev->dev_addr, pxa168_mac_str, sizeof(pxa168_mac_str)) == 0) {
			memcpy(dev->dev_addr, TSOUI, sizeof(TSOUI));
			dev->dev_addr[0] |= (1<<6);  /* set locally admin bit */
			get_random_bytes(&dev->dev_addr[4], 2);
		}

		ethernet_phy_reset(mp);
	}
	pxa168_init_ethtool_cmd(dev, mp->mii.phy_id, speed, duplex, &cmd);
	pxa168_set_settings(dev, &cmd);
	update_hash_table_mac_address(mp, dev->dev_addr, dev->dev_addr);
	return 0;
}

#ifdef MFU_TIMER
static void pxa168_eth_timer_handler(unsigned long d)
{
	struct net_device *dev = (struct net_device *)d;
	struct pxa168_private *mp = netdev_priv(dev);
	int ret;

	ret = rxq_process(dev, mp->rx_ring_size);
	if (ret)
		mp->mfu_timer->expires = jiffies+msecs_to_jiffies(1);
	else
		mp->mfu_timer->expires = jiffies+msecs_to_jiffies(2);

	 add_timer(mp->mfu_timer);
}
#endif


/************* End ethtool support *************************/


static int pxa168_get_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	struct pxa168_private *mp = netdev_priv(dev);
	return mii_ethtool_gset(&mp->mii, cmd);
}

static void pxa168_get_drvinfo(struct net_device *dev,
	struct ethtool_drvinfo *info)
{
	strcpy(info->driver, DRIVER_NAME);
	strcpy(info->version, DRIVER_VERSION);
	strcpy(info->fw_version, "N/A");
	strcpy(info->bus_info, "N/A");
}

static u32 pxa168_get_link(struct net_device *dev)
{
	struct pxa168_private *mp = netdev_priv(dev);
	return mii_link_ok(&mp->mii);
}


static int pxa168_phys_id(struct net_device *dev, u32 data)
{
	struct pxa168_private *mp = netdev_priv(dev);
	mp->phy_addr = data % 32;
	ethernet_phy_set(mp);
	return 0;
}

static const struct net_device_ops pxa168_netdev_ops = {
	.ndo_open		 = pxa168_eth_open,
	.ndo_stop		 = pxa168_eth_stop,
	.ndo_start_xmit		 = pxa168_eth_start_xmit,
	.ndo_set_mac_address	 = pxa168_eth_set_mac_address,
	.ndo_set_rx_mode	 = pxa168_eth_set_rx_mode,
	.ndo_validate_addr 	 = eth_validate_addr,
	.ndo_change_mtu		 = pxa168_eth_change_mtu,
	.ndo_do_ioctl		 = pxa168_eth_do_ioctl,
	.ndo_tx_timeout		 = pxa168_eth_tx_timeout,
};

static const struct ethtool_ops pxa168_ethtool_ops = {
	.get_settings		= pxa168_get_settings,
	.get_drvinfo		= pxa168_get_drvinfo,
	.get_link		= pxa168_get_link,
	.get_ts_info = ethtool_op_get_ts_info,
};


static int pxa168_eth_probe(struct platform_device *pdev)
{
	struct pxa168_eth_platform_data *pd;
	struct pxa168_private *mp;
	struct net_device *dev = NULL;
	struct resource *res;
	struct clk *clk;
	int err;
	int duplex = DUPLEX_FULL;
	int speed = 0;			/* default to auto-negotiation */
	volatile unsigned long *syscon;
	u16 model;

	printk(KERN_NOTICE "PXA168 10/100 Ethernet Driver\n");
	syscon = (unsigned long *)0xfe400000; //ioremap(0x80004000, 0x1000);
	model = ioread16(syscon);
	printk("pxa168: model 0x%X\n", model);
	if(model == 0x4712 ||
           model == 0x4720 ||
           model == 0x7250
           ) {
		nophy = 1;
		printk(KERN_INFO "pxa168: disabling phy\n");
	}
	//iounmap(syscon);

	/* enable MFU clock  */
	clk = clk_get(&pdev->dev, "MFUCLK");
	if (IS_ERR(clk)) {
		printk(KERN_ERR "pxa168: fast Ethernet failed to get MFU clock\n");
		return -1;
	}
	clk_prepare(clk);
	clk_enable(clk);

	dev = alloc_etherdev(sizeof(struct pxa168_private));
	if (!dev)
		return -ENOMEM;

	platform_set_drvdata(pdev, dev);

	mp = netdev_priv(dev);
	mp->dev = dev;
	mp->clk = clk;

	/*register*/
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL)
		return -ENODEV;

	mp->base  = ioremap(res->start, res->end - res->start + 1);
	if (mp->base == NULL)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	BUG_ON(!res);
	dev->irq = res->start;

	/* No need to Tx Timeout */

	dev->watchdog_timeo = 2 * HZ;
	dev->base_addr = 0;
	SET_ETHTOOL_OPS(dev, &pxa168_ethtool_ops);
	dev->netdev_ops = &pxa168_netdev_ops;

	/* Configure the timeout task */
	INIT_WORK(&mp->tx_timeout_task, pxa168_eth_tx_timeout_task);

	spin_lock_init(&mp->lock);

	mp->rx_ring_size = NUM_RX_DESCS;
	mp->tx_ring_size = NUM_TX_DESCS;
	mp->port_num = 0;

	// Set random mac address on startup
	random_ether_addr(dev->dev_addr);
	memcpy(dev->dev_addr, TSOUI, sizeof(TSOUI));
	
	/* start at 31 since it allows systems with phy's at */
	/* addr 31 and 0 to be found quickly. */
	/* Marvell boards have phys at addr 31 or 0  */
	mp->phy_addr = 31;
	pd = pdev->dev.platform_data;
	mp->pd = pd;
	if (pd != NULL) {

		pd->init();
		mp->port_num = pd->port_number;

		/* use platform data has mac_addr if existed */
		if (is_valid_ether_addr(pd->mac_addr))
			memcpy(dev->dev_addr, pd->mac_addr, 6);

		if (pd->force_phy_addr)
			mp->phy_addr = pd->phy_addr & 0x1f;  /* 0 - 31 legal */

		if (pd->rx_queue_size)
			mp->rx_ring_size = pd->rx_queue_size;

		if (pd->tx_queue_size)
			mp->tx_ring_size = pd->tx_queue_size;

		duplex = pd->duplex;
		speed = pd->speed;
	}

#ifdef NAPI_PMR
	netif_napi_add(dev, &mp->napi, pxa168_napi_poll, mp->rx_ring_size);
#endif

	/* Hook up MII support for ethtool */
	mp->mii.dev = dev;
	mp->mii.mdio_read = pxa168_mdio_read;
	mp->mii.mdio_write = pxa168_mdio_write;
	mp->mii.phy_id = ethernet_phy_get(mp);
	mp->mii.phy_id_mask = 0x1f;	/* phy_id is 5 bits */
	mp->mii.reg_num_mask = 0x1f;	/* 31 reg */

	pxa168_init_hw(mp);

	SET_NETDEV_DEV(dev, &pdev->dev);
	err = register_netdev(dev);
	if (err)
		goto out;

#ifdef MFU_TIMER
	mp->mfu_timer =
		(struct timer_list *)
			kmalloc(sizeof(struct timer_list), GFP_KERNEL);
	init_timer(mp->mfu_timer);
	mp->mfu_timer->function = pxa168_eth_timer_handler;
	mp->mfu_timer->data = (unsigned long) dev;
#endif

	return 0;

out:
	/* disable MFU clock  */
	if (mp->clk) {
		clk_disable(mp->clk);
		clk_put(mp->clk);
		mp->clk = NULL;
	}

	if (mp->base) {
		iounmap(mp->base);
		mp->base = NULL;
	}

	if (dev)
		free_netdev(dev);
	return err;
}

static int pxa168_eth_remove(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	struct pxa168_private *mp = netdev_priv(dev);

#ifdef MFU_TIMER
	kfree(mp->mfu_timer);
#endif

	if (mp->htpr) {
		dma_free_coherent(NULL, HASH_ADDR_TABLE_SIZE + 7,
				mp->htpr, mp->htpr_dma);
		mp->htpr = NULL;
	}

	/* disable MFU clock  */
	if (mp->clk) {
		clk_disable(mp->clk);
		clk_put(mp->clk);
		mp->clk = NULL;
	}

	iounmap(mp->base);
	mp->base = NULL;

	unregister_netdev(dev);
	flush_scheduled_work();

	free_netdev(dev);
	platform_set_drvdata(pdev, NULL);


	return 0;
}

static void pxa168_eth_shutdown(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	eth_port_reset(dev);
}

#ifdef CONFIG_PM
static int pxa168_eth_resume(struct platform_device *pdev)
{
	return -ENOSYS;
}

static int pxa168_eth_suspend(struct platform_device *pdev, pm_message_t state)
{
	return -ENOSYS;
}

#else
#define pxa168_eth_resume NULL
#define pxa168_eth_suspend NULL
#endif


static struct platform_driver pxa168_eth_driver = {
	.probe = pxa168_eth_probe,
	.remove = pxa168_eth_remove,
	.shutdown = pxa168_eth_shutdown,
	.resume	= pxa168_eth_resume,
	.suspend = pxa168_eth_suspend,
	.driver = {
		.name = DRIVER_NAME,
	},
};
module_platform_driver(pxa168_eth_driver);


static int __init pxa168_init_module(void)
{
	return platform_driver_register(&pxa168_eth_driver);
}

static void __exit pxa168_cleanup_module(void)
{
	platform_driver_unregister(&pxa168_eth_driver);
}

module_init(pxa168_init_module);
module_exit(pxa168_cleanup_module);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Ethernet driver for Marvell PXA168");
MODULE_ALIAS("platform:pxa168_eth");


