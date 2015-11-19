/*
 *pxa168 ethernet platform device data definition file.
 */
#ifndef __LINUX_PXA168_ETH_H
#define __LINUX_PXA168_ETH_H

struct pxa168_eth_platform_data {
	int	port_number;
	int	phy_addr;
	u16		force_phy_addr;	/* force override if phy_addr == 0 */

	/*
	 * If speed is 0, then speed and duplex are autonegotiated.
	 */
	int	speed;		/* 0, SPEED_10, SPEED_100 */
	int	duplex;		/* DUPLEX_HALF or DUPLEX_FULL */

	/*
	 * Override default RX/TX queue sizes if nonzero.
	 */
	int	rx_queue_size;
	int	tx_queue_size;

	/*
	 * init callback is used for board specific initialization
	 * e.g on Aspenite its used to initialize the PHY transceiver.
	 */
	int (*init)(void);
	u8		mac_addr[6];	/* mac address if non-zero*/
};

#endif /* __LINUX_PXA168_ETH_H */
