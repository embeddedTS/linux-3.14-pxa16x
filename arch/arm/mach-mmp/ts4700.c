/*
 *  linux/arch/arm/mach-mmp/ts4700.c
 *
 *  Support for the Marvell PXA168-based Technologic Systems' ts4700 board
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/smc91x.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/delay.h>
#include <linux/usb.h>
#include <linux/usb/otg.h>
#include <linux/pci.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/addr-map.h>
#include <mach/mfp-pxa168.h>
#include <mach/pxa168.h>
#include <mach/irqs.h>
#include <mach/mfp-pxa168.h>

#if defined(CONFIG_SPI_PXA2XX)
#include <linux/spi/spi.h>
#include <plat/pxa2xx_spi.h>
#endif

#if defined(CONFIG_TOUCHSCREEN_ILI210X) || defined(CONFIG_TOUCHSCREEN_ILI210X_MODULE)
#include <linux/input/ili210x.h>
#endif

#include <linux/proc_fs.h>
#include <linux/irq.h>
#include <linux/uaccess.h>
#include <linux/poll.h>

#include "common.h"
/*used by expander max7312, 16 pins gpio expander */
#define GPIO_EXT0(x)		(NR_BUILTIN_GPIO + (x))
#define GPIO_EXT1(x)		(NR_BUILTIN_GPIO + 16 + (x))
#define GPIO_EXT2(x)		(NR_BUILTIN_GPIO + 16 + 16 + (x))

#define CARD_EN GPIO_EXT1(0)
#define CAM_PWDN GPIO_EXT1(1)
#define TW9907_PWDN GPIO_EXT1(4)
#define TW9907_RST_N GPIO_EXT1(2)

static int tsBaseBoard;
int tsGetBaseBoard(void)
{
   return tsBaseBoard;
}
EXPORT_SYMBOL(tsGetBaseBoard);


static unsigned short model;
int tsGetModel(void) {
   return model;
}
EXPORT_SYMBOL(tsGetModel);

static unsigned short cpuModel;
int getCpuModel(void) {
   return cpuModel;  
}

static int enable_pcie;
int getPCIeEnable(void) {
   return enable_pcie;  
}

extern void spi_flashinit(void);


static unsigned long ts4700_pin_config[] __initdata = {
	/* Data Flash Interface Nana or eMMC/eSD*/
#if defined(CONFIG_MMC3)
	GPIO0_MMC3_DAT7,
	GPIO1_MMC3_DAT6,
	GPIO2_MMC3_DAT5,
	GPIO3_MMC3_DAT4,
	GPIO4_MMC3_DAT3,
	GPIO5_MMC3_DAT2,
	GPIO6_MMC3_DAT1,
	GPIO7_MMC3_DAT0,
	GPIO8_MMC3_CLK,
	GPIO9_MMC3_CMD,
	GPIO16_SMC_nCS0_DIS,
#else
	GPIO0_DFI_D15,
	GPIO1_DFI_D14,
	GPIO2_DFI_D13,
	GPIO3_DFI_D12,
	GPIO4_DFI_D11,
	GPIO5_DFI_D10,
	GPIO6_DFI_D9,
	GPIO7_DFI_D8,
	GPIO8_DFI_D7,
	GPIO9_DFI_D6,
	GPIO10_DFI_D5,
	GPIO11_DFI_D4,
	GPIO12_DFI_D3,
	GPIO13_DFI_D2,
	GPIO14_DFI_D1,
	GPIO15_DFI_D0,
#if defined(CONFIG_PXA168_CF)
	/* Compact Flash Controller */
	GPIO19_CF_nCE1,
	GPIO20_CF_nCE2,
	GPIO22_ND_CLE,
	GPIO23_CF_nALE,
	GPIO25_CF_nRESET,
	GPIO26_ND_RnB1,
	GPIO27_ND_RnB2,
	GPIO28_CF_RDY,
	GPIO29_CF_STSCH,
	GPIO30_CF_nREG,
	GPIO31_CF_nIOIS16,
#if defined(CONFIG_PXA168_CF_USE_GPIO_CARDDETECT)
	GPIO32_GPIO,	/* CF_nCD1 IRQ */
	GPIO33_GPIO,    /* CF_nCD2 IRQ */
#else
	GPIO32_CF_nCD1,
	GPIO33_CF_nCD2,
#endif
	GPIO34_SMC_nCS1,
	GPIO35_CF_INPACK,
	GPIO36_CF_nWAIT,
#else


#endif
#endif

	/* LCD */
	GPIO56_LCD_FCLK_RD,      // LCD_VSYNC
	GPIO57_LCD_LCLK_A0,      // LCD_HSYNC
	GPIO58_LCD_PCLK_WR,      // LCD_CLK
	GPIO59_LCD_DENA_BIAS,    // LCD_DE
	GPIO60_LCD_DD0,          // LCD_D00
	GPIO61_LCD_DD1,
	GPIO62_LCD_DD2,
	GPIO63_LCD_DD3,
	GPIO64_LCD_DD4,
	GPIO65_LCD_DD5,
	GPIO66_LCD_DD6,
	GPIO67_LCD_DD7,
	GPIO68_LCD_DD8,
	GPIO69_LCD_DD9,
	GPIO70_LCD_DD10,
	GPIO71_LCD_DD11,
	GPIO72_LCD_DD12,
	GPIO73_LCD_DD13,
	GPIO74_LCD_DD14,
	GPIO75_LCD_DD15,

	GPIO76_LCD_DD16,
	GPIO77_LCD_DD17,
	GPIO78_LCD_DD18,
	GPIO79_LCD_DD19,
	GPIO80_LCD_DD20,
	GPIO81_LCD_DD21,
	GPIO82_LCD_DD22,
	GPIO83_LCD_DD23,        // LCD_D23

   GPIO84_PWM2_OUT,        // PWM2 or ONE_WIRE
   GPIO85_PWM1_OUT,        // LCD_PWM
   
	/* i2c bus */
	GPIO105_CI2C_SDA,
	GPIO106_CI2C_SCL,

#if !defined(CONFIG_MTD_M25P80)
	/* UART1 */
	GPIO107_UART1_RXD,
	GPIO108_UART1_TXD,
#else
	/* SSP1 */
	GPIO107_SPI_NOR_RXD,
	GPIO108_SPI_NOR_TXD,
	GPIO109_SPI_NOR_SYSCLK,
	GPIO110_GPIO,
	GPIO111_SPI_NOR_CLK,
#endif

	/* SSP0 */
	GPIO113_I2S_MCLK,
	GPIO114_I2S_FRM,
	GPIO115_I2S_BCLK,
	GPIO116_I2S_TXD,
	GPIO117_I2S_RXD,

	/* MFU (a.k.a. ethernet) */
	GPIO86_TX_CLK,
	GPIO87_TX_EN,
	GPIO88_TX_DQ3,
	GPIO89_TX_DQ2,
	GPIO90_TX_DQ1,
	GPIO91_TX_DQ0,
	GPIO92_MII_CRS,
	GPIO93_MII_COL,
	GPIO94_RX_CLK,
	GPIO95_RX_ER,
	GPIO96_RX_DQ3,
	GPIO97_RX_DQ2,
	GPIO98_RX_DQ1,
	GPIO99_RX_DQ0,
	GPIO100_MII_MDC,
	GPIO101_MII_MDIO,
	GPIO103_RX_DV,
	
#if defined(CONFIG_WLAN_8688_SDIO)
	/* sdh MMC2, wlan*/
	GPIO90_MMC2_DAT3,
	GPIO91_MMC2_DAT2,
	GPIO92_MMC2_DAT1,
	GPIO93_MMC2_DAT0,
	GPIO94_MMC2_CMD,
	GPIO95_MMC2_CLK,
#endif

   /* MMC2 is off-board SD card */
	GPIO33_MMC2_DAT3,
   GPIO32_MMC2_DAT2,
   GPIO31_MMC2_DAT1,
   GPIO30_MMC2_DAT0,
   GPIO28_MMC2_CMD,
   GPIO118_MMC2_CLK,

   GPIO27_SMC_IRQ,   /* TS FPGA interrupt is on MFP #27 */
   
   GPIO49_GPIO,
   
};

#define ENET_RESET_N GPIO_EXT1(9)
#define ENET_COMA_N GPIO_EXT1(10)

static inline int ts4700_eth_init(void)
{
   return 0;
}

static struct pxa168_eth_platform_data pxa168_eth_data = {   
	.phy_addr    = 0,
	.speed       = 0, /* Autonegotiation */
	.init		= ts4700_eth_init,
};

#if defined(CONFIG_PCI) || defined(CONFIG_PCI_TS47XX)

//#define PCIE_1P5V_SHDN_N GPIO_EXT2(1)
#define PCIE_3P3V_SHDN_N GPIO_EXT2(2)
#define PCIE_REFCLK_OE   GPIO_EXT2(6)

/* TODO: make it a static function */
int pxa168_gpio_pcie_init(void)
{
// This GPIO request makes PCIe never work on the TS-4710.
// The DIO in question does not appear to exist?
// Removed all PCIE_1P5V_SHDN_N references.
/*	if (gpio_request(PCIE_1P5V_SHDN_N, "PCIE_1P5V_SHDN_N")) {
		printk(KERN_ERR "%s %d: Request GPIO failed,"
				"gpio: %d \n", __FILE__, __LINE__, PCIE_1P5V_SHDN_N);
		return -EIO;
	}*/

	if (gpio_request(PCIE_3P3V_SHDN_N, "PCIE_3P3V_SHDN_N")) {
//		gpio_free(PCIE_1P5V_SHDN_N);
		printk(KERN_ERR "%s %d: Request GPIO failed,"
				"gpio: %d\n", __FILE__, __LINE__, PCIE_3P3V_SHDN_N);
		return -EIO;
	}

	if (gpio_request(PCIE_REFCLK_OE, "PCIE_REFCLK_OE")) {
//		gpio_free(PCIE_1P5V_SHDN_N);
		gpio_free(PCIE_3P3V_SHDN_N);
		printk(KERN_ERR "%s %d: Request GPIO failed,"
				"gpio: %d\n", __FILE__, __LINE__, PCIE_REFCLK_OE);
		return -EIO;
	}

	//gpio_direction_output(PCIE_1P5V_SHDN_N, 1);
	gpio_direction_output(PCIE_3P3V_SHDN_N, 1);
	gpio_direction_output(PCIE_REFCLK_OE, 1);

//	gpio_free(PCIE_1P5V_SHDN_N);
	gpio_free(PCIE_3P3V_SHDN_N);
	gpio_free(PCIE_REFCLK_OE);

	return 0;
}

static struct pxa168_pcie_platform_data pxa168_pcie_data = {
	.init		= pxa168_gpio_pcie_init,
};
#endif

#if defined(CONFIG_PXA168_CF) && defined(CONFIG_PXA168_CF_USE_GPIO_CARDDETECT)
static struct resource pxa168_cf_resources[] = {
       [0] = {
               .start  = 0xD4285000,
               .end    = 0xD4285800,
               .flags  = IORESOURCE_MEM,
       },

       [1] = {
               .start  = IRQ_PXA168_CF,
               .end    = IRQ_PXA168_CF,
               .flags  = IORESOURCE_IRQ,
               },
       [2] = {
               .start  = IRQ_GPIO(32),
               .end    = IRQ_GPIO(32),
               .flags  = IORESOURCE_IRQ,
               },
};

struct platform_device pxa168_cf_device = {
       .name           = "pxa168-cf",
       .resource       = pxa168_cf_resources,
       .num_resources  = ARRAY_SIZE(pxa168_cf_resources),
};

static void __init pxa168_cf_init(void)
{
       platform_device_register(&pxa168_cf_device);
}
#endif

/*
 * mfp is shared in card, cam and tw9907, only one is effective
 */
typedef enum{
	SW_CARD    = 0x01,
	SW_CAM_ON  = 0x02,
	SW_CAM_OFF = 0x03,
	SW_TW9907  = 0x04,
} SW_TYPE_T;


#if defined(CONFIG_PXA168_MSP)
/* msp platform data */
static mfp_cfg_t mfp_cfg_msp[]  = {
	GPIO40_MSP_DAT1,
	GPIO41_MSP_DAT0,
	GPIO43_MSP_DAT2,
	GPIO44_MSP_DAT3,
	GPIO42_MSP_BS,
	GPIO50_MSP_SCLK,
};

static void mspro_mfp_config(void)
{	
	mfp_config(ARRAY_AND_SIZE(mfp_cfg_msp));
}

static struct card_platform_data msp_ops = {
	/* GPIO84 used as mspro detect pin */
	.pin_detect		= MFP_PIN_GPIO84,
	.mfp_config		= mspro_mfp_config,
};
#endif

#if defined(CONFIG_PXA168_CAMERA)
static mfp_cfg_t ts4700_cam_pins[] = {
	GPIO37_CAM_DAT7,
	GPIO38_CAM_DAT6,
	GPIO39_CAM_DAT5,
	GPIO40_CAM_DAT4,
	GPIO41_CAM_DAT3,
	GPIO42_CAM_DAT2,
	GPIO43_CAM_DAT1,
	GPIO44_CAM_DAT0,
	GPIO46_CAM_VSYNC,
	GPIO48_CAM_HSYNC,
	GPIO54_CAM_MCLK,
	GPIO55_CAM_PCLK,
};

/* sensor init */
static int sensor_power_onoff(int on, int unused)
{
	/*
	 * on, 1, power on
	 * on, 0, power off
	 */
	int ret = 0;
	if(on){		
			mfp_config(ARRAY_AND_SIZE(ts4700_cam_pins));
	}
	return ret;
}

static struct sensor_platform_data ov7670_sensor_data = {
	.id = SENSOR_LOW,
	.power_on = sensor_power_onoff,
};

/* sensor init over */
#endif


/* SPI Control Register. */
#define     CFG_SCLKCNT(div)                    (div<<24)  /* 0xFF~0x2 */
#define     CFG_RXBITS(rx)                      ((rx - 1)<<16)   /* 0x1F~0x1 */
#define     CFG_TXBITS(tx)                      ((tx - 1)<<8)    /* 0x1F~0x1, 0x1: 2bits ... 0x1F: 32bits */
#define     CFG_SPI_ENA(spi)                    (spi<<3)
#define     CFG_SPI_SEL(spi)                    (spi<<2)   /* 1: port1; 0: port0 */
#define     CFG_SPI_3W4WB(wire)                 (wire<<1)  /* 1: 3-wire; 0: 4-wire */


#if defined(CONFIG_GPIO_PCA953X)

#if defined(CONFIG_WLAN_8688_SDIO)
static int stamp_8688_wlan_poweron(void);
#endif

/* GPIO expander max7312 could reuse PCA953X */

static struct pca953x_platform_data max7312_data[] = {
	/* three max7312 in system */

	[0] = {
		.gpio_base      = GPIO_EXT0(0),
	},

	[1] = {
		.gpio_base      = GPIO_EXT1(0),
#if defined(CONFIG_WLAN_8688_SDIO)
		.poweron = stamp_8688_wlan_poweron,
#endif
	},
	/* GPIO expander #3 */
	[2] = {
		.gpio_base      = GPIO_EXT2(0),
	},
};

#endif





#if defined(CONFIG_MMC_PXA_SDH) || defined(CONFIG_MMC_PXA_SDH_MODULE)
static mfp_cfg_t ts4700_sdh_pins[] = {

	GPIO51_MMC1_DAT3,
	GPIO52_MMC1_DAT2,
	GPIO40_MMC1_DAT1,
	GPIO41_MMC1_DAT0,
	GPIO49_MMC1_CMD,
	GPIO43_MMC1_CLK,

   MFP_CFG_DRV(GPIO33, AF6, FAST),  /* SD2_D3 */
   MFP_CFG_DRV(GPIO32, AF6, FAST),  /* SD2_D2 */
   MFP_CFG_DRV(GPIO31, AF6, FAST),  /* SD2_D1 */
   MFP_CFG_DRV(GPIO30, AF6, FAST),  /* SD2_D0 */
   MFP_CFG_DRV(GPIO28, AF6, FAST),  /* SD2_CMD */
   MFP_CFG_DRV(GPIO118, AF4, FAST),
   
   GPIO26_GPIO
};


static int sdh_mfp_config(void)
{	   
       
	mfp_config(ARRAY_AND_SIZE(ts4700_sdh_pins));

   if ((model & 0x4710) == 0x4710
      || (model & 0x4740) == 0x4740
      || (model & 0x4720) == 0x4720) {
	   volatile unsigned long *p = 
	      (volatile unsigned long*)(APB_VIRT_BASE + 0x19000);
	  
	   /* Enable power to the offboard eMMC chip (471x, not 4700) */
	   p[0x0c / 4] |= (1 << 26);
	   p[0x24 / 4] = (1 << 26);
	}
	
   return 0;
}

static int sdh_mfp_unconfig(void)
{	          

   if ((model & 0x4710) == 0x4710
      || (model & 0x4740) == 0x4740
      || (model & 0x4720) == 0x4720) {
	   volatile unsigned long *p = 
	      (volatile unsigned long*)(APB_VIRT_BASE + 0x19000);
	      	      
	   /* Disable power to the offboard eMMC chip (471x, not 4700) */
	   p[0x18 / 4] = (1 << 26);
	}
	
   return 0;
}


static struct pxasdh_platform_data ts4700_sdh_platform_data_MMC1 = {
	.detect_delay	= 20,
	.ocr_mask	= MMC_VDD_29_30 | MMC_VDD_30_31,
	.mfp_config	= sdh_mfp_config,
	.mfp_unconfig	= sdh_mfp_unconfig,
	.bus_width	= 4,
};

#if defined(CONFIG_BT_HCIUART) && defined(CONFIG_WLAN_8688_SDIO)
static mfp_cfg_t ts4700_bt_uart_pins[] = {
	GPIO98_UART_SOUT,
	GPIO99_UART_SIN,
	GPIO100_UART_RTS,
	GPIO101_UART_CTS,
};

static void bt_uart_mfp_config(void)
{
	mfp_config(ARRAY_AND_SIZE(ts4700_bt_uart_pins));
	return;
}

static void __init ts4700_bt_init(void)
{
	bt_uart_mfp_config();
}

#endif



#if defined(CONFIG_WLAN_8688_SDIO)
static int stamp_8688_wlan_poweron(void)
{
	int gpio_power = 0;
	int gpio_reset = 0;
	int gpio_wake = 0;
	int gpio_h_wake = 0;

	gpio_power = GPIO_EXT1(5);
	gpio_reset = GPIO_EXT1(6);
	gpio_wake = GPIO_EXT1(8);
	gpio_h_wake = GPIO_EXT1(7);

	if (gpio_request(gpio_power, "8688 wlan power down")) {
		printk(KERN_INFO "gpio %d request failed\n", gpio_power);
		return -1;
	}

	if(gpio_request(gpio_reset, "8688 wlan reset")) {
		printk(KERN_INFO "gpio %d request failed\n", gpio_reset);
		gpio_free(gpio_power);
		return -1;
	}
	if(gpio_request(gpio_wake, "8688 wlan gpio_wake")) {
		printk(KERN_INFO "gpio %d request failed\n", gpio_wake);
		gpio_free(gpio_power);
		gpio_free(gpio_reset);
		return -1;
	}
	if(gpio_request(gpio_h_wake, "8688 wlan card gpio_h_wake")) {
		printk(KERN_INFO "gpio %d request failed\n", gpio_h_wake);
		gpio_free(gpio_power);
		gpio_free(gpio_reset);
		gpio_free(gpio_wake);
		return -1;
	}

	gpio_direction_output(gpio_power, 0);
	gpio_direction_output(gpio_reset, 0);
	gpio_direction_output(gpio_wake, 0);
	gpio_direction_input(gpio_h_wake);
	mdelay(500);
	gpio_direction_output(gpio_reset, 1);
	gpio_direction_output(gpio_power, 1);
	gpio_direction_output(gpio_wake, 1);

	gpio_free(gpio_power);
	gpio_free(gpio_reset);
	gpio_free(gpio_wake);
	gpio_free(gpio_h_wake);
	return 0;
}

static struct pxasdh_platform_data ts4700_sdh_platform_data_MMC2 = {
	.detect_delay	= 20,
	.ocr_mask	= MMC_VDD_29_30 | MMC_VDD_30_31,
	.bus_width	= 4,
};

#endif
#if defined(CONFIG_MMC3)
static struct pxasdh_platform_data ts4700_sdh_platform_data_MMC3 = {
	.detect_delay	= 20,
	.ocr_mask	= MMC_VDD_29_30 | MMC_VDD_30_31,
	.bus_width	= 8,
};
#endif
#endif


#ifdef CONFIG_USB_GADGET_PXA_U2O

static int ts4700_u2o_vbus_status(unsigned int base)
{
	int status = VBUS_LOW;
        if (u2o_get(base, U2xOTGSC) & U2xOTGSC_BSV)
                status = VBUS_HIGH;
        else
                status = VBUS_LOW;

        return status;

}

static irqreturn_t ts4700_u2o_vbus_event(int irq, void *(func)(int))
{
	if (func)
		func(1);

	return IRQ_HANDLED;
}

static int ts4700_u2o_vbus_detect(void *func, int enable)
{
	return 0;
}

static int ts4700_u2o_vbus_set(int vbus_type)
{
	return 0;
}
static int ts4700_otg_init(void)
{
	return 0;
}

static int ts4700_u2o_vbus_set_ic(int function)
{
	printk(KERN_DEBUG "%s %d not implemented yet\n", __func__, function);
	return 0;
}

static struct otg_pmic_ops ts4700_otg_ops = {
	.otg_vbus_init          = ts4700_otg_init,
	.otg_set_vbus           = ts4700_u2o_vbus_set,
	.otg_set_vbus_ic        = ts4700_u2o_vbus_set_ic,
	.otg_get_vbus_state     = ts4700_u2o_vbus_status,
};

struct otg_pmic_ops *init_ts4700_otg_ops(void)
{
	return &ts4700_otg_ops;
}

static struct pxa_usb_plat_info ts4700_u2o_info = {
	.phy_init	= pxa168_usb_phy_init,
	.phy_deinit	= pxa168_usb_phy_deinit,
	.vbus_set	= ts4700_u2o_vbus_set,
	.vbus_status	= ts4700_u2o_vbus_status,
	.vbus_detect    = ts4700_u2o_vbus_detect,
	.init_pmic_ops	= (void*)init_ts4700_otg_ops,
#ifdef CONFIG_USB_OTG
	.is_otg		= 1,
#else
	.clk_gating	= 1,
#endif
};
#endif

#ifdef CONFIG_USB_EHCI_PXA_U2H
/* USB 2.0 Host Controller */
static int ts4700_u2h_vbus_set (int enable)
{

   printk(KERN_INFO "ts4700_u2h_vbus_set() not useful on ts4700\n");
   return 0;

}

static struct pxa_usb_plat_info ts4700_u2h_info = {
	.phy_init	= pxa168_usb_phy_init,
	.vbus_set	= ts4700_u2h_vbus_set,
};
#endif

#if defined(CONFIG_SPI_PXA2XX)
static struct pxa2xx_spi_master pxa_ssp_master_info = {
	.num_chipselect = 1,
};
#endif


#if (0)
static inline void pxa168_add_rtc(void)
{
	int ret;   // why no device for the pxa168...?
	ret = platform_device_register(&pxa910_device_rtc);
	if (ret)
		dev_err(&pxa910_device_acipc.dev,
			"unable to register device: %d\n", ret);
}
#endif

#if defined(CONFIG_CIR)

static struct resource pxa168_resource_cir[] = {
	[0] = {
		.start	= 0xD4019100,   // GPIO 96 through 122
		.end	= 0xD4019100,
		.flags	= IORESOURCE_MEM,
	},

	[1] = {
		.start	= IRQ_GPIO(102),
		.end	= IRQ_GPIO(102),
		.flags	= IORESOURCE_IRQ,
		},
};

struct platform_device pxa168_device_cir = {
	.name		= "ts4700-cir",
	.resource	= pxa168_resource_cir,
	.num_resources	= ARRAY_SIZE(pxa168_resource_cir),
};
#endif


#if (defined(CONFIG_FB_PXA168) || defined(CONFIG_FB_PXA168_MODULE) || defined(CONFIG_FB_PXA168_OLD) || defined(CONFIG_FB_PXA168_OLD_MODULE))
static struct fb_videomode video_modes[] = {  	   	
   
   
   [0] = {     /* For the 7-inch 800x480 Okaya LCD RV800480T */
      .pixclock       = 30066,  /* DCLK Typ: 33.26MHz */
      .refresh        = 60,
      .xres           = 800,
      .yres           = 480,
      .hsync_len      = 50,
      .left_margin    = 50,
      .right_margin   = 70,
      .vsync_len      = 50,
      .upper_margin   = 0,
      .lower_margin   = 0,
      .sync           = FB_SYNC_VERT_HIGH_ACT | FB_SYNC_HOR_HIGH_ACT,
   },
   [1] = {     /* For the 10.4-inch 800x600 Hantronix LCD */
      .pixclock       = 25000,   /* tLVCP Typ: 25ns */
      .refresh        = 60,
      .xres           = 800,
      .yres           = 600,
      .hsync_len      = 50,
      .left_margin    = 50,
      .right_margin   = 70,
      .vsync_len      = 50,
      .upper_margin   = 0,
      .lower_margin   = 0,
      .sync           = FB_SYNC_VERT_HIGH_ACT | FB_SYNC_HOR_HIGH_ACT,
   },
   [2] = {
      .pixclock       = 16129,
      .refresh        = 60,
      .xres           = 1280,
      .yres           = 720,
      .hsync_len      = 40,
      .left_margin    = 220,
      .right_margin   = 110,
      .vsync_len      = 5,
      .upper_margin   = 20,
      .lower_margin   = 5,
      .sync           = 0,
   },
   [3] = {
      .pixclock       = 35000, /*35ns PXLCLK */
      .refresh        = 60,
      .xres           = 640,
      .yres           = 480,
      .hsync_len      = 50,
      .left_margin    = 50,
      .right_margin   = 70,
      .vsync_len      = 50,
      .upper_margin   = 0,
      .lower_margin   = 0,
      .sync           = FB_SYNC_VERT_HIGH_ACT | FB_SYNC_HOR_HIGH_ACT,
   },
   [4] = {     /* For the 10.4-inch 1024x600 boundary devices LCD */
      .pixclock       = 20000,   /* tclk 51.2MHz */
      .refresh        = 60,
      .xres           = 1024,
      .yres           = 600,
      .hsync_len      = 50,
      .left_margin    = 50,
      .right_margin   = 70,
      .vsync_len      = 50,
      .upper_margin   = 0,
      .lower_margin   = 0,
      .sync           = FB_SYNC_VERT_HIGH_ACT | FB_SYNC_HOR_HIGH_ACT,
   },
};
#endif


#if (defined(CONFIG_FB_PXA168_OLD) || defined(CONFIG_FB_PXA168_OLD_MODULE))
/* Use the old Marvell-supplied video driver from 2.6.29 */
struct pxa168fb_mach_info ts4700_lcd_info __initdata = {
	.id                     = "Graphic Frame",
	.modes                  = video_modes,
	.num_modes              = ARRAY_SIZE(video_modes),
	.pix_fmt                = PIX_FMT_RGB565,
	.io_pin_allocation_mode = PIN_MODE_DUMB_24,
	.dumb_mode              = DUMB_MODE_RGB888,
	.active                 = 1,
	.panel_rbswap		= 1,
	.pxa168fb_lcd_power     = NULL, //tpo_lcd_power,	
	.max_fb_size		= 1280 * 720 * 4 * 2,
	.invert_pixclock        = 0,
};

struct pxa168fb_mach_info ts4700_lcd_ovly_info __initdata = {
	.id                     = "Graphic Ovly",
	.modes                  = video_modes,
	.num_modes              = ARRAY_SIZE(video_modes),
	.pix_fmt                = PIX_FMT_RGB565,
	.io_pin_allocation_mode = PIN_MODE_DUMB_24,
	.dumb_mode              = DUMB_MODE_RGB888,
	.active                 = 1,
	.panel_rbswap		= 1,	
	.max_fb_size		= 1280 * 720 * 4 * 2,
	.invert_pixclock        = 0,
};

#elif (defined(CONFIG_FB_PXA168) || defined(CONFIG_FB_PXA168_MODULE))
struct pxa168fb_mach_info ts4700_lcd_info __initdata = {
	.id                     = "Graphic Frame",
	.modes                  = video_modes,
	.num_modes              = ARRAY_SIZE(video_modes),
	.pix_fmt                = PIX_FMT_RGB888PACK,
	.io_pin_allocation_mode = PIN_MODE_DUMB_24,
	.dumb_mode              = DUMB_MODE_RGB888,
	.active                 = 1,
	.panel_rbswap		= 0,
	.invert_pixclock        = 0,		
	.panel_rgb_reverse_lanes = 0,
	.invert_pix_val_ena = 0,
};

struct pxa168fb_mach_info ts4700_lcd_ovly_info __initdata = {
	.id                     = "Graphic Ovly",
	.modes                  = video_modes,
	.num_modes              = ARRAY_SIZE(video_modes),
	.pix_fmt                = PIX_FMT_RGB888PACK,
	.io_pin_allocation_mode = PIN_MODE_DUMB_24,
	.dumb_mode              = DUMB_MODE_RGB888,
	.active                 = 1,
	.panel_rbswap		       = 1,
	.invert_pixclock        = 0,
};
#endif

#if defined(CONFIG_TOUCHSCREEN_ILI210X) || defined(CONFIG_TOUCHSCREEN_ILI210X_MODULE)
static bool ili210x_pendown_state(void)
{
   unsigned int pen;
	pen = mfp_to_gpio(MFP_PIN_GPIO49);
   gpio_request(pen, "ili210x pendown irq");
	gpio_direction_input(pen);
}

static struct ili210x_platform_data ili_info = {
	.irq_flags =  0,
	.poll_period = 1,
	.get_pendown_state	= ili210x_pendown_state,
};
#endif

static struct i2c_board_info pwri2c_board_info[] = {
   {
		.type           = "nothing",
		.addr           = 0x2a,  //
	},
#if defined(CONFIG_TOUCHSCREEN_ILI210X) || defined(CONFIG_TOUCHSCREEN_ILI210X_MODULE)

	{
	    .type = "ili210x",
	    .addr = 0x41,
	    .platform_data = &ili_info,
	    .irq = IRQ_GPIO(49),
	},
	
#endif	
#if defined(CONFIG_RTC_DRV_DS1307)
   {
		.type		= "m41t00",   /* RTC */
		.addr      = (0xd0 >> 1),
	},
#endif

#if defined(CONFIG_FB_HDMI_SII9022) || defined (CONFIG_FB_HDMI_SII9022_MODULE)
   /* HDMI baseboard support (for boards with SII9022 HDMI Transmitter) */
	{
	   .type = "sii9022",
      .addr = 0x39,
      .platform_data = NULL,
   },
   
#endif

};

static struct i2c_pxa_platform_data pwri2c_info __initdata = {
	.use_pio		= 1,
};


struct irq_proc {
   unsigned long irq;
   wait_queue_head_t q;
   atomic_t count;
   char devname[TASK_COMM_LEN];
};

static irqreturn_t irq_proc_irq_handler(int irq, void *vidp)
{
   struct irq_proc *idp = (struct irq_proc *)vidp;

   BUG_ON(idp->irq != irq);

   disable_irq_nosync(irq);
   atomic_inc(&idp->count);

   wake_up(&idp->q);
   return IRQ_HANDLED;
}

#if (0)  //FIX ME!
/*
 * Signal to userspace an interrupt has occured.
 */
static ssize_t irq_proc_read(struct file *filp, char  __user *bufp, size_t len, loff_t *ppos)
{
   struct irq_proc *ip = (struct irq_proc *)filp->private_data;
   struct irq_desc *idp = irq_desc + ip->irq;
   int pending;

   DEFINE_WAIT(wait);

   if (len < sizeof(int))
      return -EINVAL;

   pending = atomic_read(&ip->count);
   if (pending == 0) {
      if (idp->status & IRQ_DISABLED)
         enable_irq(ip->irq);
      if (filp->f_flags & O_NONBLOCK)
         return -EWOULDBLOCK;
   }

   while (pending == 0) {
      prepare_to_wait(&ip->q, &wait, TASK_INTERRUPTIBLE);
      pending = atomic_read(&ip->count);
      if (pending == 0)
         schedule();
      finish_wait(&ip->q, &wait);
      if (signal_pending(current))
         return -ERESTARTSYS;
   }

   if (copy_to_user(bufp, &pending, sizeof pending))
      return -EFAULT;

   *ppos += sizeof pending;

   atomic_sub(pending, &ip->count);
   return sizeof pending;
}


static int irq_proc_open(struct inode *inop, struct file *filp)
{
   struct irq_proc *ip;
   struct proc_dir_entry *ent = PDE(inop);
   int error;

   ip = kmalloc(sizeof *ip, GFP_KERNEL);
   if (ip == NULL)
      return -ENOMEM;

   memset(ip, 0, sizeof(*ip));
   strcpy(ip->devname, current->comm);
   init_waitqueue_head(&ip->q);
   atomic_set(&ip->count, 0);
   ip->irq = (unsigned long)ent->data;

   error = request_irq(ip->irq,
             irq_proc_irq_handler,
             0,
             ip->devname,
             ip);
   if (error < 0) {
      kfree(ip);
      return error;
   }
   filp->private_data = (void *)ip;

   return 0;
}

static int irq_proc_release(struct inode *inop, struct file *filp)
{
   struct irq_proc *ip = (struct irq_proc *)filp->private_data;

   free_irq(ip->irq, ip);
   filp->private_data = NULL;
   kfree(ip);
   return 0;
}

static unsigned int irq_proc_poll(struct file *filp, struct poll_table_struct *wait)
{
   struct irq_proc *ip = (struct irq_proc *)filp->private_data;
   struct irq_desc *idp = irq_desc + ip->irq;

   if (atomic_read(&ip->count) > 0)
      return POLLIN | POLLRDNORM; /* readable */

   /* if interrupts disabled and we don't have one to process... */
   if (idp->status & IRQ_DISABLED)
      enable_irq(ip->irq);

   poll_wait(filp, &ip->q, wait);

   if (atomic_read(&ip->count) > 0)
      return POLLIN | POLLRDNORM; /* readable */

   return 0;
}

static struct file_operations irq_proc_file_operations = {
   .read = irq_proc_read,
   .open = irq_proc_open,
   .release = irq_proc_release,
   .poll = irq_proc_poll,
};

#undef MAX_NAMELEN
#define MAX_NAMELEN 32


static void ts4700_create_proc_irq(void)
{
   char name [MAX_NAMELEN];
   unsigned int irq;
   struct irq_desc *desc;

 /*
   * Create handles for user-mode interrupt handlers
   * if the kernel hasn't already grabbed the IRQ
   */

   for_each_irq_desc(irq, desc) {
      if (!desc)
         continue;

      if(irq >= NR_IRQS)
         break;

      sprintf(name, "irq/%d/irq", irq);

      proc_create_data(name, 0600, NULL,
            &irq_proc_file_operations, (void *)(long)irq);

   }
}


#endif

#define peek16(adr) (vreg[(adr)/2])
#define poke16(adr, val) (vreg[(adr)/2] = (val))

void ts4700_restart(enum reboot_mode mode, const char *cmd)
{
	volatile unsigned short *vreg = (unsigned short *)TS47XX_FPGA_VIRT_BASE;

	local_irq_disable();
	peek16(0x6) = 0x0; // Set the watchdog to 0.338s
	while(1){};
}

void ts4700_readbaseboard(volatile unsigned short *vreg)
{
	unsigned short prev1, prev2, prev3, prev4;
	unsigned int i, x;    
	tsBaseBoard = 0;         

	prev1 = peek16(0x4);
	prev2 = peek16(0x12);
	prev3 = peek16(0x1a);
	prev4 = peek16(0x10);
	poke16(0x4, 0); /* disable muxbus */
	poke16(0x10, prev4 & ~0x20);
	poke16(0x1a, prev3 | 0x200);
	for(i=0; i<8; i++) {
		x = prev2 & ~0x1a00;
		if (!(i & 1)) x |= 0x1000;
		if (!(i & 2)) x |= 0x0800;
		if (i & 4) x |= 0x0200;
		poke16(0x12, x);
		udelay(1);
		tsBaseBoard = (tsBaseBoard >> 1);
		if (peek16(0x20) & 0x20) tsBaseBoard |= 0x80;
	}
	poke16(0x4, prev1);
	poke16(0x12, prev2);
	poke16(0x1a, prev3);
	poke16(0x10, prev4);
	/* Only lower 6 bits are model; upper 2 bits are Rev. */
	tsBaseBoard &= 0x3F;
	
}

static void ts4700_readcpumodel(void)
{
	volatile unsigned long *mvgpioregs;
	mvgpioregs = (unsigned long*)(APB_VIRT_BASE + 0x19000);
	mvgpioregs[0x10c/4] &= ~(1 << 25); 
	if(mvgpioregs[0x100/4] & (1 << 25)) {
		cpuModel = 0x166;
	} else {
		cpuModel = 0x168;
	}
}
#if (0)
static void __devinit i210_pci_fixup(struct pci_dev *dev)
{

   printk("%s\n", __func__);
		dev->resource[2].start = 0;
		dev->resource[2].end   = 0;
		dev->resource[2].flags = 0;
	
}
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL, 1532, i210_pci_fixup);
#endif


#if defined(CONFIG_USB_EHCI_MV)
static struct mv_usb_platform_data pxa168_sph_pdata = {
	.mode           = MV_USB_MODE_HOST,
	.phy_init	= pxa_usb_phy_init,
	.phy_deinit	= pxa_usb_phy_deinit,
	.set_vbus	= NULL,
};
#endif


static void __init ts4700_init(void)
{   
	int baseboardHasLCD;
	volatile unsigned short *vreg = (unsigned short *)TS47XX_FPGA_VIRT_BASE;

	mfp_config(ARRAY_AND_SIZE(ts4700_pin_config));
	pxa168_add_uart(1);
	
	model = peek16(0x0);
	switch (model) {
	case 0x4700:
		cpuModel = 0x166;
		ts4700_readbaseboard(vreg);
		enable_pcie = 0;
		break;
	case 0x7250:
	case 0x7700:
	case 0x7750:
		ts4700_readcpumodel();
		enable_pcie = 0;
		break;
	default:
		ts4700_readcpumodel();
		ts4700_readbaseboard(vreg);
		if(cpuModel == 0x166)
			enable_pcie = 0;
		else
			enable_pcie = 1;
	}

	if(cpuModel == 0x166) printk("CPU: pxa166\n");
	else if(cpuModel == 0x168) printk("CPU: pxa168\n");

	printk("Model: 0x%04X\nBaseboard: ", model);      
	switch(tsBaseBoard) {
	case 1:  printk("TS-8395\n"); break;
	case 2:  printk("TS-8390\n"); break;
	case 5:  printk("TS-8400\n"); break;
	case 10: printk("TS-8900\n"); break;
	case 11: printk("TS-8290\n"); break;
	case 15: printk("TS-8380\n"); break;      
	case 17: printk("TS-8920\n"); break;
	default: printk("Unknown\n");
	}

	switch(tsBaseBoard) {
	case 17:     /* TS-8920*/
	case 10:    /* TS-8900 */
	case 5:     /* TS-8400 */	
#if (defined(CONFIG_FB_PXA168_OLD) ||  \
     defined(CONFIG_FB_PXA168_OLD_MODULE) || \
     defined(CONFIG_FB_PXA168) || \
     defined(CONFIG_FB_PXA168_MODULE))	      
		ts4700_lcd_info.invert_pixclock = 1;
#endif      
	case 1:     /* TS-8395 */
	case 2:     /* TS-8390 */	
	case 11:    /* TS-8290 */
	case 15:    /* TS-8380 */      
		baseboardHasLCD = 1;
	break;
	default:
		baseboardHasLCD = 0;
	}

	/* on-chip devices */

	pxa168_add_eth(&pxa168_eth_data);
	
	pxa168_add_ssp(0);
	pxa168_add_twsi(1, &pwri2c_info, ARRAY_AND_SIZE(pwri2c_board_info));

#ifdef CONFIG_USB_GADGET_PXA_U2O
	pxa168_add_u2o(&ts4700_u2o_info);
#endif

#ifdef CONFIG_USB_OTG
	pxa168_add_u2ootg(&ts4700_u2o_info);
	pxa168_add_u2oehci(&ts4700_u2o_info);
#endif


#if defined(CONFIG_PCI) || defined(CONFIG_PCI_TS47XX)
	if (enable_pcie) {
		printk("CPU: Enabling PCIe\n");
		pxa168_add_pcie(&pxa168_pcie_data);
	} else {
		printk("CPU: Disabling PCIe\n");
	}
#endif

#if defined(CONFIG_USB_EHCI_MV)
	pxa168_add_usb_host(&pxa168_sph_pdata);
#endif


#if defined(CONFIG_MMC_PXA_SDH) || defined(CONFIG_MMC_PXA_SDH_MODULE)
	pxa168_add_sdh(1, &ts4700_sdh_platform_data_MMC1);

#endif

	
#if (defined(CONFIG_FB_PXA168_OLD) || defined(CONFIG_FB_PXA168_OLD_MODULE) || defined(CONFIG_FB_PXA168) || defined(CONFIG_FB_PXA168_MODULE))	
	if (baseboardHasLCD) {
		pxa168_add_fb(&ts4700_lcd_info);
		//pxa168_add_fb_ovly(&ts4700_lcd_ovly_info);

#if (defined(CONFIG_TOUCHSCREEN_TSLCD) || defined(CONFIG_TOUCHSCREEN_TSLCD_MODULE))
		pxa_register_device(&pxa168_device_tslcd, 0, 0); 
#endif	   
	}
#endif

	//ts4700_create_proc_irq();
}


MACHINE_START(TS47XX, "ts471x board")
	.map_io		= mmp_map_io,
	.nr_irqs	= MMP_NR_IRQS,
	.init_irq       = pxa168_init_irq,
	.init_time	= pxa168_timer_init,
	.init_machine   = ts4700_init,
	.restart	= ts4700_restart,
MACHINE_END


