#ifndef __LINUX_USB_GADGET_PXA_COMMON_H
#define __LINUX_USB_GADGET_PXA_COMMON_H

enum ep0_state {
	EP0_IDLE,
	EP0_IN_DATA_PHASE,
	EP0_OUT_DATA_PHASE,
	EP0_STALL,
	EP0_IN_FAKE,
	EP0_NO_ACTION
};

#define DMSG(stuff...)  pr_debug("udc: " stuff)



#endif /* __LINUX_USB_GADGET_PXA_COMMON_H */
