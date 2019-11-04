/* Auto-generated config file hpl_usb_config.h */
#ifndef HPL_USB_CONFIG_H
#define HPL_USB_CONFIG_H

// <<< Use Configuration Wizard in Context Menu >>>

#define CONF_USB_D_N_0 0
#define CONF_USB_D_N_1 1
#define CONF_USB_D_N_2 2
#define CONF_USB_D_N_3 3
#define CONF_USB_D_N_4 4
#define CONF_USB_D_N_5 5

#define CONF_USB_D_NUM_EP_SP_MAX (CONF_USB_D_MAX_EP_N + 1)

// <y> Max Endpoint Number supported
// <i> Limits the max endpoint number.
// <CONF_USB_D_N_0"> 0 (EP00)
// <CONF_USB_D_N_1"> 1 (EP01/EP81)
// <CONF_USB_D_N_2"> 2 (EP02/EP82)
// <CONF_USB_D_N_3"> 3 (EP03/EP83)
// <CONF_USB_D_N_4"> 4 (EP04/EP84)
// <CONF_USB_D_N_5"> 5 (EP05/EP85)

// <id> usbd_arch_max_ep_n
#ifndef CONF_USB_D_MAX_EP_N
#define CONF_USB_D_MAX_EP_N CONF_USB_D_N_2
#endif

// <y> Max number of endpoints supported
// <i> Limits the number of endpoints (described by EP address) can be used in app.
// <CONF_USB_D_N_1"> 1
// <CONF_USB_D_N_2"> 2
// <CONF_USB_D_N_3"> 3
// <CONF_USB_D_N_4"> 4
// <CONF_USB_D_N_5"> 5
// <CONF_USB_D_N_6"> 6

// <CONF_USB_D_NUM_EP_SP_MAX"> Max possible (by "Max Endpoint Number" config)
// <id> usbd_num_ep_sp
#ifndef CONF_USB_D_NUM_EP_SP
#define CONF_USB_D_NUM_EP_SP CONF_USB_D_NUM_EP_SP_MAX
#endif

// <<< end of configuration section >>>

#endif // HPL_USB_CONFIG_H
