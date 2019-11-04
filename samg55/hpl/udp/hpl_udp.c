/**
 * \file
 *
 * \brief SAM UDP HPL
 *
 * Copyright (c) 2016-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */

#include <hpl_usb_device.h>
#include <peripheral_clk_config.h>

#define REF_NOP_COUNT 20
#define REF_CPU_FREQUENCY 48000000

//! Bitmap for all status bits in CSR that are not affected by a value 1.
#define UDP_REG_NO_EFFECT_1_ALL                                                                                        \
	(UDP_CSR_RX_DATA_BK0 | UDP_CSR_RX_DATA_BK1 | UDP_CSR_STALLSENT | UDP_CSR_RXSETUP | UDP_CSR_TXCOMP)
/**
 * \brief Calculate actural nop count
 */
static inline uint32_t _udp_get_nop_count(void)
{
#if CONF_CPU_FREQUENCY <= 12000000
	return ((uint32_t)CONF_CPU_FREQUENCY * REF_NOP_COUNT / REF_CPU_FREQUENCY)
	           ? ((uint32_t)CONF_CPU_FREQUENCY * REF_NOP_COUNT / REF_CPU_FREQUENCY)
	           : 1;
#else
	return ((CONF_CPU_FREQUENCY / 1000) * REF_NOP_COUNT / (REF_CPU_FREQUENCY / 1000))
	           ? ((CONF_CPU_FREQUENCY / 1000) * REF_NOP_COUNT / (REF_CPU_FREQUENCY / 1000))
	           : 1;
#endif
}

/*! Sets specified bit(s) in the UDP_CSR.
 * \param ep Endpoint number.
 * \param bits Bitmap to set to 1.
 */
#define udp_set_csr(ep, bits)                                                                                          \
	do {                                                                                                               \
		volatile uint32_t nop_count;                                                                                   \
		hri_udp_set_CSR_reg(UDP, ep, UDP_REG_NO_EFFECT_1_ALL | (bits));                                                \
		for (nop_count = 0; nop_count < _udp_get_nop_count(); nop_count++) {                                           \
			__NOP();                                                                                                   \
		}                                                                                                              \
	} while (0)

/*! Clears specified bit(s) in the UDP_CSR.
 * \param ep Endpoint number.
 * \param bits Bitmap to set to 0.
 */
#define udp_clear_csr(ep, bits)                                                                                        \
	do {                                                                                                               \
		volatile uint32_t nop_count;                                                                                   \
		hri_udp_set_CSR_reg(UDP, ep, UDP_REG_NO_EFFECT_1_ALL);                                                         \
		hri_udp_clear_CSR_reg(UDP, ep, (bits));                                                                        \
		for (nop_count = 0; nop_count < _udp_get_nop_count(); nop_count++) {                                           \
			__NOP();                                                                                                   \
		}                                                                                                              \
	} while (0)

/** Dual bank feature of endpoints */
static const bool _is_dual_bank[] = {false, true, true, false, true, true};
/** Max packet size feature of endpoints */
static const uint16_t _max_ep_size[] = {64, 128, 128, 64, 512, 512};

/** Check if endpoint is dual bank */
#define udp_ep_is_dual_bank(epn) _is_dual_bank[epn]
/** Check if endpoint support ISO */
#define udp_ep_support_iso(epn) _is_dual_bank[epn]
/** Return endpoint max packet size */
#define udp_ep_max_pkt_siz(epn) _max_ep_size[epn]

/* For G55, the max endpoint number is 5 (total 6 physical endpoints) */
#if CONF_USB_D_MAX_EP_N > 5
#warning Max endpoint number too large, please check your settings
#endif

/** Number of endpoints supported. */
#define USB_D_N_EP (CONF_USB_D_MAX_EP_N + 1)

/** HPL USB device endpoint struct. */
struct _usb_d_dev_ep {
	/** Pointer to transaction buffer. */
	uint8_t *trans_buf;
	/** Transaction size. */
	uint32_t trans_size;
	/** Transaction transferred count. */
	uint32_t trans_count;

	/** Endpoint max packet size */
	uint16_t pkt_size : 12;
	/** Endpoint type (for UDP) */
	uint16_t ep_type : 4;
	/** Endpoint address (transaction) */
	uint8_t ep;
	/** Current bank */
	uint8_t bank : 2;
	/** Endpoint busy in transfer */
	uint8_t busy : 1;
	/** Endpoint stall requested */
	uint8_t stall_req : 1;
	/** Endpoint ZLP requested */
	uint8_t zlp_req : 1;
	/** The buffer reach its end but still things in FIFO */
	uint8_t buf_end : 1;
};

/** HPL USB device struct. */
struct _usb_d_dev {
	/** Callbacks of USB device. */
	struct _usb_d_dev_callbacks callbacks;
	/** Endpoint transaction callbacks. */
	struct _usb_d_dev_ep_callbacks ep_callbacks;
	/** Endpoints. */
	struct _usb_d_dev_ep ep[USB_D_N_EP];
};

/** USB device instance. */
struct _usb_d_dev dev_inst;

/**
 * \brief Finish the transaction and invoke callback
 * \param[in, out] ept Pointer to endpoint information.
 * \param[in] code Information code passed.
 */
static void _usb_d_dev_trans_done(struct _usb_d_dev_ep *ept, const int32_t code);

/**
 *  \brief Terminate the transaction with specific status code
 * \param[in] epn Endpoint number.
 * \param[in] dir Endpoint direction.
 * \param[in] code Information code passed.
 */
static void _usb_d_dev_trans_stop(uint8_t epn, bool dir, const int32_t code);

/**
 * \brief Commit IN packets
 * \param[in] epn     Endpoint number
 * \param[in] tx_exec Start transmit
 * \return \c true if data packet pending
 */
static bool _usb_d_dev_in_sent(uint8_t epn, bool tx_exec);

/**
 * \brief Dummy callback function
 * \return Always false.
 */
static bool _dummy_func_no_return(uint32_t unused0, uint32_t unused1)
{
	(void)unused0;
	(void)unused1;
	return false;
}

/**
 * \brief Reset all endpoint software instances
 */
static void _usb_d_dev_reset_epts(void)
{
	uint8_t i;

	for (i = 0; i < USB_D_N_EP; i++) {
		_usb_d_dev_trans_done(&dev_inst.ep[i], USB_TRANS_RESET);
		dev_inst.ep[i].ep      = 0xFF;
		dev_inst.ep[i].ep_type = 0;
		dev_inst.ep[i].bank    = 0;
	}
}

/**
 * \brief Handles USB SOF interrupt
 */
static inline void _usb_d_dev_sof(void)
{
	/* ACK SOF interrupt. */
	hri_udp_write_ICR_reg(UDP, UDP_ICR_SOFINT);

	dev_inst.callbacks.sof();
}

/**
 * \brief Handles USB resume/wakeup interrupts
 */
static inline void _usb_d_dev_wakeup(void)
{
	dev_inst.callbacks.event(USB_EV_WAKEUP, 0);

	hri_udp_write_ICR_reg(UDP, UDP_ICR_WAKEUP | UDP_ICR_RXRSM | UDP_ICR_EXTRSM);
	hri_udp_clear_IMR_reg(UDP, UDP_IDR_WAKEUP | UDP_IDR_RXRSM | UDP_IDR_EXTRSM);

	hri_udp_write_ICR_reg(UDP, UDP_ICR_RXSUSP);
	hri_udp_set_IMR_reg(UDP, UDP_IER_RXSUSP | UDP_IER_SOFINT);
}

/**
 * \brief Handles USB suspend interrupt
 */
static inline void _usb_d_dev_suspend(void)
{
	hri_udp_write_ICR_reg(UDP, UDP_ICR_RXSUSP);
	hri_udp_clear_IMR_reg(UDP, UDP_IDR_RXSUSP);
	hri_udp_set_IMR_reg(UDP, UDP_IER_WAKEUP | UDP_IER_RXRSM | UDP_IER_EXTRSM);

	/* Callback */
	dev_inst.callbacks.event(USB_EV_SUSPEND, 0);
}

/**
 * \brief Handles USB signal reset interrupt
 */
static inline void _usb_d_dev_reset(void)
{
	hri_udp_write_ICR_reg(UDP, UDP_ICR_ENDBUSRES);
	hri_udp_set_IMR_reg(UDP, UDP_IER_RXSUSP | UDP_IER_SOFINT);

	_usb_d_dev_reset_epts();

	dev_inst.callbacks.event(USB_EV_RESET, 0);
}

/**
 * \brief Handles IN interrupt
 * \param epn Endpoint number
 * \param ept Pointer to endpoint instance
 */
static inline void _usb_d_dev_handle_ep_in(uint8_t epn, struct _usb_d_dev_ep *ept)
{
	/* One bank must sent */
	ept->bank--;
	/* Stall when all banks free */
	if (ept->stall_req) {
		if (ept->bank) {
			/* Send remaining */
			udp_set_csr(epn, UDP_CSR_TXPKTRDY);
			udp_clear_csr(epn, UDP_CSR_TXCOMP);
		} else {
			/* Ack last packet */
			udp_clear_csr(epn, UDP_CSR_TXCOMP);
			/* Enable stall */
			udp_set_csr(epn, UDP_CSR_FORCESTALL);
			/* Halt executed */
			ept->stall_req = 0;
		}
		return;
	}
	/* Ask more when buffer end */
	if (ept->buf_end) {
		ept->buf_end = false;
		ept->busy    = 0;
		if (dev_inst.ep_callbacks.more(ept->ep, ept->trans_count)) {
			/* More data added */
			return;
		}
		ept->busy = 1;
	}
	/* All things done, including ZLP */
	if (ept->trans_count >= ept->trans_size && !ept->zlp_req && ept->bank == 0) {
		udp_clear_csr(epn, UDP_CSR_TXCOMP);
		if (ept->ep_type != 0) {
			/* Disable interrupt for none-control endpoints */
			hri_udp_clear_IMR_reg(UDP, UDP_IDR_EP0INT << epn);
		}
		_usb_d_dev_trans_done(ept, USB_TRANS_DONE);
	} else if (udp_ep_is_dual_bank(epn) && ept->bank) {
		/* Bank already buffered, transmit while loading */
		udp_set_csr(epn, UDP_CSR_TXPKTRDY);
		udp_clear_csr(epn, UDP_CSR_TXCOMP);
		_usb_d_dev_in_sent(epn, false);
	} else if (udp_ep_is_dual_bank(epn)) {
		/* Still bank free, load and transmit */
		if (!_usb_d_dev_in_sent(epn, true)) {
			ept->buf_end = false;
			udp_clear_csr(epn, UDP_CSR_TXCOMP);
			if (ept->ep_type != 0) {
				/* Disable interrupt for none-control endpoints */
				hri_udp_clear_IMR_reg(UDP, UDP_IDR_EP0INT << epn);
			}
			_usb_d_dev_trans_done(ept, USB_TRANS_DONE);
			return;
		}
		udp_clear_csr(epn, UDP_CSR_TXCOMP);
		_usb_d_dev_in_sent(epn, false);
	} else {
		/* Single bank transfer, ack when ready */
		_usb_d_dev_in_sent(epn, true);
		udp_clear_csr(epn, UDP_CSR_TXCOMP);
	}
}

/**
 * \brief Handles ep setup
 * \param epn Endpoint number
 * \param ept Pointer to endpoint instance
 */
static void _usb_d_dev_handle_ep_setup(uint8_t epn, struct _usb_d_dev_ep *ept)
{
	if (ept->ep_type != 0) {
		/* None control! Never here */
		udp_clear_csr(epn, UDP_CSR_RXSETUP);
		return;
	}
	/* Control transfer:
	 * SETUP transaction will terminate IN/OUT transaction,
	 * and start new transaction with received SETUP packet.
	 * - Will be done after setup is read
	 */
	/* Control transfer:
	 * Ack SETUP.
	 * - Will be done after setup is read
	 */
	/* Invoke callback. */
	dev_inst.ep_callbacks.setup(ept->ep);
}

/**
 * \brief Ack the OUT packet
 * \param epn Endpoint number
 * \param csr CSR register value
 */
static void _usb_d_dev_ack_out(uint8_t epn, struct _usb_d_dev_ep *ept, uint32_t csr)
{
	bool bk0 = (csr & UDP_CSR_RX_DATA_BK0), bk1 = (csr & UDP_CSR_RX_DATA_BK1);

	if (bk0 && bk1) {
		/* The only way is to use ept->bank */
	} else if (bk0) {
		/* Must be bank0 */
		ept->bank = 0;
	} else if (bk1) {
		/* Must be bank1 */
		ept->bank = 1;
	}
	if (ept->bank == 0) {
		udp_clear_csr(epn, UDP_CSR_RX_DATA_BK0);
		if (udp_ep_is_dual_bank(epn)) {
			ept->bank = 1;
		}
	} else {
		udp_clear_csr(epn, UDP_CSR_RX_DATA_BK1);
		ept->bank = 0;
	}
}

/**
 * \brief Handles OUT interrupt
 * \param epn Endpoint number
 * \param ept Pointer to endpoint instance
 * \param csr CSR flags
 */
static inline void _usb_d_dev_handle_ep_out(uint8_t epn, struct _usb_d_dev_ep *ept, uint32_t csr)
{
	uint16_t n_bytes   = (csr & UDP_CSR_RXBYTECNT_Msk) >> UDP_CSR_RXBYTECNT_Pos;
	uint32_t n_remain  = ept->trans_size - ept->trans_count;
	uint8_t *dst       = &ept->trans_buf[ept->trans_count];
	bool     short_pkt = (n_bytes < ept->pkt_size), full = false;

	/* Copy data if there is */
	if (n_bytes > 0) {
		uint16_t i;

		if (n_bytes >= n_remain) {
			n_bytes = n_remain;
			full    = true;
		}
		ept->trans_count += n_bytes;
		for (i = 0; i < n_bytes; i++) {
			*dst++ = hri_udp_read_FDR_reg(UDP, epn);
		}
	} else if ((csr & (UDP_CSR_DIR | UDP_CSR_EPTYPE_Msk)) == UDP_CSR_DIR && ept->trans_size > ept->trans_count) {
		/* Abort control IN transaction */
		_usb_d_dev_ack_out(epn, ept, csr);
		_usb_d_dev_trans_stop(epn, true, USB_TRANS_DONE);
		return;
	}
	/* Clear FIFO status */
	_usb_d_dev_ack_out(epn, ept, csr);
	/* Finish job on error or short packet */
	if ((full || short_pkt) && !(csr & UDP_CSR_FORCESTALL)) {
		if (ept->ep_type != 0) {
			/* Disable interrupt for none-control endpoints */
			hri_udp_clear_IMR_reg(UDP, UDP_IDR_EP0INT << epn);
		}
		_usb_d_dev_trans_done(ept, USB_TRANS_DONE);
	}
}

/**
 * \brief Handles USB endpoint interrupts
 * \return \c true if endpoint handled
 */
static inline bool _usb_d_dev_handle_ep(uint8_t epn)
{
	struct _usb_d_dev_ep *ept = &dev_inst.ep[epn];
	uint32_t              csr = hri_udp_read_CSR_reg(UDP, epn);

	/* RXSETUP: Setup packet received */
	if (csr & UDP_CSR_RXSETUP) {
		_usb_d_dev_handle_ep_setup(epn, ept);
		return true;
	}
	/* RXOUT: Full packet received */
	if (csr & (UDP_CSR_RX_DATA_BK0 | UDP_CSR_RX_DATA_BK1)) {
		_usb_d_dev_handle_ep_out(epn, ept, csr);
		return true;
	}
	/* TXIN: packet sent */
	if (csr & UDP_CSR_TXCOMP) {
		_usb_d_dev_handle_ep_in(epn, ept);
		return true;
	}
	/* Stall sent */
	if (csr & UDP_CSR_STALLSENT) {
		udp_clear_csr(epn, UDP_CSR_STALLSENT);
		_usb_d_dev_trans_done(ept, USB_TRANS_STALL);
		return true;
	}

	return false;
}

/**
 * \brief USB device interrupt handler
 */
void UDP_Handler(void)
{
	uint32_t isr = hri_udp_read_ISR_reg(UDP);
	uint32_t imr = hri_udp_read_IMR_reg(UDP);
	uint8_t  i;
	isr &= imr;

	/* SOF interrupt */
	if (isr & UDP_ISR_SOFINT) {
		_usb_d_dev_sof();
		return;
	}

	/* Endpoint interrupts */
	if (isr & ((1 << USB_D_N_EP) - 1)) {
		for (i = 0; i < USB_D_N_EP; i++) {
			if (isr & (UDP_ISR_EP0INT << i)) {
				if (_usb_d_dev_handle_ep(i)) {
					/* Handle just one endpoint once */
					break;
				}
			}
		}
		return;
	}

	/* Wakeup interrupt */
	if (isr & (UDP_ISR_WAKEUP | UDP_ISR_RXRSM | UDP_ISR_EXTRSM)) {
		_usb_d_dev_wakeup();
		return;
	} else if (isr & UDP_ISR_RXSUSP) {
		_usb_d_dev_suspend();
		return;
	} else if (isr & UDP_ISR_ENDBUSRES) {
		_usb_d_dev_reset();
		return;
	}
}

int32_t _usb_d_dev_init(void)
{
	if (!hri_udp_get_TXVC_reg(UDP, UDP_TXVC_TXVDIS)) {
		/* if transceiver is enabled */
		return -USB_ERR_DENIED;
	}

	dev_inst.callbacks.sof   = (_usb_d_dev_sof_cb_t)_dummy_func_no_return;
	dev_inst.callbacks.event = (_usb_d_dev_event_cb_t)_dummy_func_no_return;

	dev_inst.ep_callbacks.setup = (_usb_d_dev_ep_cb_setup_t)_dummy_func_no_return;
	dev_inst.ep_callbacks.more  = (_usb_d_dev_ep_cb_more_t)_dummy_func_no_return;
	dev_inst.ep_callbacks.done  = (_usb_d_dev_ep_cb_done_t)_dummy_func_no_return;

	/* USB Device mode & Transceiver active */
	hri_matrix_write_CCFG_USBMR_reg(MATRIX, CCFG_USBMR_USBMODE);

	return USB_OK;
}

void _usb_d_dev_deinit()
{
	/* Disable transceiver and pull-up */
	hri_udp_write_TXVC_reg(UDP, UDP_TXVC_TXVDIS);
	NVIC_DisableIRQ(UDP_IRQn);
	NVIC_ClearPendingIRQ(UDP_IRQn);

	/* USB Device mode & Transceiver deactive */
	hri_matrix_write_CCFG_USBMR_reg(MATRIX, 0);
}

void _usb_d_dev_register_callback(const enum usb_d_cb_type type, const FUNC_PTR func)
{
	FUNC_PTR f = (func == NULL) ? (FUNC_PTR)_dummy_func_no_return : (FUNC_PTR)func;
	if (type == USB_D_CB_EVENT) {
		dev_inst.callbacks.event = (_usb_d_dev_event_cb_t)f;
	} else if (type == USB_D_CB_SOF) {
		dev_inst.callbacks.sof = (_usb_d_dev_sof_cb_t)f;
	}
}

void _usb_d_dev_register_ep_callback(const enum usb_d_dev_ep_cb_type type, const FUNC_PTR func)
{
	FUNC_PTR f = (func == NULL) ? (FUNC_PTR)_dummy_func_no_return : (FUNC_PTR)func;
	if (type == USB_D_DEV_EP_CB_SETUP) {
		dev_inst.ep_callbacks.setup = (_usb_d_dev_ep_cb_setup_t)f;
	} else if (type == USB_D_DEV_EP_CB_MORE) {
		dev_inst.ep_callbacks.more = (_usb_d_dev_ep_cb_more_t)f;
	} else if (type == USB_D_DEV_EP_CB_DONE) {
		dev_inst.ep_callbacks.done = (_usb_d_dev_ep_cb_done_t)f;
	}
}

int32_t _usb_d_dev_enable(void)
{
	NVIC_EnableIRQ(UDP_IRQn);
	return USB_OK;
}

int32_t _usb_d_dev_disable(void)
{
	/* Disable transceiver and pull-up */
	hri_udp_write_TXVC_reg(UDP, UDP_TXVC_TXVDIS);
	NVIC_DisableIRQ(UDP_IRQn);
	return USB_OK;
}

void _usb_d_dev_attach(void)
{
	hri_udp_write_TXVC_reg(UDP, UDP_TXVC_PUON);
}

void _usb_d_dev_detach(void)
{
	/* Disable transceiver and pull-up */
	hri_udp_write_TXVC_reg(UDP, UDP_TXVC_TXVDIS);
}

void _usb_d_dev_send_remotewakeup(void)
{
	/* Initialize remote wakeup */
	hri_udp_set_GLB_STAT_ESR_bit(UDP);
	hri_udp_clear_GLB_STAT_ESR_bit(UDP);
}

enum usb_speed _usb_d_dev_get_speed()
{
	return USB_SPEED_FS;
}

void _usb_d_dev_set_address(const uint8_t addr)
{
	if (addr) {
		hri_udp_write_FADDR_reg(UDP, UDP_FADDR_FEN | UDP_FADDR_FADD(addr));
		hri_udp_write_GLB_STAT_reg(UDP, UDP_GLB_STAT_FADDEN);
	} else {
		hri_udp_clear_FADDR_FEN_bit(UDP);
		/* GLB_STAT could not be cleared except reception of RESET */
	}
}

uint8_t _usb_d_dev_get_address(void)
{
	if (hri_udp_get_GLB_STAT_reg(UDP, UDP_GLB_STAT_FADDEN)) {
		return hri_udp_get_FADDR_FADD_bf(UDP, 0xFF);
	}
	return 0;
}

uint16_t _usb_d_dev_get_frame_n(void)
{
	return hri_udp_get_FRM_NUM_FRM_NUM_bf(UDP, 0x3FF);
}

uint8_t _usb_d_dev_get_uframe_n(void)
{
	return 0;
}

int32_t _usb_d_dev_ep0_init(const uint8_t max_pkt_siz)
{
	return _usb_d_dev_ep_init(0, USB_EP_XTYPE_CTRL, max_pkt_siz);
}

int32_t _usb_d_dev_ep_init(const uint8_t ep, const uint8_t attr, uint16_t max_pkt_siz)
{
	uint8_t epn     = USB_EP_GET_N(ep);
	bool    dir     = USB_EP_GET_DIR(ep);
	uint8_t ep_type = attr & USB_EP_XTYPE_MASK;
	bool    iso     = (ep_type == USB_EP_XTYPE_ISOCH);

	if (epn > CONF_USB_D_MAX_EP_N) {
		return -USB_ERR_PARAM;
	}
	if (udp_ep_max_pkt_siz(epn) < max_pkt_siz) {
		return -USB_ERR_PARAM;
	}
	if (iso && !udp_ep_support_iso(epn)) {
		return -USB_ERR_ALLOC_FAIL;
	}
	if (dev_inst.ep[epn].ep != 0xFF) {
		return -USB_ERR_REDO;
	}
	if (dir) {
		ep_type |= 0x4;
	}
	dev_inst.ep[epn].ep       = ep;
	dev_inst.ep[epn].ep_type  = ep_type;
	dev_inst.ep[epn].pkt_size = max_pkt_siz;
	hri_udp_write_CSR_reg(UDP, epn, UDP_CSR_EPTYPE(ep_type));

	return USB_OK;
}

void _usb_d_dev_ep_deinit(const uint8_t ep)
{
	uint8_t epn = USB_EP_GET_N(ep);
	bool    dir = USB_EP_GET_DIR(ep);

	if (epn > CONF_USB_D_MAX_EP_N) {
		return;
	}
	if (dev_inst.ep[epn].ep == 0xFF) {
		return;
	}
	/* Finish pending transactions */
	_usb_d_dev_trans_stop(epn, dir, USB_TRANS_RESET);

	/* Disable the endpoint interrupts */
	hri_udp_clear_IMR_reg(UDP, epn << UDP_IMR_EP0INT);
	/* Disable the endpoint */
	hri_udp_write_CSR_reg(UDP, epn, 0);
	dev_inst.ep[epn].ep = 0xFF;
}

int32_t _usb_d_dev_ep_enable(const uint8_t ep)
{
	uint8_t epn = USB_EP_GET_N(ep);

	if (epn > CONF_USB_D_MAX_EP_N) {
		return -USB_ERR_PARAM;
	}
	/* Physically enable the endpoint */
	udp_set_csr(epn, UDP_CSR_EPEDS);
	/* Enable interrupt for default endpoint 0 */
	if (epn == 0) {
		hri_udp_set_IMR_reg(UDP, UDP_IMR_EP0INT);
	}

	return USB_OK;
}

void _usb_d_dev_ep_disable(const uint8_t ep)
{
	uint8_t epn = USB_EP_GET_N(ep);
	bool    dir = USB_EP_GET_DIR(ep);

	if (epn > CONF_USB_D_MAX_EP_N) {
		return;
	}
	/* Stop transfer */
	_usb_d_dev_trans_stop(epn, dir, USB_TRANS_RESET);

	/* Disable */
	udp_clear_csr(epn, UDP_CSR_EPEDS);

	return;
}

/**
 * \brief Set endpoint stall
 * \param[in] epn Endpoint number
 */
static inline int32_t _usb_d_dev_ep_stall_set(const uint8_t epn)
{
	uint32_t csr               = hri_udp_read_CSR_reg(UDP, epn);
	dev_inst.ep[epn].stall_req = 1;

	if ((csr & 0x4) && ((csr & UDP_CSR_TXPKTRDY) || dev_inst.ep[epn].bank > 1)) {
		/* IN: stall after bank sent */
	} else {
		udp_set_csr(epn, UDP_CSR_FORCESTALL);
	}
	/* Wait interrupt on endpoint */
	hri_udp_set_IMR_reg(UDP, UDP_IMR_EP0INT << epn);

	return 1;
}

/**
 * \brief Clear endpoint stall
 * \param[in] epn Endpoint number
 */
static inline int32_t _usb_d_dev_ep_stall_clr(const uint8_t epn)
{
	uint32_t csr = hri_udp_read_CSR_reg(UDP, epn);

	if (csr & (UDP_CSR_FORCESTALL | UDP_CSR_STALLSENT)) {
		/* Remove stall */
		udp_clear_csr(epn, UDP_CSR_FORCESTALL);
		/* Reset FIFO and data toggle */
		if (epn) {
			hri_udp_set_RST_EP_reg(UDP, UDP_RST_EP_EP0 << epn);
			hri_udp_clear_RST_EP_reg(UDP, UDP_RST_EP_EP0 << epn);
		}
		/* Clear stall status */
		udp_clear_csr(epn, UDP_CSR_STALLSENT);
	}
	dev_inst.ep[epn].stall_req = 0;

	return 0;
}

/**
 * \brief Return endpoint stalled status
 *
 * \param[in] epn Endpoint number
 * \retval 1 Endpoint is stalled
 * \retval 0 Endpoint is not stalled
 */
static inline int32_t _usb_d_dev_ep_stall_get(const uint8_t epn)
{
	if (dev_inst.ep[epn].stall_req) {
		return 1;
	}
	if (hri_udp_get_CSR_reg(UDP, epn, UDP_CSR_FORCESTALL | UDP_CSR_STALLSENT)) {
		return 1;
	}

	return 0;
}

int32_t _usb_d_dev_ep_stall(const uint8_t ep, const enum usb_ep_stall_ctrl ctrl)
{
	uint8_t epn = USB_EP_GET_N(ep);

	if (epn > CONF_USB_D_MAX_EP_N) {
		return -USB_ERR_PARAM;
	}

	if (ctrl == USB_EP_STALL_SET) {
		return _usb_d_dev_ep_stall_set(epn);
	} else if (ctrl == USB_EP_STALL_CLR) {
		return _usb_d_dev_ep_stall_clr(epn);
	} else {
		return _usb_d_dev_ep_stall_get(epn);
	}
}

int32_t _usb_d_dev_ep_read_req(const uint8_t ep, uint8_t *req_buf)
{
	uint8_t               epn = USB_EP_GET_N(ep);
	uint16_t              n, i;
	uint32_t              csr = hri_udp_read_CSR_reg(UDP, epn);
	struct _usb_d_dev_ep *ept = &dev_inst.ep[epn];

	if (epn > CONF_USB_D_MAX_EP_N || !req_buf) {
		return -USB_ERR_PARAM;
	}
	if ((csr & UDP_CSR_EPTYPE_Msk) != UDP_CSR_EPTYPE_CTRL) {
		return ERR_DENIED;
	}
	if (csr & UDP_CSR_RXSETUP) {
		n = (csr & UDP_CSR_RXBYTECNT_Msk) >> UDP_CSR_RXBYTECNT_Pos;
		for (i = 0; i < n; i++) {
			req_buf[i] = hri_udp_read_FDR_reg(UDP, epn);
		}
		/* Terminate IN/OUT transaction on SETUP */
		if (ept->busy) {
			if (csr & UDP_CSR_TXPKTRDY) {
				udp_clear_csr(epn, UDP_CSR_TXPKTRDY);
				udp_clear_csr(epn, UDP_CSR_TXCOMP);
			}
			if (csr & (UDP_CSR_RX_DATA_BK0 | UDP_CSR_RX_DATA_BK1)) {
				udp_clear_csr(epn, UDP_CSR_RX_DATA_BK0 | UDP_CSR_RX_DATA_BK1);
			}
			/* Reset FIFO */
			hri_udp_set_RST_EP_reg(UDP, UDP_RST_EP_EP0 << epn);
			hri_udp_clear_RST_EP_reg(UDP, UDP_RST_EP_EP0 << epn);
			/* Done with hidden packet */
			_usb_d_dev_trans_done(ept, USB_TRANS_DONE);
		}
		/* Set DIR according to valid request bmRequestType with wLength > 0 */
		if ((n == 8) && (req_buf[0] & 0x80) && (req_buf[6] || req_buf[7])) {
			udp_set_csr(epn, UDP_CSR_DIR);
		}
		udp_clear_csr(epn, UDP_CSR_RXSETUP);
		return n;
	}

	return 0;
}

static void _usb_d_dev_trans_done(struct _usb_d_dev_ep *ept, const int32_t code)
{
	if (!ept->busy) {
		return;
	}
	ept->busy = 0;
	dev_inst.ep_callbacks.done(ept->ep, code, ept->trans_count);
}

/**
 * \brief Fill TX FIFO
 * \param epn Endpoint number
 * \return \c true if it's short packet
 */
static bool udp_fill_fifo(uint8_t epn)
{
	struct _usb_d_dev_ep *ept       = &dev_inst.ep[epn];
	uint8_t *             src       = &ept->trans_buf[ept->trans_count];
	uint32_t              remain    = ept->trans_size - ept->trans_count;
	uint32_t              pkt_size  = ept->pkt_size;
	bool                  short_pkt = false;

	/* Packet size */
	if (remain < pkt_size) {
		pkt_size  = remain;
		short_pkt = true;
	}
	/* Modify count */
	ept->trans_count += pkt_size;
	/* Speed block data transfer to FIFO */
	for (; pkt_size >= 8; pkt_size -= 8) {
		hri_udp_write_FDR_reg(UDP, epn, *src++);
		hri_udp_write_FDR_reg(UDP, epn, *src++);
		hri_udp_write_FDR_reg(UDP, epn, *src++);
		hri_udp_write_FDR_reg(UDP, epn, *src++);

		hri_udp_write_FDR_reg(UDP, epn, *src++);
		hri_udp_write_FDR_reg(UDP, epn, *src++);
		hri_udp_write_FDR_reg(UDP, epn, *src++);
		hri_udp_write_FDR_reg(UDP, epn, *src++);
	}
	/* byte by byte transfer to FIFO */
	for (; pkt_size; pkt_size--) {
		hri_udp_write_FDR_reg(UDP, epn, *src++);
	}
	/* Added to banks */
	ept->bank++;

	return short_pkt;
}

static bool _usb_d_dev_in_sent(uint8_t epn, bool tx_exec)
{
	const uint8_t         n_bank = udp_ep_is_dual_bank(epn) ? 2 : 1;
	struct _usb_d_dev_ep *ept    = &dev_inst.ep[epn];
	bool                  short_pkt;

	/* All banks are full */
	if (ept->bank >= n_bank) {
		return true; /* Data pending */
	}
	/* No more data in buffer */
	if (ept->trans_count >= ept->trans_size && !ept->zlp_req) {
		return false;
	}
	/* Fill FIFO */
	short_pkt = udp_fill_fifo(epn);
	/* Send it */
	if (tx_exec) {
		udp_set_csr(epn, UDP_CSR_TXPKTRDY);
	}
	/* Short packet? no need to send it again */
	if (short_pkt) {
		ept->zlp_req = false;
	}
	/* All transaction done, including ZLP, finish */
	if (ept->trans_count >= ept->trans_size && !ept->zlp_req) {
		ept->buf_end = true;
		return false;
	}

	return true; /* Pending */
}

int32_t _usb_d_dev_ep_trans(const struct usb_d_transfer *trans)
{
	uint8_t               epn            = USB_EP_GET_N(trans->ep);
	bool                  dir            = USB_EP_GET_DIR(trans->ep);
	struct _usb_d_dev_ep *ept            = &dev_inst.ep[epn];
	uint16_t              size_mask      = ept->pkt_size - 1;
	bool                  size_n_aligned = (trans->size & size_mask);
	// bool is_ctrl = ept->ep_type == 0;
	volatile hal_atomic_t flags;

	if (epn > CONF_USB_D_MAX_EP_N) {
		return -USB_ERR_PARAM;
	}
	/* Check halt */
	if (ept->stall_req) {
		return USB_HALTED;
	}
	/* Try to start transaction */
	atomic_enter_critical(&flags);
	if (ept->busy) {
		atomic_leave_critical(&flags);
		return USB_BUSY;
	}

	ept->busy = 1;
	atomic_leave_critical(&flags);
	/* Copy transaction information */
	ept->ep          = trans->ep;
	ept->trans_buf   = trans->buf;
	ept->trans_size  = trans->size;
	ept->trans_count = 0;
	ept->zlp_req     = trans->zlp && (!size_n_aligned);
	ept->buf_end     = false;

	hri_udp_set_IMR_reg(UDP, UDP_IMR_EP0INT << epn);

	/* Request first transfer for IN */
	if (dir) {
		atomic_enter_critical(&flags);
		/* There is pending IN packet */
		if (hri_udp_get_CSR_reg(UDP, epn, UDP_CSR_TXPKTRDY | UDP_CSR_TXCOMP)) {
			/* Append more data (handled in interrupt service) */
		} else {
			/* Start new, try to fill 1~2 banks before handling status */
			if (_usb_d_dev_in_sent(epn, true)) {
				/* Over one bank */
				_usb_d_dev_in_sent(epn, false);
			} else {
				/* Less than one bank */
			}
		}
		atomic_leave_critical(&flags);
	} else {
		/* Waiting for OUT interrupt */
	}

	return USB_OK;
}

static void _usb_d_dev_trans_stop(uint8_t epn, bool dir, const int32_t code)
{
	struct _usb_d_dev_ep *ept = &dev_inst.ep[epn];

	/* Disable interrupts for none-control */
	if (ept->ep_type != 0) {
		hri_udp_clear_IMR_reg(UDP, UDP_IMR_EP0INT << epn);
	}
	/* Clear pending things */
	if (dir) {
		/* Kill banks */
		if (hri_udp_get_CSR_reg(UDP, epn, UDP_CSR_TXPKTRDY)) {
			if (udp_ep_is_dual_bank(epn)) {
				udp_clear_csr(epn, UDP_CSR_TXPKTRDY);
				while (hri_udp_get_CSR_reg(UDP, epn, UDP_CSR_TXPKTRDY)) {
					;
				}
				udp_set_csr(epn, UDP_CSR_TXPKTRDY);
				while (!hri_udp_get_CSR_reg(UDP, epn, UDP_CSR_TXPKTRDY)) {
					;
				}
			}
			udp_clear_csr(epn, UDP_CSR_TXPKTRDY);
		}
		udp_clear_csr(epn, UDP_CSR_TXCOMP);
		/* Reset number of bank */
		dev_inst.ep[epn].bank = 0;
	}
	/* Reset FIFO and data toggle */
	hri_udp_set_RST_EP_reg(UDP, UDP_RST_EP_EP0 << epn);
	hri_udp_clear_RST_EP_reg(UDP, UDP_RST_EP_EP0 << epn);
	/* Abort transaction */
	_usb_d_dev_trans_done(ept, USB_TRANS_ABORT);
}

void _usb_d_dev_ep_abort(const uint8_t ep)
{
	uint8_t epn = USB_EP_GET_N(ep);
	bool    dir = USB_EP_GET_DIR(ep);
	if (epn > CONF_USB_D_MAX_EP_N) {
		return;
	}
	_usb_d_dev_trans_stop(epn, dir, USB_TRANS_ABORT);
}

int32_t _usb_d_dev_ep_get_status(const uint8_t ep, struct usb_d_trans_status *stat)
{
	uint8_t               epn = USB_EP_GET_N(ep);
	struct _usb_d_dev_ep *ept = &dev_inst.ep[epn];
	uint32_t              csr = hri_udp_read_CSR_reg(UDP, epn);
	bool                  busy, stall;

	if (epn > CONF_USB_D_MAX_EP_N) {
		return -USB_ERR_PARAM;
	}
	busy  = ept->busy;
	stall = ept->stall_req;
	if (stat) {
		stat->stall = stall;
		stat->busy  = busy;
		stat->setup = csr & UDP_CSR_RXSETUP;
		stat->dir   = USB_EP_GET_DIR(ept->ep);
		stat->size  = ept->trans_size;
		stat->count = ept->trans_count;
		stat->ep    = ep;
		stat->xtype = ept->ep_type & 0x3;
	}
	if (stall) {
		return USB_HALTED;
	}
	if (busy) {
		return USB_BUSY;
	}

	return USB_OK;
}
