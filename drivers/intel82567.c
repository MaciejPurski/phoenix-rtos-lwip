/*
 * Phoenix-RTOS --- networking stack
 *
 * Intel 82567V NIC driver
 *
 * Copyright 2020 Phoenix Systems
 * Author: Maciej Purski
 *
 * %LICENSE%
 */

#include "arch/cc.h"
#include "lwip/etharp.h"
#include "netif-driver.h"
#include "physmmap.h"
#include "bdring.h"
#include "pci.h"
#include "res-create.h"
#include "rtl8139cp-regs.h"
#include "intel82567.h"

#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/ethip6.h"
#include "lwip/etharp.h"
#include "netif/ppp/pppoe.h"



#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <unistd.h>
#include <sys/threads.h>
#include <sys/interrupt.h>

#define USE_DMA64 (sizeof(addr_t) > 4)
#define RTL_RING_BUF_SIZE	0x1000	/* RX + TX */
#define RTL_RX_RING_SIZE	64
#define RTL_TX_RING_SIZE	64
#define DEBUG_RINGS	0

typedef struct
{
	// volatile struct rtl_regs *mmio;
	struct netif *netif;
	void *mmio;
	int bartype;

#define PRIV_RESOURCES(s) &(s)->irq_lock, 5, ~0x07
	handle_t irq_lock, rx_lock, tx_lock, rx_irq_cond, tx_irq_cond, rx_irq_handle, tx_irq_handle;
	unsigned drv_exit;

	net_bufdesc_ring_t rx, tx;

	uint16_t devnum;

	uint32_t rx_stack[2048], tx_stack[2048], irq_stack[256];
} intel82567_priv_t;

/* FROM OSDEV BEGIN */
#define E1000_NUM_RX_DESC 32
#define E1000_NUM_TX_DESC 8
 
typedef struct {
        uint64_t addr;
        uint16_t length;
        uint16_t checksum;
        uint8_t status;
        uint8_t errors;
        uint16_t special;
} intel82567_rx_desc_t __attribute__((packed));
 
typedef struct {
        uint64_t addr;
        uint16_t length;
        uint8_t cso;
        uint8_t cmd;
        uint8_t status;
        uint8_t css;
        uint16_t special;
} intel82567_tx_desc_t  __attribute__((packed));
/* FROM OSDEV END */


static void intel82567_write(intel82567_priv_t *ctx, uint16_t reg, uint32_t value)
{
	*((volatile uint32_t *)(ctx->mmio + reg)) = value;
}

static uint32_t intel82567_read(intel82567_priv_t *ctx, uint16_t reg)
{
	return *((volatile uint32_t *)(ctx->mmio + reg));
}

// static void rtl_printf(intel82567_priv_t *state, const char *format, ...)
// {
// 	char buf[256];
// 	va_list arg;

// 	va_start(arg, format);
// 	vsnprintf(buf, sizeof(buf), format, arg);
// 	va_end(arg);

// 	printf("PCI " PCI_DEVNUM_FMT ": %s\n", PCI_DEVNUM_ARGS(state->devnum), buf);
// }


// static void rtl_chipReset(intel82567_priv_t *state)
// {
// 	/* trigger and wait for reset */
// 	state->mmio->CR = RTL_CMD_RESET;
// 	while (state->mmio->CR & RTL_CMD_RESET)
// 		usleep(100);

// 	/* enable C+ mode */
// 	state->mmio->CPCR = RTL_CMD_TX_MODE_CP|RTL_CMD_RX_MODE_CP|RTL_CMD_RX_CSUM|RTL_CMD_RX_VLAN|RTL_CMD_PCI_MULRW;

// 	/* clear RX multicast filter */
// 	state->mmio->MAR[0] = 0;
// 	state->mmio->MAR[1] = 0;
// }


// static void rtl_readCardMac(intel82567_priv_t *state)
// {
// 	uint32_t buf[2];

// 	// XX: cpu_to_le(), need dword access
// 	buf[0] = state->mmio->IDR[0];
// 	buf[1] = state->mmio->IDR[1];

// 	memcpy(&state->netif->hwaddr, buf, ETH_HWADDR_LEN);
// }


// static void rtl_showCardId(intel82567_priv_t *state)
// {
// 	uint32_t tc, rc;
// 	uint8_t *mac;

// 	tc = state->mmio->TCR;
// 	rc = state->mmio->RCR;
// 	rtl_printf(state, "HW ver-id %03x dma-burst: tx %u rx %u",
// 		(tc >> 20) & 0x7cc,
// 		16 << ((tc & RTL_TX_DMA_BURST) >> RTL_TX_DMA_BURST_SHIFT),
// 		16 << ((rc & RTL_RX_DMA_BURST) >> RTL_RX_DMA_BURST_SHIFT));

// 	mac = (void *)&state->netif->hwaddr;
// 	rtl_printf(state, "MAC: %02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
// }


static size_t intel82567_nextRxBufferSize(const net_bufdesc_ring_t *ring, size_t i)
{
	//printf("nextRxBufferSize\n");
	volatile intel82567_rx_desc_t *r = ring->ring;

	/* Todo: filter */
	return r[i].length;
}


static int intel82567_pktRxFinished(const net_bufdesc_ring_t *ring, size_t i)
{
	volatile intel82567_rx_desc_t *r = ring->ring;

	if (!(r[i].status & RSTAT_DD))
		return 0;

	return r[i].status & RSTAT_EOP;
}


static void intel82567_fillTxDesc(const net_bufdesc_ring_t *ring, size_t i, addr_t pa, size_t sz, unsigned seg)
{
	//printf("fillDesc\n");
	volatile intel82567_tx_desc_t *r = ring->ring;
	// uint32_t cmd = sz;

	r[i].addr = pa;
	r[i].length = sz;
	r[i].cmd = CMD_EOP | CMD_RS;
// 	if (seg & BDRING_SEG_FIRST)
// 		cmd |= RTL_DESC_FS;
// 	if (seg & BDRING_SEG_LAST)
// 		cmd |= RTL_DESC_LS;

// 	if (i == ring->last)
// 		cmd |= RTL_DESC_EOR;
// 	cmd |= RTL_DESC_OWN;

// 	if (USE_DMA64)
// 		r[i].addr.h = USE_DMA64 ? pa >> 32 : 0;
// 	r[i].addr.l = pa;
// 	asm volatile ("" ::: "memory");
// 	r[i].cmd = cmd;
}


static void intel82567_fillRxDesc(const net_bufdesc_ring_t *ring, size_t i, addr_t pa, size_t sz, unsigned seg)
{
	//printf("fillRxDesc\n");
	volatile intel82567_rx_desc_t *r = ring->ring;

	r[i].addr = pa;
	r[i].length = sz;
	/* TODO: not 0? */
	r[i].status = 0;
	//rtl_fillDesc(ring, i, pa + ETH_PAD_SIZE, sz - ETH_PAD_SIZE, 0);
}

static int intel82567_nextTxDone(const net_bufdesc_ring_t *ring, size_t i)
{
	volatile rtl_buf_desc_t *r = ring->ring;

	return r[i].cmd & RSTAT_DD;
}


static const net_bufdesc_ops_t intel82567_ring_ops = {
	intel82567_nextRxBufferSize,
	intel82567_pktRxFinished,
	intel82567_fillRxDesc,
	intel82567_nextTxDone,
	intel82567_fillTxDesc,

	/* desc_size */		sizeof(intel82567_rx_desc_t),
	/* ring_alignment */	64,
	/* pkt_buf_sz */	8192 - 64,
	/* max_tx_frag */	TXCMD_SZ_MASK,
};

static const size_t intel82567_ring_sz[] = { E1000_NUM_RX_DESC, E1000_NUM_TX_DESC };


static int intel82567_initRings(intel82567_priv_t *state)
{
	int err;

	err = net_initRings(&state->rx, intel82567_ring_sz, 2, &intel82567_ring_ops);
	if (err)
		return err;

	net_refillRx(&state->rx, 0);
 
	/* Rx buffer */
    intel82567_write(state, REG_RDBAL0, state->rx.phys);
	intel82567_write(state, REG_RDBAH0, USE_DMA64 ? state->rx.phys >> 32 : 0);
 
    intel82567_write(state, REG_RDLEN0, E1000_NUM_RX_DESC * 16);
 
    intel82567_write(state, REG_RDH0, 0);
    intel82567_write(state, REG_RDT0, E1000_NUM_RX_DESC - 1);

	/* Tx buffer */
    intel82567_write(state, REG_TDBAL0, state->rx.phys);
	intel82567_write(state, REG_TDBAH0, USE_DMA64 ? state->rx.phys >> 32 : 0);
 
    intel82567_write(state, REG_TDLEN0, E1000_NUM_TX_DESC * 16);
 
    intel82567_write(state, REG_TDH0, 0);
    intel82567_write(state, REG_TDT0, E1000_NUM_TX_DESC - 1);

	return 0;
}


// /* IRQ: RX */


static int rtl_rx_irq_handler(unsigned irq, void *arg)
{
	intel82567_priv_t *state = arg;
	uint32_t icr = intel82567_read(state, REG_ICR);
	printf("intel IRQ\n");
	if (!(icr & IMS_RXO))
		return -1;

	//__sync_fetch_and_and(&state->mmio->IMR, ~RTL_INT_RX);

	printf("intel82567 RXO IRQ\n");
	return 0;
}


static void rtl_rx_irq_thread(void *arg)
{
	intel82567_priv_t *state = arg;
	size_t rx_done;

	mutexLock(state->irq_lock);
	while (1) {
		intel82567_write(state, REG_ICS, IMS_RXO);
		//state->mmio->ISR = RTL_INT_RX;
		mutexUnlock(state->irq_lock);

		rx_done = net_receivePackets(&state->rx, state->netif, 0);
		if (rx_done || !net_rxFullyFilled(&state->rx))
			net_refillRx(&state->rx, 0);
		printf("RX thread work \n");
		mutexLock(state->irq_lock);
		if (!(intel82567_read(state, REG_ICR) & IMS_RXO)) {
			//__sync_fetch_and_or(&state->mmio->IMR, RTL_INT_RX);
			//printf("cond wait\n");
			condWait(state->rx_irq_cond, state->irq_lock, 0);
		}
	}
	mutexUnlock(state->irq_lock);

	endthread();
}


// /* IRQ: TX */

static int rtl_tx_irq_handler(unsigned irq, void *arg)
{
	intel82567_priv_t *state = arg;
	uint32_t icr = intel82567_read(state, REG_ICR);

	if (!(icr & IMS_TXDW))
		return -1;

	// __sync_fetch_and_and(&state->mmio->IMR, ~RTL_INT_TX);
	return 0;
}


static void rtl_tx_irq_thread(void *arg)
{
	intel82567_priv_t *state = arg;
	size_t tx_done;

	mutexLock(state->irq_lock);
	while (1) {
		intel82567_write(state, REG_ICS, IMS_TXDW);
		//state->mmio->ISR = RTL_INT_TX;
		mutexUnlock(state->irq_lock);

		tx_done = net_reapTxFinished(&state->tx);
		tx_done = 0;
 		mutexLock(state->irq_lock);
		if (!tx_done) {
 			//__sync_fetch_and_or(&state->mmio->IMR, RTL_INT_TX);
 			condWait(state->tx_irq_cond, state->irq_lock, 0);
 		}
 	}
 	mutexUnlock(state->irq_lock);

 	endthread();
}


static int intel82567_eepromExists(intel82567_priv_t *ctx)
{
	uint32_t val = 0;
	int i;
    intel82567_write(ctx, REG_EEPROM, 0x1); 
 
    for (i = 0; i < 10000; i++) {
            val = intel82567_read(ctx, REG_EEPROM);
            if (val & 0x10)
                    return 1;
    }

    return 0;
}

static int intel82567_readMAC(intel82567_priv_t *ctx)
{
	uint32_t macl, mach;

	macl = intel82567_read(ctx, REG_RAL);
	mach = intel82567_read(ctx, REG_RAH);

	memcpy(ctx->netif->hwaddr, &macl, 4);
	memcpy(ctx->netif->hwaddr + 4, &mach, 2);
	ctx->netif->hwaddr_len = 6;

	printf("MAC address: ");
	for (int i = 0; i < 6; i++) {
		printf("%x", ctx->netif->hwaddr[i]);
		if (i != 5)
			printf(":");
	}
	printf("\n");

	return 0;
}

static int intel82567_hwreset(intel82567_priv_t *ctx)
{
	uint32_t ctrl, reg;

	/* Mask all interrupts */
	intel82567_write(ctx, REG_IMC, 0xffffffff);

	/* Disable the Transmit and Receive units.  Then delay to allow
	 * any pending transactions to complete before we hit the MAC
	 * with the global reset.
	 */
	/* TODO */
	// intel82567_write(REG_RCTL, 0);
	// intel82567_write(REG_TCTL, TCTL_PSP);
	
	// e1e_flush();

	// usleep_range(10000, 11000);

	// /* Set Tx and Rx buffer allocation to 8k apiece. */
	// intel82567_write(ctx, REG_PBA, 0x0008);
	// /* Set Packet Buffer Size to 16k. */
	// intel82567_write(ctx, REG_PBS, 0x0010);

	// if (hw->mac.type == e1000_pchlan) {
	// 	/* Save the NVM K1 bit setting */
	// 	ret_val = e1000_read_nvm(hw, E1000_NVM_K1_CONFIG, 1, &kum_cfg);
	// 	if (ret_val)
	// 		return ret_val;

	// 	if (kum_cfg & E1000_NVM_K1_ENABLE)
	// 		dev_spec->nvm_k1_enabled = true;
	// 	else
	// 		dev_spec->nvm_k1_enabled = false;
	// }

	ctrl = intel82567_read(ctx, REG_CTRL);
	intel82567_write(ctx, REG_CTRL, (ctrl | CTRL_SWRST | CTRL_LCD_RST));
	/* Documentation requires at lest 15 ms wait */
	usleep(20000);

	/* Disable interrupts again */
	intel82567_write(ctx, REG_IMC, 0xffffffff);
	intel82567_read(ctx, REG_ICR);
	return 0;
}

static int intel82567_initConfig(intel82567_priv_t *ctx)
{
	/* Set full duplex and speed */
	return 0;
}

/* Establish link between MAC and PHY */
static int intel82567_linkSetup(intel82567_priv_t *ctx)
{
	uint32_t ctrl = intel82567_read(ctx, REG_CTRL);

	/* MAC settings automatically based on duplex and speed resolved by PHY */
	if (ctrl & CTRL_FRCDPLX == 0 &&
		ctrl & CTRL_FRCSPD) {
			/* TODO */
	}

	return 0;
}

static int intel82567_initStatcnts(intel82567_priv_t *ctx)
{
	/* TODO */
	return 0;
}


static int intel82567_interruptsEnable(intel82567_priv_t *ctx)
{
	//intel82567_write(ctx, REG_IMS, IMS_RXT0 | IMS_TXDW |
	//							   IMS_RXDMT0 | IMS_LSC);

	intel82567_write(ctx, REG_IMS, 0xffffffff);
	
	/* Flush device */
	intel82567_read(ctx, REG_STATUS);

	return 0;
}

static err_t intel82567_netifOutput(struct netif *netif, struct pbuf *p)
{
	intel82567_priv_t *state = netif->state;
	size_t nf;

	if (ETH_PAD_SIZE != 2)
		pbuf_header(p, -ETH_PAD_SIZE); /* drop the padding word */

	mutexLock(state->tx_lock);
	nf = net_transmitPacket(&state->tx, p);
	printf("Transmit packet: %d\n", nf);
	// if (nf)
	// 	state->mmio->TDAR = ~0u;
	mutexUnlock(state->tx_lock);

	return nf ? ERR_OK : ERR_BUF;
}


static int intel82567_initDevice(intel82567_priv_t *ctx, uint16_t devnum, int irq)
{
	uint32_t crev;
	int err, i;
	uint32_t val;

	ctx->devnum = devnum;
	val = pci_configRead(devnum, 0);

	printf("vid: %x pid: %x\n", val & 0xFFFF, val >> 16);

	ctx->mmio = pci_mapMemBAR(devnum, 0);
	if (!ctx->mmio)
		return -ENOMEM;

	printf("intel82567: memory mapped: %p\n", ctx->mmio);
	val = intel82567_eepromExists(ctx);
	printf("intel82567: EEPROM exists %d\n", val);

	if ((err = create_mutexcond_bulk(PRIV_RESOURCES(ctx))))
		return err;

	pci_setBusMaster(devnum, 1);
	intel82567_hwreset(ctx);
	intel82567_initConfig(ctx);
	intel82567_linkSetup(ctx);
	intel82567_initStatcnts(ctx);

	intel82567_readMAC(ctx);

	/* Clear Multicast Table Array */
	for (i = 0; i < 0x80; i++)
        intel82567_write(ctx, REG_MTA + i*4, 0);
	
	if ((err = intel82567_initRings(ctx) != EOK))
		goto err_exit;

	beginthread(rtl_rx_irq_thread, 0, (void *)ctx->rx_stack, sizeof(ctx->rx_stack), ctx);
	beginthread(rtl_tx_irq_thread, 0, (void *)ctx->tx_stack, sizeof(ctx->tx_stack), ctx);
	interrupt(irq, rtl_rx_irq_handler, ctx, ctx->rx_irq_cond, &ctx->rx_irq_handle);
	interrupt(irq, rtl_tx_irq_handler, ctx, ctx->tx_irq_cond, &ctx->tx_irq_handle);

	/* Enable RCTL from OSDEV */
	intel82567_write(ctx, REG_RCTL, RCTL_EN | RCTL_SBP| RCTL_UPE | RCTL_MPE | RCTL_LBM_NONE | RTCL_RDMTS_HALF | RCTL_BAM | RCTL_SECRC  | RCTL_BSIZE_8192);
	
	/* Enable TCTL from OSDEV */
	intel82567_write(ctx, REG_TCTL,  0b0110000000000111111000011111010);
    intel82567_write(ctx, REG_TIPG,  0x0060200A);
	
	intel82567_interruptsEnable(ctx);
	printf("intel82567: interrupts enable\n");
// 	crev = pci_configRead(devnum, 8) & 0xFF;
// 	if (crev < 20) {
// 		rtl_printf(state, "error: card does not support C+ mode");
// 		return -ENOTTY;
// 	}

// 	if ((err = create_mutexcond_bulk(PRIV_RESOURCES(state))))
// 		return err;

// 	rtl_chipReset(state);
// 	pci_setBusMaster(devnum, 1);
// 	rtl_readCardMac(state);
// 	rtl_showCardId(state);



// 	beginthread(rtl_rx_irq_thread, 0, (void *)state->rx_stack, sizeof(state->rx_stack), state);
// 	beginthread(rtl_tx_irq_thread, 0, (void *)state->tx_stack, sizeof(state->tx_stack), state);
// 	interrupt(irq, rtl_rx_irq_handler, state, state->rx_irq_cond, &state->rx_irq_handle);
// 	interrupt(irq, rtl_tx_irq_handler, state, state->tx_irq_cond, &state->tx_irq_handle);

// 	state->mmio->CR = RTL_CMD_RX_ENABLE | RTL_CMD_TX_ENABLE;
// 	state->mmio->RCR = (4 << RTL_RX_DMA_BURST_SHIFT) | RTL_RX_FTH | RTL_RX_BCAST | RTL_RX_MCAST | RTL_RX_UCAST;
// 	state->mmio->IMR = RTL_INT_RX | RTL_INT_TX;

// 	return EOK;

err_exit:
// 	rtl_chipReset(state);
// 	return err;
	return EOK;
}


// static err_t rtl_netifOutput(struct netif *netif, struct pbuf *p)
// {
// 	intel82567_priv_t *state = netif->state;
// 	size_t nf;
// 	int do_unref = 0;

// 	if (ETH_PAD_SIZE)
// 		pbuf_header(p, -ETH_PAD_SIZE); /* drop the padding word */

// 	if (p->tot_len < 60) {
// 		struct pbuf *q = pbuf_alloc(PBUF_RAW, 60 + ETH_PAD_SIZE, PBUF_RAM);
// 		pbuf_header(q, -ETH_PAD_SIZE);
// 		pbuf_copy(q, p);
// 		p = q;
// 		do_unref = 1;
// 	}

// 	mutexLock(state->tx_lock);
// 	nf = net_transmitPacket(&state->tx, p);
// 	if (nf)
// 		state->mmio->TPPOLL = RTL_POLL_NPQ;
// 	mutexUnlock(state->tx_lock);

// 	if (do_unref)
// 		pbuf_free(p);

// 	return nf ? ERR_OK : ERR_BUF;
// }


static int intel82567_netifInit(struct netif *netif, char *cfg)
{
	intel82567_priv_t *priv;
	unsigned devnum;
	char *p;
	int irq;
	unsigned i;

	netif->linkoutput = intel82567_netifOutput;

	printf("intel82567: init\n");
	priv = netif->state;
	priv->netif = netif;

	if (!cfg)
		return ERR_ARG;

	devnum = strtoul(cfg, &p, 0);
	if (!*cfg || *p++ != ':' || devnum > 0xFFFF)
		return ERR_ARG;

	irq = strtoul((cfg = p), &p, 0);
	if (!*cfg || *p || irq < 0)
		return ERR_ARG;

	printf("devnum: %x irq: %d\n", devnum, irq);

	return intel82567_initDevice(priv, devnum, irq);
}


static netif_driver_t intel82567_drv = {
	.init = intel82567_netifInit,
	.state_sz = sizeof(intel82567_priv_t),
	.state_align = _Alignof(intel82567_priv_t),
	.name = "intel82567",
};


__constructor__(1000)
void register_driver_intel82567(void)
{
	printf("intel82567: registered\n");
	register_netif_driver(&intel82567_drv);
}
