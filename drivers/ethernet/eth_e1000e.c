/*
 * Copyright (c) 2018 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_MODULE_NAME eth_e1000e
#define LOG_LEVEL CONFIG_ETHERNET_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <zephyr.h>
#include <net/ethernet.h>
#include <ethernet/eth_stats.h>
#include <pcie/pcie.h>
#include "eth_e1000e_priv.h"

static const char *e1000e_reg_to_string(enum e1000e_reg_t r)
{
#define _(_x)	case _x: return #_x
	switch (r) {
	_(CTRL);
	_(ICR);
	_(ICS);
	_(IMS);
	_(RCTL);
	_(TCTL);
	_(RDBAL);
	_(RDBAH);
	_(RDLEN);
	_(RDH);
	_(RDT);
	_(TDBAL);
	_(TDBAH);
	_(TDLEN);
	_(TDH);
	_(TDT);
	_(RAL);
	_(RAH);
	}
#undef _
	LOG_ERR("Unsupported register: 0x%x", r);
	k_oops();
	return NULL;
}

static enum ethernet_hw_caps e1000e_caps(struct device *dev)
{
	return ETHERNET_LINK_1000BASE_T;
}

static int e1000e_send(struct device *device, struct net_pkt *pkt)
{
	struct e1000e_dev *dev = device->driver_data;
	size_t len = net_pkt_get_len(pkt);
	u32_t tail = ior32(dev, TDT);
	void *txb = dev->dma_regions.txb[tail];

	if (net_pkt_read(pkt, txb, len)) {
		return -EIO;
	}

	dev->dma_regions.tx[tail].addr = POINTER_TO_INT(txb);
	dev->dma_regions.tx[tail].len = len;
	dev->dma_regions.tx[tail].cmd = TDESC_EOP | TDESC_RS;

	iow32(dev, TDT, (tail + 1) % E1000E_TX_DESC_SIZE);

	while (!(dev->dma_regions.tx[tail].sta)) {
		k_yield();
	}

	LOG_DBG("tx[%d].sta: 0x%02hx", tail, dev->dma_regions.tx[tail].sta);

	return (dev->dma_regions.tx[tail].sta & TDESC_STA_DD) ? 0 : -EIO;
}

static void e1000e_rx(struct e1000e_dev *dev)
{
	struct net_pkt *pkt = NULL;
	u32_t head = ior32(dev, RDH), tail = ior32(dev, RDT), handled = tail;

	LOG_DBG("rx tail: %d, head: %d", tail, head);
	tail = (tail + 1) % E1000E_RX_DESC_SIZE;

	do {
		volatile struct e1000e_rx *rx = dev->dma_regions.rx + tail;

		LOG_DBG("rx[%d].sta: 0x%02hx", tail, rx->sta);
		if (!(rx->sta & RDESC_STA_DD)) {
			LOG_ERR("RX descriptor not ready");
			break;
		}

		pkt = net_pkt_rx_alloc_with_buffer(dev->iface, rx->len - 4,
						   AF_UNSPEC, 0, K_NO_WAIT);
		if (!pkt) {
			LOG_ERR("Out of buffers");
			break;
		}

		if (net_pkt_write(pkt, INT_TO_POINTER((u32_t) rx->addr),
				  rx->len - 4)) {
			LOG_ERR("Out of memory for received frame");
			net_pkt_unref(pkt);
			eth_stats_update_errors_rx(dev->iface);
			break;
		}

		net_recv_data(dev->iface, pkt);

		handled = tail;
		tail = (tail + 1) % E1000E_RX_DESC_SIZE;
	} while (tail != head);

	iow32(dev, RDT, handled);
}

static void e1000e_isr(void *parameter)
{
	struct device *device = (struct device *)parameter;
	struct e1000e_dev *dev = device->driver_data;
	u32_t icr = ior32(dev, ICR), icr_ack = 0;

	if (icr & ICR_TXDW) {
		icr &= ~ICR_TXDW;
		icr_ack |= ICR_TXDW;
	}

	if (icr & ICR_TXQE) {
		icr &= ~ICR_TXQE;
		icr_ack |= ICR_TXQE;
	}

	if (icr & ICR_LSC) {
		icr &= ~ICR_LSC;
		icr_ack |= ICR_LSC;
	}

	if (icr & (ICR_RXO | ICR_RXT0)) {
		e1000e_rx(dev);
		icr_ack |= (icr & (ICR_RXO | ICR_RXT0));
		icr &= ~(ICR_RXO | ICR_RXT0);
	}

	if (icr & ~ICR_ASSERTED) {
		LOG_ERR("Unhandled interrupt, ICR: 0x%x", icr);
	}

	iow32(dev, ICR, icr_ack);
}

#define PCI_VENDOR_ID_INTEL	0x8086
#define PCI_DEVICE_ID_I82574L	0x10d3
const pcie_bdf_t e1000e_bdf = PCIE_BDF(0, 1, 0);

int e1000e_probe(struct device *device)
{
	struct e1000e_dev *dev = device->driver_data;
	int retval = -ENODEV;

	if (pcie_probe(e1000e_bdf, PCIE_ID(PCI_VENDOR_ID_INTEL,
			    PCI_DEVICE_ID_I82574L))) {
		dev->address = pcie_get_mbar(e1000e_bdf, 0);
		pcie_set_cmd(e1000e_bdf, PCIE_CONF_CMDSTAT_MEM |
				  PCIE_CONF_CMDSTAT_MASTER, true);
		retval = 0;
	}

	return retval;
}

static struct device DEVICE_NAME_GET(eth_e1000e);

#define ETH_E1000E_IRQ 0x60
#define ETH_E1000E_IRQ_PRIORITY 0x6
#define ETH_E1000E_IRQ_FLAGS 0

static void e1000e_init(struct net_if *iface)
{
	struct e1000e_dev *dev = net_if_get_device(iface)->driver_data;
	u32_t ral, rah;
	unsigned int vector;
	u32_t i;

	dev->iface = iface;

	/* Setup TX descriptor */

	iow32(dev, TDBAL, (u32_t) &dev->dma_regions.tx);
	iow32(dev, TDBAH, 0);
	iow32(dev, TDLEN, E1000E_TX_DESC_SIZE * sizeof(struct e1000e_tx));

	iow32(dev, TDH, 0);
	iow32(dev, TDT, 0);

	iow32(dev, TCTL, TCTL_EN);

	/* Setup RX descriptor */

	for (i = 0; i < E1000E_RX_DESC_SIZE; i++) {
		dev->dma_regions.rx[i].addr = POINTER_TO_INT(dev->dma_regions.rxb[i]);
		dev->dma_regions.rx[i].len = sizeof(dev->dma_regions.rxb[i]);
	}

	iow32(dev, RDBAL, (u32_t) &dev->dma_regions.rx);
	iow32(dev, RDBAH, 0);
	iow32(dev, RDLEN, E1000E_RX_DESC_SIZE * sizeof(struct e1000e_rx));

	iow32(dev, RDH, 0);
	iow32(dev, RDT, E1000E_RX_DESC_SIZE - 1);

	/* Setup interrupt conditions */

	iow32(dev, IMS, IMS_RXO | IMS_RXT0);

	/* Read MAC address */

	ral = ior32(dev, RAL);
	rah = ior32(dev, RAH);

	memcpy(dev->mac, &ral, 4);
	memcpy(dev->mac + 4, &rah, 2);

	ethernet_init(iface);

	net_if_set_link_addr(iface, dev->mac, sizeof(dev->mac),
				NET_LINK_ETHERNET);

	vector = irq_connect_dynamic(ETH_E1000E_IRQ,
				     ETH_E1000E_IRQ_PRIORITY,
				     e1000e_isr,
				     DEVICE_GET(eth_e1000e),
				     ETH_E1000E_IRQ_FLAGS);

	pcie_irq_enable(e1000e_bdf, ETH_E1000E_IRQ);

	iow32(dev, CTRL, CTRL_SLU); /* Set link up */

	iow32(dev, RCTL, RCTL_EN | RCTL_MPE | RCTL_BAM);

	LOG_DBG("done");
}

static struct e1000e_dev e1000e_dev;

static const struct ethernet_api e1000e_api = {
	.iface_api.init		= e1000e_init,
	.get_capabilities	= e1000e_caps,
	.send			= e1000e_send,
};

NET_DEVICE_INIT(eth_e1000e,
		"ETH_0",
		e1000e_probe,
		&e1000e_dev,
		NULL,
		CONFIG_ETH_INIT_PRIORITY,
		&e1000e_api,
		ETHERNET_L2,
		NET_L2_GET_CTX_TYPE(ETHERNET_L2),
		NET_ETH_MTU);
