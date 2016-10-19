/*
 *
 *  Bluetooth HCI UART H4 driver with Nokia Extensions
 *
 *  Copyright (C) 2015 Marcel Holtmann <marcel@holtmann.org>
 *  Copyright (C) 2016 Sebastian Reichel <sre@kernel.org>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */

#include <linux/module.h>

#include <linux/clk.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/interrupt.h>
#include <linux/ptrace.h>
#include <linux/poll.h>
#include <linux/pm_runtime.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/signal.h>
#include <linux/ioctl.h>
#include <linux/skbuff.h>
#include <linux/delay.h>
#include <linux/platform_device.h>

#include <linux/gpio/consumer.h>

#include <linux/unaligned/le_struct.h>
#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>

#include "hci_uart.h"

#define NOKIA_ID_CSR		0x02
#define NOKIA_ID_BCM2048	0x04
#define NOKIA_ID_TI1271		0x31

#define FIRMWARE_CSR		"nokia/bc4fw.bin"
#define FIRMWARE_BCM2048	"nokia/bcmfw.bin"
#define FIRMWARE_TI1271		"nokia/ti1273.bin"

#define NOKIA_BCM_BDADDR	0xfc01

#define HCI_NOKIA_NEG_PKT	0x06
#define HCI_NOKIA_ALIVE_PKT	0x07
#define HCI_NOKIA_RADIO_PKT	0x08

#define HCI_NOKIA_NEG_HDR_SIZE		1
#define HCI_NOKIA_MAX_NEG_SIZE		255
#define HCI_NOKIA_ALIVE_HDR_SIZE	1
#define HCI_NOKIA_MAX_ALIVE_SIZE	255
#define HCI_NOKIA_RADIO_HDR_SIZE	2
#define HCI_NOKIA_MAX_RADIO_SIZE	255

#define NOKIA_PROTO_PKT		0x44
#define NOKIA_PROTO_BYTE	0x4c

#define NOKIA_NEG_REQ		0x00
#define NOKIA_NEG_ACK		0x20
#define NOKIA_NEG_NAK		0x40

#define H4_TYPE_SIZE		1

#define NOKIA_RECV_ACL \
	H4_RECV_ACL, \
	.wordaligned = true

#define NOKIA_RECV_SCO \
	H4_RECV_SCO, \
	.wordaligned = true

#define NOKIA_RECV_EVENT \
	H4_RECV_EVENT, \
	.wordaligned = true

#define NOKIA_RECV_ALIVE \
	.type = HCI_NOKIA_ALIVE_PKT, \
	.hlen = HCI_NOKIA_ALIVE_HDR_SIZE, \
	.loff = 0, \
	.lsize = 1, \
	.maxlen = HCI_NOKIA_MAX_ALIVE_SIZE, \
	.wordaligned = true

#define NOKIA_RECV_NEG \
	.type = HCI_NOKIA_NEG_PKT, \
	.hlen = HCI_NOKIA_NEG_HDR_SIZE, \
	.loff = 0, \
	.lsize = 1, \
	.maxlen = HCI_NOKIA_MAX_NEG_SIZE, \
	.wordaligned = true

#define NOKIA_RECV_RADIO \
	.type = HCI_NOKIA_RADIO_PKT, \
	.hlen = HCI_NOKIA_RADIO_HDR_SIZE, \
	.loff = 1, \
	.lsize = 1, \
	.maxlen = HCI_NOKIA_MAX_RADIO_SIZE, \
	.wordaligned = true

struct hci_nokia_neg_hdr {
	__u8	dlen;
} __packed;

struct hci_nokia_neg_cmd {
	__u8	ack;
	__u16	baud;
	__u16	unused1;
	__u8	proto;
	__u16	sys_clk;
	__u16	unused2;
} __packed;

static inline struct hci_nokia_neg_hdr *hci_nokia_neg_hdr(const struct sk_buff *skb)
{
	return (struct hci_nokia_neg_hdr *) skb->data;
}

#define NOKIA_ALIVE_REQ   0x55
#define NOKIA_ALIVE_RESP  0xcc

struct hci_nokia_alive_hdr {
	__u8	dlen;
} __packed;

struct hci_nokia_alive_pkt {
	__u8	mid;
	__u8	unused;
} __packed;

static inline struct hci_nokia_alive_hdr *hci_nokia_alive_hdr(const struct sk_buff *skb)
{
	return (struct hci_nokia_alive_hdr *) skb->data;
}

struct hci_nokia_neg_evt {
	__u8	ack;
	__u16	baud;
	__u16	unused1;
	__u8	proto;
	__u16	sys_clk;
	__u16	unused2;
	__u8	man_id;
	__u8	ver_id;
} __packed;

#define BT_BAUDRATE_DIVIDER     384000000
#define BC4_MAX_BAUD_RATE       3692300
#define MAX_BAUD_RATE           921600
#define INIT_SPEED              120000

struct hci_nokia_radio_hdr {
	__u8	evt;
	__u8	dlen;
} __packed;


struct nokia_uart_dev {
	struct device *dev;
	struct tty_port *port;
	struct gpio_desc *reset;
	struct gpio_desc *wakeup_host;
	struct gpio_desc *wakeup_bt;
	unsigned long sysclk_speed;
};

struct nokia_bt_dev {
	struct hci_uart *hu;
	struct nokia_uart_dev *btdata;
	int wake_irq;
	bool wake_state;
	struct sk_buff *rx_skb;
	struct sk_buff_head txq;
	bdaddr_t bdaddr;

	int init_error;
	struct completion init_completion;

	uint8_t man_id;
	uint8_t ver_id;
};

static char *nokia_get_fw_name(struct nokia_bt_dev *btdev)
{
	switch (btdev->man_id) {
	case NOKIA_ID_CSR:
		return FIRMWARE_CSR;
	case NOKIA_ID_BCM2048:
		return FIRMWARE_BCM2048;
	case NOKIA_ID_TI1271:
		return FIRMWARE_TI1271;
	default:
		return NULL;
	}
}

static int hci_uart_wait_for_cts(struct hci_uart *hu, bool state,
				 int timeout_ms)
{
	unsigned long timeout;
	int signal;

	timeout = jiffies + msecs_to_jiffies(timeout_ms);
	for (;;) {
		signal = hu->tty->ops->tiocmget(hu->tty) & TIOCM_CTS;
		if (!!signal == !!state) {
			dev_dbg(hu->tty->dev, "wait for cts... received!\n");
			return 0;
		}
		if (time_after(jiffies, timeout)) {
			dev_dbg(hu->tty->dev, "wait for cts... timeout!\n");
			return -ETIMEDOUT;
		}
		usleep_range(1000, 2000);
	}
}

static int btdev_match(struct device *child, void *data)
{
	if (!strcmp(child->driver->name, "nokia-bluetooth"))
		return 1;
	else
		return 0;
}

static irqreturn_t wakeup_handler(int irq, void *data)
{
	struct nokia_bt_dev *btdev = data;
	struct device *serialdev = btdev->hu->tty->dev;
	int wake_state = gpiod_get_value(btdev->btdata->wakeup_host);

	dev_dbg(serialdev, "wakeup received: %d -> %d\n",
		btdev->wake_state, wake_state);

	if (btdev->wake_state == wake_state)
		return IRQ_HANDLED;

	if (wake_state)
		pm_runtime_get_sync(serialdev);
	else if (!wake_state)
		pm_runtime_put(serialdev);

	btdev->wake_state = wake_state;

	return IRQ_HANDLED;
}

static int nokia_reset(struct hci_uart *hu)
{
	struct nokia_bt_dev *btdev = hu->priv;
	int err;

	/* reset routine */
	gpiod_set_value_cansleep(btdev->btdata->reset, 0);
	gpiod_set_value_cansleep(btdev->btdata->wakeup_bt, 1);

	msleep(50);

	/* safety check */
	err = gpiod_get_value_cansleep(btdev->btdata->wakeup_host);
	if (err == 1) {
		dev_err(hu->tty->dev, "reset: host wakeup not low!\n");
		return -EPROTO;
	}

	/* flush queues */
	tty_ldisc_flush(hu->tty);
	tty_driver_flush_buffer(hu->tty);

	/* init uart */
	hci_uart_init_tty(hu);
	hci_uart_set_flow_control(hu, true);
	hci_uart_set_baudrate(hu, INIT_SPEED);

	gpiod_set_value_cansleep(btdev->btdata->reset, 1);
	gpiod_set_value_cansleep(btdev->btdata->wakeup_bt, 0);

	msleep(100);

	err = gpiod_get_value_cansleep(btdev->btdata->wakeup_host);
	if (err == 0) {
		dev_err(hu->tty->dev, "reset: host wakeup not high!\n");
		return -EPROTO;
	}

	/* wait for cts */
	err = hci_uart_wait_for_cts(hu, true, 100);
	if (err < 0) {
		dev_err(hu->tty->dev, "CTS not received: %d\n", err);
		return err;
	}

	gpiod_set_value_cansleep(btdev->btdata->wakeup_bt, 1);
	hci_uart_set_flow_control(hu, false);

	return 0;
}

static int nokia_send_alive_packet(struct hci_uart *hu)
{
	struct nokia_bt_dev *btdev = hu->priv;
	struct hci_nokia_alive_hdr *hdr;
	struct hci_nokia_alive_pkt *pkt;
	struct sk_buff *skb;
	int len;

	dev_dbg(hu->tty->dev, "Sending alive packet...\n");

	init_completion(&btdev->init_completion);

	len = H4_TYPE_SIZE + sizeof(*hdr) + sizeof(*pkt);
	skb = bt_skb_alloc(len, GFP_KERNEL);
	if (!skb)
		return -ENOMEM;

	hci_skb_pkt_type(skb) = HCI_NOKIA_ALIVE_PKT;
	memset(skb->data, 0x00, len);

	hdr = (struct hci_nokia_alive_hdr *)skb_put(skb, sizeof(*hdr));
	hdr->dlen = sizeof(*pkt);
	pkt = (struct hci_nokia_alive_pkt *)skb_put(skb, sizeof(*pkt));
	pkt->mid = NOKIA_ALIVE_REQ;

	hu->hdev->send(hu->hdev, skb);

	if (!wait_for_completion_interruptible_timeout(&btdev->init_completion,
		msecs_to_jiffies(1000))) {
		return -ETIMEDOUT;
	}

	if (btdev->init_error < 0)
		return btdev->init_error;

	return 0;
}

static int nokia_send_negotiation(struct hci_uart *hu)
{
	struct nokia_bt_dev *btdev = hu->priv;
	struct hci_nokia_neg_cmd *neg_cmd;
	struct hci_nokia_neg_hdr *neg_hdr;
	struct sk_buff *skb;
	int len, err;
	u16 baud = DIV_ROUND_CLOSEST(BT_BAUDRATE_DIVIDER, MAX_BAUD_RATE);
	int sysclk = btdev->btdata->sysclk_speed / 1000;

	dev_dbg(hu->tty->dev, "Sending negotiation...\n");

	len = H4_TYPE_SIZE + sizeof(*neg_hdr) + sizeof(*neg_cmd);
	skb = bt_skb_alloc(len, GFP_KERNEL);
	if (!skb)
		return -ENOMEM;

	hci_skb_pkt_type(skb) = HCI_NOKIA_NEG_PKT;

	neg_hdr = (struct hci_nokia_neg_hdr *)skb_put(skb, sizeof(*neg_hdr));
	neg_hdr->dlen = sizeof(*neg_cmd);

	neg_cmd = (struct hci_nokia_neg_cmd *)skb_put(skb, sizeof(*neg_cmd));
	neg_cmd->ack = NOKIA_NEG_REQ;
	neg_cmd->baud = cpu_to_le16(baud);
	neg_cmd->unused1 = 0x0000;
	neg_cmd->proto = NOKIA_PROTO_BYTE;
	neg_cmd->sys_clk = cpu_to_le16(sysclk);
	neg_cmd->unused2 = 0x0000;

	btdev->init_error = 0;
	init_completion(&btdev->init_completion);

	hu->hdev->send(hu->hdev, skb);

	if (!wait_for_completion_interruptible_timeout(&btdev->init_completion,
		msecs_to_jiffies(10000))) {
		return -ETIMEDOUT;
	}

	if (btdev->init_error < 0)
		return btdev->init_error;

	/* Change to operational settings */
	hci_uart_set_flow_control(hu, true); // disable flow control

	/* setup negotiated max. baudrate */
	hci_uart_set_baudrate(hu, MAX_BAUD_RATE);

	err = hci_uart_wait_for_cts(hu, true, 100);
	if (err < 0)
		return err;

	hci_uart_set_flow_control(hu, false); // re-enable flow control

	dev_dbg(hu->tty->dev, "Negotiation successful...\n");

	return 0;
}

static int nokia_setup_fw(struct hci_uart *hu)
{
	struct nokia_bt_dev *btdev = hu->priv;
	const struct firmware *fw;
	const u8 *fw_ptr;
	size_t fw_size;
	int err;

	BT_DBG("hu %p", hu);

	err = request_firmware(&fw, nokia_get_fw_name(btdev), hu->tty->dev);
	if (err < 0) {
		BT_ERR("%s: Failed to load Nokia firmware file (%d)",
		       hu->hdev->name, err);
		return err;
	}

	fw_ptr = fw->data;
	fw_size = fw->size;

	while (fw_size >= 4) {
		u16 pkt_size = get_unaligned_le16(fw_ptr);
		u8 pkt_type = fw_ptr[2];
		const struct hci_command_hdr *cmd;
		u16 opcode;
		struct sk_buff *skb;

		switch (pkt_type) {
		case HCI_COMMAND_PKT:
			cmd = (struct hci_command_hdr *)(fw_ptr + 3);
			opcode = le16_to_cpu(cmd->opcode);

			skb = __hci_cmd_sync(hu->hdev, opcode, cmd->plen,
					     fw_ptr + 3 + HCI_COMMAND_HDR_SIZE,
					     HCI_INIT_TIMEOUT);
			if (IS_ERR(skb)) {
				err = PTR_ERR(skb);
				BT_ERR("%s: Firmware command %04x failed (%d)",
				       hu->hdev->name, opcode, err);
				goto done;
			}
			kfree_skb(skb);
			break;
		case HCI_NOKIA_RADIO_PKT:
		case HCI_NOKIA_NEG_PKT:
		case HCI_NOKIA_ALIVE_PKT:
			break;
		}

		fw_ptr += pkt_size + 2;
		fw_size -= pkt_size + 2;
	}

done:
	release_firmware(fw);
	return err;
}

static int nokia_setup(struct hci_uart *hu)
{
	int err;

	pm_runtime_get_sync(hu->tty->dev);

	dev_dbg(hu->tty->dev, "Nokia H4+ protocol setup...\n");

	/* 0. reset connection */
	err = nokia_reset(hu);
	if (err < 0) {
		dev_err(hu->tty->dev, "Reset failed: %d\n", err);
		goto out;
	}

	/* 1. negotiate speed etc */
	err = nokia_send_negotiation(hu);
	if (err < 0) {
		dev_err(hu->tty->dev, "Negotiation failed: %d\n", err);
		goto out;
	}

	/* 2. verify correct setup using alive packet */
	err = nokia_send_alive_packet(hu);
	if (err < 0) {
		dev_err(hu->tty->dev, "Alive check failed: %d\n", err);
		goto out;
	}

	/* 3. send firmware */
	err = nokia_setup_fw(hu);
	if (err < 0) {
		dev_err(hu->tty->dev, "Could not setup FW: %d\n", err);
		goto out;
	}

	hci_uart_set_flow_control(hu, true);
	hci_uart_set_baudrate(hu, BC4_MAX_BAUD_RATE);
	hci_uart_set_flow_control(hu, false);

	dev_dbg(hu->tty->dev, "Nokia H4+ protocol setup done!\n");

	/*
	 * TODO:
	 * disable wakeup_bt at this point and automatically enable it when
	 * data is about to be written until all data has been written (+ some
	 * delay).
	 *
	 * Since this is not yet support by the uart/tty kernel framework we
	 * will always keep enabled the wakeup_bt gpio for now, so that the
	 * bluetooth chip will never transit into idle modes.
	 */

out:
	pm_runtime_put(hu->tty->dev);

	return err;
}

static int nokia_open(struct hci_uart *hu)
{
	struct device *serialdev = hu->tty->dev;
	struct nokia_bt_dev *btdev;
	struct device *uartbtdev;
	int err;

	btdev = kzalloc(sizeof(*btdev), GFP_KERNEL);
	if (!btdev)
		return -ENOMEM;

	btdev->hu = hu;

	skb_queue_head_init(&btdev->txq);

	uartbtdev = device_find_child(serialdev, NULL, btdev_match);
	if (!uartbtdev) {
		dev_err(serialdev, "bluetooth device node not found!\n");
		return -ENODEV;
	}

	btdev->btdata = dev_get_drvdata(uartbtdev);
	if (!btdev->btdata)
		return -EINVAL;

	hu->priv = btdev;

	/* register handler for host wakeup gpio */
	btdev->wake_irq = gpiod_to_irq(btdev->btdata->wakeup_host);
	err = request_threaded_irq(btdev->wake_irq, NULL, wakeup_handler,
		IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
		"wakeup", btdev);
	if (err) {
		gpiod_set_value(btdev->btdata->reset, 0);
		gpiod_set_value(btdev->btdata->wakeup_bt, 0);
		return err;
	}

	dev_dbg(serialdev, "Nokia H4+ protocol initialized with %s!\n",
		dev_name(uartbtdev));

	pm_runtime_enable(hu->tty->dev);

	return 0;
}

static int nokia_flush(struct hci_uart *hu)
{
	struct nokia_bt_dev *btdev = hu->priv;

	BT_DBG("hu %p", hu);

	skb_queue_purge(&btdev->txq);

	return 0;
}

static int nokia_close(struct hci_uart *hu)
{
	struct nokia_bt_dev *btdev = hu->priv;

	hu->priv = NULL;

	BT_DBG("hu %p", hu);

	skb_queue_purge(&btdev->txq);

	kfree_skb(btdev->rx_skb);

	free_irq(btdev->wake_irq, btdev);

	/* disable module */
	gpiod_set_value(btdev->btdata->reset, 0);
	gpiod_set_value(btdev->btdata->wakeup_bt, 0);

	hu->priv = NULL;
	kfree(btdev);

	pm_runtime_disable(hu->tty->dev);

	return 0;
}

/* Enqueue frame for transmittion (padding, crc, etc) */
static int nokia_enqueue(struct hci_uart *hu, struct sk_buff *skb)
{
	struct nokia_bt_dev *btdev = hu->priv;
	int err;

	BT_DBG("hu %p skb %p", hu, skb);

	/* Prepend skb with frame type */
	memcpy(skb_push(skb, 1), &bt_cb(skb)->pkt_type, 1);

	/* Packets must be word aligned */
	if (skb->len % 2) {
		err = skb_pad(skb, 1);
		if (err)
			return err;
		*skb_put(skb, 1) = 0x00;
	}

	skb_queue_tail(&btdev->txq, skb);

	return 0;
}

static int nokia_recv_negotiation_packet(struct hci_dev *hdev,
					 struct sk_buff *skb)
{
	struct hci_uart *hu = hci_get_drvdata(hdev);
	struct nokia_bt_dev *btdev = hu->priv;
	struct hci_nokia_neg_hdr *hdr;
	struct hci_nokia_neg_evt *evt;
	int ret = 0;

	hdr = (struct hci_nokia_neg_hdr *)skb->data;
	if (hdr->dlen != sizeof(*evt)) {
		btdev->init_error = -EIO;
		ret = -EIO;
		goto finish_neg;
	}

	evt = (struct hci_nokia_neg_evt *)skb_pull(skb, sizeof(*hdr));

	if (evt->ack != NOKIA_NEG_ACK) {
		dev_err(hu->tty->dev, "Could not negotiate hci_nokia settings\n");
		btdev->init_error = -EINVAL;
	}

	btdev->man_id = evt->man_id;
	btdev->ver_id = evt->ver_id;

	dev_dbg(hu->tty->dev, "NOKIA negotiation:\n");
	dev_dbg(hu->tty->dev, "\tbaudrate = %u\n", evt->baud);
	dev_dbg(hu->tty->dev, "\tsystem clock = %u\n", evt->sys_clk);
	dev_dbg(hu->tty->dev, "\tmanufacturer id = %u\n", evt->man_id);
	dev_dbg(hu->tty->dev, "\tversion id = %u\n", evt->ver_id);

finish_neg:
	complete(&btdev->init_completion);
	kfree_skb(skb);
	return ret;
}

static int nokia_recv_alive_packet(struct hci_dev *hdev, struct sk_buff *skb)
{
	struct hci_uart *hu = hci_get_drvdata(hdev);
	struct nokia_bt_dev *btdev = hu->priv;
	struct hci_nokia_alive_hdr *hdr;
	struct hci_nokia_alive_pkt *pkt;
	int ret = 0;

	hdr = (struct hci_nokia_alive_hdr *)skb->data;
	if (hdr->dlen != sizeof(*pkt)) {
		dev_err(hu->tty->dev, "Corrupted alive message\n");
		btdev->init_error = -EIO;
		ret = -EIO;
		goto finish_alive;
	}

	pkt = (struct hci_nokia_alive_pkt *)skb_pull(skb, sizeof(*hdr));

	if (pkt->mid != NOKIA_ALIVE_RESP) {
		dev_err(hu->tty->dev, "Invalid alive response: 0x%02x!\n",
			pkt->mid);
		btdev->init_error = -EINVAL;
		goto finish_alive;
	}

	dev_dbg(hu->tty->dev, "Received alive packet!\n");

finish_alive:
	complete(&btdev->init_completion);
	kfree_skb(skb);
	return ret;
}

static int nokia_recv_radio(struct hci_dev *hdev, struct sk_buff *skb)
{
	/* Packets received on the dedicated radio channel are
	 * HCI events and so feed them back into the core.
	 */
	bt_cb(skb)->pkt_type = HCI_EVENT_PKT;
	return hci_recv_frame(hdev, skb);
}

/* Recv data */
static const struct h4_recv_pkt nokia_recv_pkts[] = {
	{ NOKIA_RECV_ACL,	.recv = hci_recv_frame },
	{ NOKIA_RECV_SCO,	.recv = hci_recv_frame },
	{ NOKIA_RECV_EVENT,	.recv = hci_recv_frame },
	{ NOKIA_RECV_ALIVE,	.recv = nokia_recv_alive_packet },
	{ NOKIA_RECV_NEG,	.recv = nokia_recv_negotiation_packet },
	{ NOKIA_RECV_RADIO,	.recv = nokia_recv_radio },
};

static int nokia_recv(struct hci_uart *hu, const void *data, int count)
{
	struct nokia_bt_dev *btdev = hu->priv;
	int err;

	if (!test_bit(HCI_UART_REGISTERED, &hu->flags))
		return -EUNATCH;

	btdev->rx_skb = h4_recv_buf(hu->hdev, btdev->rx_skb, data, count,
				  nokia_recv_pkts, ARRAY_SIZE(nokia_recv_pkts));
	if (IS_ERR(btdev->rx_skb)) {
		err = PTR_ERR(btdev->rx_skb);
		BT_ERR("%s: Frame reassembly failed (%d)", hu->hdev->name, err);
		btdev->rx_skb = NULL;
		return err;
	}

	return count;
}

static struct sk_buff *nokia_dequeue(struct hci_uart *hu)
{
	struct nokia_bt_dev *btdev = hu->priv;

	return skb_dequeue(&btdev->txq);
}

static const struct hci_uart_proto nokia_proto = {
	.id		= HCI_UART_NOKIA,
	.name		= "Nokia",
	.open		= nokia_open,
	.close		= nokia_close,
	.recv		= nokia_recv,
	.enqueue	= nokia_enqueue,
	.dequeue	= nokia_dequeue,
	.flush		= nokia_flush,
	.setup		= nokia_setup,
};

static int nokia_bluetooth_probe(struct platform_device *pdev)
{
	struct nokia_uart_dev *btdata;
	struct device *bcmdev = &pdev->dev;
	struct clk *sysclk;
	int err = 0;

	if(!bcmdev->parent) {
		dev_err(bcmdev, "parent device missing!\n");
		return -ENODEV;
	}

	btdata = devm_kmalloc(bcmdev, sizeof(*btdata), GFP_KERNEL);
	if(!btdata)
		return -ENOMEM;

	btdata->dev = bcmdev;
	dev_set_drvdata(bcmdev, btdata);

	btdata->port = dev_get_drvdata(bcmdev->parent);
	if(!btdata->port) {
		dev_err(bcmdev, "port data missing in parent device!\n");
		return -ENODEV;
	}

	btdata->reset = devm_gpiod_get(bcmdev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(btdata->reset)) {
		err = PTR_ERR(btdata->reset);
		dev_err(bcmdev, "could not get reset gpio: %d\n", err);
		return err;
	}

	btdata->wakeup_host = devm_gpiod_get(bcmdev, "host-wakeup", GPIOD_IN);
	if (IS_ERR(btdata->wakeup_host)) {
		err = PTR_ERR(btdata->wakeup_host);
		dev_err(bcmdev, "could not get host wakeup gpio: %d\n", err);
		return err;
	}


	btdata->wakeup_bt = devm_gpiod_get(bcmdev, "bluetooth-wakeup",
					    GPIOD_OUT_LOW);
	if (IS_ERR(btdata->wakeup_bt)) {
		err = PTR_ERR(btdata->wakeup_bt);
		dev_err(bcmdev, "could not get BT wakeup gpio: %d\n", err);
		return err;
	}

	sysclk = devm_clk_get(bcmdev, "sysclk");
	if (IS_ERR(sysclk)) {
		err = PTR_ERR(sysclk);
		dev_err(bcmdev, "could not get sysclk: %d\n", err);
		return err;
	}

	clk_prepare_enable(sysclk);
	btdata->sysclk_speed = clk_get_rate(sysclk);
	clk_disable_unprepare(sysclk);

	dev_dbg(bcmdev, "parent uart: %s\n", dev_name(bcmdev->parent));
	dev_dbg(bcmdev, "sysclk speed: %ld kHz\n", btdata->sysclk_speed / 1000);

	/* TODO: open tty and setup line disector from kernel-side */

	return err;
}

static const struct of_device_id nokia_bluetooth_of_match[] = {
	{ .compatible = "nokia,brcm,bcm2048", },
	{ .compatible = "nokia,ti,wl1271-bluetooth", },
	{},
};
MODULE_DEVICE_TABLE(of, nokia_bluetooth_of_match);

static struct platform_driver platform_nokia_driver = {
	.driver = {
		.name = "nokia-bluetooth",
		.of_match_table = nokia_bluetooth_of_match,
	},
	.probe = nokia_bluetooth_probe,
};

int __init nokia_init(void)
{
	platform_driver_register(&platform_nokia_driver);
	return hci_uart_register_proto(&nokia_proto);
}

int __exit nokia_deinit(void)
{
	platform_driver_unregister(&platform_nokia_driver);
	return hci_uart_unregister_proto(&nokia_proto);
}
