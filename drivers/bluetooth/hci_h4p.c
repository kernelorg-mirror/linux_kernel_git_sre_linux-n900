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
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#define DEBUG

#include <linux/module.h>

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

#include <linux/gpio/consumer.h>

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>

#include "hci_uart.h"
#include "hci_h4p.h"

struct h4p_struct {
	struct hci_uart *hu;
	struct h4p_dev_struct *btdata;
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

static char* h4p_get_fw_name(struct h4p_struct *h4p) {
	switch(h4p->man_id) {
		case H4P_ID_CSR:
			return FIRMWARE_CSR;
		case H4P_ID_BCM2048:
			return FIRMWARE_BCM2048;
		case H4P_ID_TI1271:
			return FIRMWARE_TI1271;
		default:
			return NULL;
	}
}

static int h4p_wait_for_cts(struct hci_uart *hu, bool state, int timeout_ms)
{
	unsigned long timeout;
	int signal;

	timeout = jiffies + msecs_to_jiffies(timeout_ms);
	for (;;) {
		signal = hu->tty->ops->tiocmget(hu->tty) & TIOCM_CTS;
		if(!!signal == !!state) {
			dev_dbg(hu->tty->dev, "wait for cts... received!\n");
			return 0;
		}
		if (time_after(jiffies, timeout)) {
			dev_dbg(hu->tty->dev, "wait for cts... timeout!\n");
			return -ETIMEDOUT;
		}
		msleep(1);
	}
}

static void h4p_set_speed(struct hci_uart *hu, unsigned long speed)
{
	dev_dbg(hu->tty->dev, "setting speed to %lu baud\n", speed);
	hci_uart_set_baudrate(hu, speed);
}

static int btdev_match(struct device *child, void *data)
{
	/* TODO: do not simply return the first child */
	return 1;
}

static irqreturn_t wakeup_handler(int irq, void *data)
{
	struct h4p_struct *h4p = data;
	struct device *serialdev = h4p->hu->tty->dev;
	int wake_state = gpiod_get_value(h4p->btdata->wakeup_host);

	dev_dbg(serialdev, "wakeup received: %d -> %d\n",
		h4p->wake_state, wake_state);

	if (h4p->wake_state == wake_state)
		return IRQ_HANDLED;

	if(wake_state)
		pm_runtime_get_sync(serialdev);
	else if(!wake_state)
		pm_runtime_put(serialdev);

	h4p->wake_state = wake_state;

	return IRQ_HANDLED;
}

static int h4p_reset(struct hci_uart *hu)
{
	struct h4p_struct *h4p = hu->priv;
	int err;

	/* reset routine */
	gpiod_set_value_cansleep(h4p->btdata->reset, 0);
	gpiod_set_value_cansleep(h4p->btdata->wakeup_bt, 1);
	msleep(50);

	/* flush queues */
	tty_ldisc_flush(hu->tty);
	tty_driver_flush_buffer(hu->tty);

	/* init uart */
	hci_uart_init_tty(hu);
	h4p_set_speed(hu, INIT_SPEED);
	hci_uart_set_flow_control(hu, true);

	/* safety check */
	err = gpiod_get_value_cansleep(h4p->btdata->wakeup_host);
	if (err == 1) {
		dev_err(hu->tty->dev, "reset: host wakeup not low!\n");
		return -EPROTO;
	}

	gpiod_set_value_cansleep(h4p->btdata->reset, 1);
	gpiod_set_value_cansleep(h4p->btdata->wakeup_bt, 0);

	msleep(100);

	err = gpiod_get_value_cansleep(h4p->btdata->wakeup_host);
	if (err == 0) {
		dev_err(hu->tty->dev, "reset: host wakeup not high!\n");
		return -EPROTO;
	}

	/* wait for cts */
	err = h4p_wait_for_cts(hu, true, 100);
	if (err < 0) {
		dev_err(hu->tty->dev, "CTS not received: %d\n", err);
		return err;
	}

	gpiod_set_value_cansleep(h4p->btdata->wakeup_bt, 1);
	hci_uart_set_flow_control(hu, false);

	return 0;
}

static int h4p_send_alive_packet(struct hci_uart *hu)
{
	struct h4p_struct *h4p = hu->priv;
	struct hci_h4p_alive_hdr *hdr;
	struct hci_h4p_alive_pkt *pkt;
	struct sk_buff *skb;
	int len;

	dev_dbg(hu->tty->dev, "Sending alive packet...\n");

	init_completion(&h4p->init_completion);

	len = H4_TYPE_SIZE + sizeof(*hdr) + sizeof(*pkt);
	skb = bt_skb_alloc(len, GFP_KERNEL);
	if (!skb)
		return -ENOMEM;

	hci_skb_pkt_type(skb) = HCI_H4P_ALIVE_PKT;
	memset(skb->data, 0x00, len);

	hdr = (struct hci_h4p_alive_hdr *)skb_put(skb, sizeof(*hdr));
	hdr->dlen = sizeof(*pkt);
	pkt = (struct hci_h4p_alive_pkt *)skb_put(skb, sizeof(*pkt));
	pkt->mid = H4P_ALIVE_REQ;

	hu->hdev->send(hu->hdev, skb);

	if (!wait_for_completion_interruptible_timeout(&h4p->init_completion,
		msecs_to_jiffies(1000))) {
		return -ETIMEDOUT;
	}

	if (h4p->init_error < 0)
		return h4p->init_error;

	return 0;
}

static int h4p_send_radio_packet(struct hci_uart *hu,
				struct hci_h4p_radio_hdr * hdr, const u8 *data)
{
	struct h4p_struct *h4p = hu->priv;
	struct sk_buff *skb;
	int len;

	dev_dbg(hu->tty->dev, "Sending radio packet...\n");

	init_completion(&h4p->init_completion);

	len = H4_TYPE_SIZE + sizeof(*hdr) + hdr->dlen;
	skb = bt_skb_alloc(len, GFP_KERNEL);
	if (!skb)
		return -ENOMEM;

	hci_skb_pkt_type(skb) = HCI_H4P_RADIO_PKT;

	memset(skb->data, 0x00, len);
	*skb_put(skb, 1) = hdr->evt;
	*skb_put(skb, 1) = hdr->dlen;
	memcpy(skb->data + H4_TYPE_SIZE + sizeof(*hdr), data, hdr->dlen);

	hu->hdev->send(hu->hdev, skb);

	dev_dbg(hu->tty->dev, "Radio packet sent\n");

	/* TODO: FIXME: handler in h4p_reassembly() is missing */

	if (!wait_for_completion_interruptible_timeout(&h4p->init_completion,
		msecs_to_jiffies(1000))) {
		dev_err(hu->tty->dev, "radio packet timeout!\n");
		return -ETIMEDOUT;
	}

	if (h4p->init_error < 0)
		return h4p->init_error;

	return 0;
}

static int h4p_send_negotiation(struct hci_uart *hu)
{
	struct h4p_struct *h4p = hu->priv;
	struct hci_h4p_neg_cmd *neg_cmd;
	struct hci_h4p_neg_hdr *neg_hdr;
	struct sk_buff *skb;
	int len, err;
	u16 baud = DIV_ROUND_CLOSEST(BT_BAUDRATE_DIVIDER, MAX_BAUD_RATE);
	int sysclk = h4p->btdata->sysclk_speed / 1000;

	dev_dbg(hu->tty->dev, "Sending negotiation...\n");

	/*
	 * TODO: FIXME: This fails on N900 (bcm2048)
	 * (04 10 01 00) = hardware error event
	 *
	 * Sending Tests:
	 * (06 0a) => No reply
	 * (06 0a 00) => No reply
	 * (06 0a 00 a1) => (04 10 01 00)
	 * (06 0a 00 a1 01) => (04 10 01 00)
	 * (06 0a 00 a1 01 00 00 4c 00 96 00 00) => (04 10 01 00)
	 */

	len = H4_TYPE_SIZE + sizeof(*neg_hdr) + sizeof(*neg_cmd);
	skb = bt_skb_alloc(len, GFP_KERNEL);
	if (!skb)
		return -ENOMEM;

	hci_skb_pkt_type(skb) = HCI_H4P_NEG_PKT;

	neg_hdr = (struct hci_h4p_neg_hdr *)skb_put(skb, sizeof(*neg_hdr));
	neg_hdr->dlen = sizeof(*neg_cmd);

	neg_cmd = (struct hci_h4p_neg_cmd *)skb_put(skb, sizeof(*neg_cmd));
	neg_cmd->ack = H4P_NEG_REQ;
	neg_cmd->baud = cpu_to_le16(baud);
	neg_cmd->unused1 = 0x0000;
	neg_cmd->proto = H4P_PROTO_BYTE;
	neg_cmd->sys_clk = cpu_to_le16(sysclk);
	neg_cmd->unused2 = 0x0000;

	h4p->init_error = 0;
	init_completion(&h4p->init_completion);

	dev_dbg(hu->tty->dev, "gpio state: reset=%x wakehost=%x wakebt=%x\n", gpiod_get_value(h4p->btdata->reset), gpiod_get_value(h4p->btdata->wakeup_host), gpiod_get_value(h4p->btdata->wakeup_bt));

	hu->hdev->send(hu->hdev, skb);

	if (!wait_for_completion_interruptible_timeout(&h4p->init_completion,
		msecs_to_jiffies(10000))) {
		return -ETIMEDOUT;
	}

	if (h4p->init_error < 0)
		return h4p->init_error;

	/* Change to operational settings */
	hci_uart_set_flow_control(hu, true); // disable flow control

	/* setup negotiated max. baudrate */
	h4p_set_speed(hu, MAX_BAUD_RATE);

	err = h4p_wait_for_cts(hu, true, 100);
	if (err < 0)
		return err;

	hci_uart_set_flow_control(hu, false); // re-enable flow control

	dev_dbg(hu->tty->dev, "Negotiation successful...\n");

	return 0;
}

static int h4p_fw_command(struct hci_uart *hu, const struct firmware *fw,
			  int *fw_pos)
{
	struct h4p_struct *h4p = hu->priv;
	unsigned int cmd_len;
	u8 pkt_type;

	if ((*fw_pos) >= fw->size)
		return 0;

	if ((*fw_pos) + 2 > fw->size) {
		dev_err(hu->tty->dev, "Corrupted firmware image 1\n");
		return -EMSGSIZE;
	}

	cmd_len = fw->data[(*fw_pos)++];
	cmd_len += fw->data[(*fw_pos)++] << 8;
	if (cmd_len == 0)
		return 1;

	if ((*fw_pos) + cmd_len > fw->size) {
		dev_err(hu->tty->dev, "Corrupted firmware image 2\n");
		return -EMSGSIZE;
	}

	pkt_type = fw->data[(*fw_pos)];

	if (pkt_type == HCI_COMMAND_PKT) {
		struct hci_command_hdr *cmd;
		const u8 *cmd_param;
		struct sk_buff *evt;

		cmd = (struct hci_command_hdr *)(fw->data + *fw_pos +1);
		cmd_param = fw->data + *fw_pos + H4_TYPE_SIZE +
			    HCI_COMMAND_HDR_SIZE;

		if (le16_to_cpu(cmd->opcode) == HCI_H4P_BCM_BDADDR) {
			set_bit(HCI_QUIRK_INVALID_BDADDR, &hu->hdev->quirks);
			if(cmd_len >= 10)
				cmd_param = (u8*) &(h4p->bdaddr);
		}

		evt = __hci_cmd_sync(hu->hdev, le16_to_cpu(cmd->opcode),
				     cmd->plen, cmd_param, HCI_INIT_TIMEOUT);
		if (IS_ERR(evt)) {
			dev_err(hu->tty->dev, "Sending FW cmd failed: %ld\n",
				PTR_ERR(evt));
			return PTR_ERR(evt);
		}

		if (evt->data[0] != 0x00) {
			dev_err(hu->tty->dev, "Sending FW cmd failed: 0x%.2x\n",
				evt->data[0]);
			return -EPROTO;
		}
	} else if (pkt_type == HCI_H4P_RADIO_PKT) {
		struct hci_h4p_radio_hdr *hdr;
		const u8 *radio_data;

		hdr = (struct hci_h4p_radio_hdr *)(fw->data + *fw_pos +1);
		radio_data = fw->data + *fw_pos + H4_TYPE_SIZE +
			     HCI_H4P_RADIO_HDR_SIZE;
		h4p_send_radio_packet(hu, hdr, radio_data);
	} else if (pkt_type == HCI_H4P_NEG_PKT) {
		dev_dbg(hu->tty->dev, "FW: Skip negotion packet!\n");
	} else if (pkt_type == HCI_H4P_ALIVE_PKT) {
		dev_dbg(hu->tty->dev, "FW: Skip alive packet!\n");
	} else {
		dev_err(hu->tty->dev, "Corrupted firmware image 3\n");
		return -EFAULT;
	}

	(*fw_pos) += cmd_len;

	return 1;
}

static int h4p_setup_fw(struct hci_uart *hu)
{
	struct h4p_struct *h4p = hu->priv;
	const struct firmware *fw = NULL;
	int fw_pos, err;

	err = request_firmware(&fw, h4p_get_fw_name(h4p), hu->tty->dev);
	if (err < 0 || !fw)
		goto clean;

	fw_pos = 0;
	while ((err = h4p_fw_command(hu, fw, &fw_pos))) {
		if (err < 0)
			goto clean;
	}

clean:
	release_firmware(fw);
	return err;
}

static int h4p_setup(struct hci_uart *hu)
{
	int err;

	pm_runtime_get_sync(hu->tty->dev);

	dev_info(hu->tty->dev, "Nokia H4+ protocol setup...\n");

	/* 0. reset connection */
	err = h4p_reset(hu);
	if (err < 0) {
		dev_err(hu->tty->dev, "Reset failed: %d\n", err);
		goto out;
	}

	/* 1. negotiate speed etc */
	err = h4p_send_negotiation(hu);
	if (err < 0) {
		dev_err(hu->tty->dev, "Negotiation failed: %d\n", err);
		goto out;
	}

	/* 2. verify correct setup using alive packet */
	err = h4p_send_alive_packet(hu);
	if (err < 0) {
		dev_err(hu->tty->dev, "Alive check failed: %d\n", err);
		goto out;
	}

	/* 3. send firmware */
	err = h4p_setup_fw(hu);
	if (err < 0) {
		dev_err(hu->tty->dev, "Could not setup FW: %d\n", err);
		goto out;
	}

	dev_info(hu->tty->dev, "Firmware has been loaded successfully, switching device speed!");

	hci_uart_set_flow_control(hu, true);
	h4p_set_speed(hu, BC4_MAX_BAUD_RATE);
	hci_uart_set_flow_control(hu, false);

out:
	pm_runtime_put(hu->tty->dev);

	return err;
}

/* Initialize protocol */
static int h4p_open(struct hci_uart *hu)
{
	struct device *serialdev = hu->tty->dev;
	struct h4p_struct *h4p;
	struct device *btdev;
	int err;

	h4p = kzalloc(sizeof(*h4p), GFP_KERNEL);
	if (!h4p)
		return -ENOMEM;

	h4p->hu = hu;

	skb_queue_head_init(&h4p->txq);

	btdev = device_find_child(serialdev, NULL, btdev_match);
	if(!btdev) {
		dev_err(serialdev, "bluetooth device node not found!\n");
		return -ENODEV;
	}

	h4p->btdata = dev_get_drvdata(btdev);
	if (!h4p->btdata)
		return -EINVAL;

	hu->priv = h4p;

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

	/* register handler for host wakeup gpio */
	h4p->wake_irq = gpiod_to_irq(h4p->btdata->wakeup_host);
	err = request_threaded_irq(h4p->wake_irq, NULL, wakeup_handler, IRQF_TRIGGER_RISING |
			  IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "wakeup", h4p);
	if (err) {
		gpiod_set_value(h4p->btdata->reset, 0);
		gpiod_set_value(h4p->btdata->wakeup_bt, 0);
		return err;
	}

	dev_dbg(serialdev, "Nokia H4+ protocol initialized with %s!\n", dev_name(btdev));

	pm_runtime_enable(hu->tty->dev);

	return 0;
}

/* Flush protocol data */
static int h4p_flush(struct hci_uart *hu)
{
	struct h4p_struct *h4p = hu->priv;

	BT_DBG("flush: hu %p", hu);

	skb_queue_purge(&h4p->txq);

	return 0;
}

/* Close protocol */
static int h4p_close(struct hci_uart *hu)
{
	struct h4p_struct *h4p = hu->priv;

	hu->priv = NULL;

	BT_DBG("close: hu %p", hu);

	skb_queue_purge(&h4p->txq);

	kfree_skb(h4p->rx_skb);

	free_irq(h4p->wake_irq, h4p);

	/* disable module */
	gpiod_set_value(h4p->btdata->reset, 0);
	gpiod_set_value(h4p->btdata->wakeup_bt, 0);

	hu->priv = NULL;
	kfree(h4p);

	pm_runtime_disable(hu->tty->dev);

	return 0;
}

/* Enqueue frame for transmittion (padding, crc, etc) */
static int h4p_enqueue(struct hci_uart *hu, struct sk_buff *skb)
{
	struct h4p_struct *h4p = hu->priv;
	int err;

	BT_DBG("enqueue: hu %p skb %p", hu, skb);

	/* Prepend skb with frame type */
	memcpy(skb_push(skb, 1), &bt_cb(skb)->pkt_type, 1);

	/* Packets must be word aligned */
	if (skb->len % 2) {
		err = skb_pad(skb, 1);
		if (err)
			return err;
		*skb_put(skb, 1) = 0x00;
	}

#ifdef DEBUG_VERBOSE
	print_hex_dump_bytes("send data: ", DUMP_PREFIX_NONE, skb->data, skb->len);
#endif

	skb_queue_tail(&h4p->txq, skb);

	return 0;
}

int h4p_recv_negotiation_packet(struct hci_dev *hdev, struct sk_buff *skb)
{
	struct hci_uart *hu = hci_get_drvdata(hdev);
	struct h4p_struct *h4p = hu->priv;
	struct hci_h4p_neg_hdr *hdr;
	struct hci_h4p_neg_evt *evt;
	int ret = 0;

	hdr = (struct hci_h4p_neg_hdr *)skb->data;
	if (hdr->dlen != sizeof(*evt)) {
		h4p->init_error = -EIO;
		ret = -EIO;
		goto finish_neg;
	}

	evt = (struct hci_h4p_neg_evt *)skb_pull(skb, sizeof(*hdr));

	if (evt->ack != H4P_NEG_ACK) {
		dev_err(hu->tty->dev, "Could not negotiate hci_h4p settings\n");
		h4p->init_error = -EINVAL;
	}

	h4p->man_id = evt->man_id;
	h4p->ver_id = evt->ver_id;

	dev_dbg(hu->tty->dev, "H4P negotiation:\n");
	dev_dbg(hu->tty->dev, "\tbaudrate = %u\n", evt->baud);
	dev_dbg(hu->tty->dev, "\tsystem clock = %u\n", evt->sys_clk);
	dev_dbg(hu->tty->dev, "\tmanufacturer id = %u\n", evt->man_id);
	dev_dbg(hu->tty->dev, "\tversion id = %u\n", evt->ver_id);

finish_neg:
	complete(&h4p->init_completion);
	kfree_skb(skb);
	return ret;
}

int h4p_recv_alive_packet(struct hci_dev *hdev, struct sk_buff *skb)
{
	struct hci_uart *hu = hci_get_drvdata(hdev);
	struct h4p_struct *h4p = hu->priv;
	struct hci_h4p_alive_hdr *hdr;
	struct hci_h4p_alive_pkt *pkt;
	int ret = 0;

	hdr = (struct hci_h4p_alive_hdr *)skb->data;
	if (hdr->dlen != sizeof(*pkt)) {
		dev_err(hu->tty->dev, "Corrupted alive message\n");
		h4p->init_error = -EIO;
		ret = -EIO;
		goto finish_alive;
	}

	pkt = (struct hci_h4p_alive_pkt *)skb_pull(skb, sizeof(*hdr));

	if (pkt->mid != H4P_ALIVE_RESP) {
		dev_err(hu->tty->dev, "Invalid alive response: 0x%02x!\n", pkt->mid);
		h4p->init_error = -EINVAL;
		goto finish_alive;
	}

	dev_dbg(hu->tty->dev, "Received alive packet!\n");

finish_alive:
	complete(&h4p->init_completion);
	kfree_skb(skb);
	return ret;
}

int h4p_recv_frame(struct hci_dev *hdev, struct sk_buff *skb)
{
	return hci_recv_frame(hdev, skb);
}

/* Recv data */
static const struct h4_recv_pkt h4p_recv_pkts[] = {
	{ H4P_RECV_ACL,		.recv = hci_recv_frame },
	{ H4P_RECV_SCO,		.recv = hci_recv_frame },
	{ H4P_RECV_EVENT,	.recv = hci_recv_frame },
	{ H4P_RECV_ALIVE,	.recv = h4p_recv_alive_packet },
	{ H4P_RECV_NEG,		.recv = h4p_recv_negotiation_packet },
};

static int h4p_recv(struct hci_uart *hu, const void *data, int count)
{
	struct h4p_struct *h4p = hu->priv;

#ifdef DEBUG_VERBOSE
	print_hex_dump_bytes("recv data: ", DUMP_PREFIX_NONE, data, count);
#endif

	if (!test_bit(HCI_UART_REGISTERED, &hu->flags))
		return -EUNATCH;

	h4p->rx_skb = h4_recv_buf(hu->hdev, h4p->rx_skb, data, count,
				  h4p_recv_pkts, ARRAY_SIZE(h4p_recv_pkts));
	if (IS_ERR(h4p->rx_skb)) {
		int err = PTR_ERR(h4p->rx_skb);
		BT_ERR("%s: Frame reassembly failed (%d)", hu->hdev->name, err);
		h4p->rx_skb = NULL;
		return err;
	}

	return count;
}

static struct sk_buff *h4p_dequeue(struct hci_uart *hu)
{
	struct h4p_struct *h4p = hu->priv;
	return skb_dequeue(&h4p->txq);
}

static const struct hci_uart_proto h4pp = {
	.id		= HCI_UART_H4P,
	.name		= "H4+",
	.open		= h4p_open,
	.close		= h4p_close,
	.recv		= h4p_recv,
	.enqueue	= h4p_enqueue,
	.dequeue	= h4p_dequeue,
	.flush		= h4p_flush,
	.setup		= h4p_setup,
	// TODO: FIXME: set_bdaddr
};

int __init h4p_init(void)
{
	return hci_uart_register_proto(&h4pp);
}

int __exit h4p_deinit(void)
{
	return hci_uart_unregister_proto(&h4pp);
}
