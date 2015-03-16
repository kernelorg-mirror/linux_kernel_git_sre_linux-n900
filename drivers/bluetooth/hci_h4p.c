/*
 *
 *  Bluetooth HCI UART H4 driver with Nokia Extensions
 *
 *  Copyright (C) 2000-2001  Qualcomm Incorporated
 *  Copyright (C) 2002-2003  Maxim Krasnyansky <maxk@qualcomm.com>
 *  Copyright (C) 2004-2005  Marcel Holtmann <marcel@holtmann.org>
 *  Copyright (C) 2015  Sebastian Reichel <sre@kernel.org>
 *
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

#include <linux/gpio/consumer.h>

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>

#include "hci_uart.h"
#include "hci_h4p.h"

#define STREAM_REASSEMBLY 0

struct h4p_struct {
	struct hci_uart *hu;
	struct h4p_dev_struct *btdata;
	int wake_irq;
	bool wake_state;
	struct sk_buff *rx_skb;
	struct sk_buff_head txq;
	bdaddr_t bdaddr;

	bool negotiated;

	int init_error;
	struct completion init_completion;
};

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

static int h4p_set_rts(struct hci_uart *hu, bool state)
{
	dev_dbg(hu->tty->dev, "setting rts: %u\n", state);

	if(state)
		return hu->tty->ops->tiocmset(hu->tty, TIOCM_RTS, 0);
	else
		return hu->tty->ops->tiocmset(hu->tty, 0, TIOCM_RTS);
}

static void h4p_set_speed(struct hci_uart *hu, unsigned long speed)
{
	struct ktermios old_termios;

	dev_dbg(hu->tty->dev, "setting speed to %lu baud\n", speed);

	down_write(&hu->tty->termios_rwsem);
	old_termios = hu->tty->termios;
	/* setup baud rate */
	tty_encode_baud_rate(hu->tty, speed, speed);
	/* 8 bit data */
	hu->tty->termios.c_cflag |= CS8;
	/* enable receiver */
	hu->tty->termios.c_cflag |= CREAD;
	/* ignore CD signal */
	hu->tty->termios.c_cflag |= CLOCAL;
	/* use one stop bit */
	hu->tty->termios.c_cflag &= ~CSTOPB;
	/* disable parity */
	hu->tty->termios.c_cflag &= ~PARENB;
	/* enable cts and rts */
	hu->tty->termios.c_cflag |= CRTSCTS;
	hu->tty->ops->set_termios(hu->tty, &old_termios);
	up_write(&hu->tty->termios_rwsem);
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

	dev_dbg(hu->tty->dev, "reset BT device...\n");

	h4p_set_speed(hu, INIT_SPEED);

	/* flush queues */
	tty_ldisc_flush(hu->tty);
	tty_driver_flush_buffer(hu->tty);

	/* reset routine */
	gpiod_set_value(h4p->btdata->reset, 0);
	gpiod_set_value(h4p->btdata->wakeup_bt, 1);
	msleep(10);

	/* safety check */
	err = gpiod_get_value(h4p->btdata->wakeup_host);
	if (err == 1) {
		dev_err(hu->tty->dev, "reset: host wakeup not low!\n");
		return -EPROTO;
	}

	gpiod_set_value(h4p->btdata->reset, 1);
	gpiod_set_value(h4p->btdata->wakeup_bt, 0);

	msleep(100);

	err = gpiod_get_value(h4p->btdata->wakeup_host);
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

	gpiod_set_value(h4p->btdata->wakeup_bt, 1);
	h4p_set_rts(hu, true);

	return 0;
}

static int h4p_send_alive_packet(struct hci_uart *hu)
{
	struct h4p_struct *h4p = hu->priv;
	struct hci_h4p_alive_hdr *hdr;
	struct hci_h4p_alive_pkt *pkt;
	struct sk_buff *skb;
	u8 type = HCI_H4P_ALIVE_PKT;
	int len;

	dev_dbg(hu->tty->dev, "Sending alive packet...\n");

	init_completion(&h4p->init_completion);

	len = H4_TYPE_SIZE + sizeof(*hdr) + sizeof(*pkt);
	skb = bt_skb_alloc(len, GFP_KERNEL);
	if (!skb)
		return -ENOMEM;

	memset(skb->data, 0x00, len);
	memcpy(skb_push(skb, 1), &type, 1);

	hdr = (struct hci_h4p_alive_hdr *)skb_put(skb, sizeof(*hdr));
	hdr->dlen = sizeof(*pkt);
	pkt = (struct hci_h4p_alive_pkt *)skb_put(skb, sizeof(*pkt));
	pkt->mid = H4P_ALIVE_REQ;

	skb_queue_tail(&h4p->txq, skb);
	hci_uart_tx_wakeup(hu);

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

	memset(skb->data, 0x00, len);
	*skb_put(skb, 1) = HCI_H4P_RADIO_PKT;
	*skb_put(skb, 1) = hdr->evt;
	*skb_put(skb, 1) = hdr->dlen;
	memcpy(skb->data + H4_TYPE_SIZE + sizeof(*hdr), data, hdr->dlen);

	skb_queue_tail(&h4p->txq, skb);
	hci_uart_tx_wakeup(hu);

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
	u8 type = HCI_H4P_NEG_PKT;
	int len, err;
	u16 baud = DIV_ROUND_CLOSEST(BT_BAUDRATE_DIVIDER, MAX_BAUD_RATE);
	int sysclk = h4p->btdata->sysclk_speed / 1000;

	dev_dbg(hu->tty->dev, "Sending negotiation...\n");

	len = H4_TYPE_SIZE + sizeof(*neg_hdr) + sizeof(*neg_cmd);
	skb = bt_skb_alloc(len, GFP_KERNEL);
	if (!skb)
		return -ENOMEM;

	memcpy(skb_push(skb, 1), &type, 1);
	neg_hdr = (struct hci_h4p_neg_hdr *)skb_put(skb, sizeof(*neg_hdr));
	neg_cmd = (struct hci_h4p_neg_cmd *)skb_put(skb, sizeof(*neg_cmd));

	neg_hdr->dlen = sizeof(*neg_cmd);
	neg_cmd->ack = H4P_NEG_REQ;
	neg_cmd->baud = cpu_to_le16(baud);
	neg_cmd->unused1 = 0x0000;
	neg_cmd->proto = H4P_PROTO_BYTE;
	neg_cmd->sys_clk = cpu_to_le16(sysclk);
	neg_cmd->unused2 = 0x0000;

	h4p->init_error = 0;
	init_completion(&h4p->init_completion);

	skb_queue_tail(&h4p->txq, skb);
	h4p_set_rts(hu, true);
	hci_uart_tx_wakeup(hu);

	/* disable BT wakeup (this may be checked by BT module during init) */
	/* TODO: fast enough? */
	tty_wait_until_sent(hu->tty, 0);
	gpiod_set_value(h4p->btdata->wakeup_bt, 0);

	if (!wait_for_completion_interruptible_timeout(&h4p->init_completion,
		msecs_to_jiffies(1000))) {
		return -ETIMEDOUT;
	}

	if (h4p->init_error < 0)
		return h4p->init_error;

	/* Change to operational settings */
	h4p_set_rts(hu, false);

	/* setup negotiated max. baudrate */
	h4p_set_speed(hu, MAX_BAUD_RATE);

	err = h4p_wait_for_cts(hu, true, 100);
	if (err < 0)
		return err;

	h4p_set_rts(hu, true);

	dev_dbg(hu->tty->dev, "Negotiation successful...\n");

	h4p->negotiated = true;

	/* re-enable BT wakeup */
	msleep(100);
	gpiod_set_value(h4p->btdata->wakeup_bt, 1);

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
		return 0;

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
			if(!bacmp(&h4p->bdaddr, BDADDR_ANY))
				set_bit(HCI_QUIRK_INVALID_BDADDR,
					&hu->hdev->quirks);
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

		if (evt->data[5] != 0x00) {
			dev_err(hu->tty->dev, "Sending FW cmd failed: 0x%.2x\n",
				evt->data[5]);
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

	return 0;
}

static int h4p_setup_fw(struct hci_uart *hu)
{
	const struct firmware *fw = NULL;
	int fw_pos, err;

	err = request_firmware(&fw, "bcmfw.bin", hu->tty->dev);
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

	dev_dbg(hu->tty->dev, "Nokia H4+ Protocol setup...\n");

	/* 0. verify connection using alive packet */
	err = h4p_reset(hu);
	if (err < 0) {
		dev_err(hu->tty->dev, "Reset failed: %d\n", err);
		pm_runtime_put(hu->tty->dev);
		return err;
	}

#if 0
	/* ~. verify connection using alive packet */
	err = h4p_send_alive_packet(hu);
	if (err < 0) {
		dev_err(hu->tty->dev, "Initial alive check failed: %d\n", err);
		return err;
	}

	dev_info(hu->tty->dev, "Bluetooth H4+ device found!\n");
#endif

	/* 1. negotiate speed etc */
	err = h4p_send_negotiation(hu);
	if (err < 0) {
		dev_err(hu->tty->dev, "Negotiation failed: %d\n", err);
		pm_runtime_put(hu->tty->dev);
		return err;
	}

	/* 2. verify correct setup using alive packet */
	err = h4p_send_alive_packet(hu);
	if (err < 0) {
		dev_err(hu->tty->dev, "Alive check failed: %d\n", err);
		pm_runtime_put(hu->tty->dev);
		return err;
	}

	/* 3. send firmware */
	err = h4p_setup_fw(hu);
	if (err < 0) {
		dev_err(hu->tty->dev, "Could not setup FW: %d\n", err);
		pm_runtime_put(hu->tty->dev);
		return err;
	}

	h4p_set_rts(hu, false);
	h4p_set_speed(hu, BC4_MAX_BAUD_RATE);
	h4p_set_rts(hu, true);

	pm_runtime_put(hu->tty->dev);

	return 0;
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
	h4p->negotiated = false;

	skb_queue_head_init(&h4p->txq);

	btdev = device_find_child(serialdev, NULL, btdev_match);
	if(!btdev) {
		dev_err(serialdev, "bcm2048 not found!\n");
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
	err = request_irq(h4p->wake_irq, wakeup_handler, IRQF_TRIGGER_RISING |
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

	BT_DBG("hu %p", hu);

	skb_queue_purge(&h4p->txq);

	return 0;
}

/* Close protocol */
static int h4p_close(struct hci_uart *hu)
{
	struct h4p_struct *h4p = hu->priv;

	hu->priv = NULL;

	BT_DBG("hu %p", hu);

	skb_queue_purge(&h4p->txq);

	kfree_skb(h4p->rx_skb);

	free_irq(h4p->wake_irq, h4p);

	/* disable module */
	gpiod_set_value(h4p->btdata->reset, 0);
	gpiod_set_value(h4p->btdata->wakeup_bt, 0);

	hu->priv = NULL;
	kfree(h4p);

	pm_runtime_disable(hu->tty->dev);

	clear_bit(HCI_QUIRK_INVALID_BDADDR, &hu->hdev->quirks);

	return 0;
}

/* Enqueue frame for transmittion (padding, crc, etc) */
static int h4p_enqueue(struct hci_uart *hu, struct sk_buff *skb)
{
	struct h4p_struct *h4p = hu->priv;
	u8 type = bt_cb(skb)->pkt_type;

	BT_DBG("hu %p skb %p", hu, skb);

	if(!h4p->negotiated && type != HCI_H4P_NEG_PKT) {
		dev_warn(hu->tty->dev, "skip sending message (type=%d) until negotiated!\n", type);
		return -EBUSY;
	}

	/* Prepend skb with frame type */
	memcpy(skb_push(skb, 1), &bt_cb(skb)->pkt_type, 1);
	skb_queue_tail(&h4p->txq, skb);

	return 0;
}

void h4p_recv_negotiation_packet(struct hci_uart *hu, struct sk_buff *skb)
{
	struct h4p_struct *h4p = hu->priv;
	struct hci_h4p_neg_hdr *hdr;
	struct hci_h4p_neg_evt *evt;

	hdr = (struct hci_h4p_neg_hdr *)skb->data;
	if (hdr->dlen != sizeof(*evt)) {
		h4p->init_error = -EIO;
		goto finish_neg;
	}

	evt = (struct hci_h4p_neg_evt *)skb_pull(skb, sizeof(*hdr));

	if (evt->ack != H4P_NEG_ACK) {
		dev_err(hu->tty->dev, "Could not negotiate hci_h4p settings\n");
		h4p->init_error = -EINVAL;
	}

	dev_dbg(hu->tty->dev, "H4P negotiation:\n");
	dev_dbg(hu->tty->dev, "\tbaudrate = %u\n", evt->baud);
	dev_dbg(hu->tty->dev, "\tsystem clock = %u\n", evt->sys_clk);
	dev_dbg(hu->tty->dev, "\tmanufacturer id = %u\n", evt->man_id);
	dev_dbg(hu->tty->dev, "\tversion id = %u\n", evt->ver_id);

finish_neg:
	complete(&h4p->init_completion);
	kfree_skb(skb);
}

void h4p_recv_alive_packet(struct hci_uart *hu, struct sk_buff *skb)
{
	struct h4p_struct *h4p = hu->priv;
	struct hci_h4p_alive_hdr *hdr;
	struct hci_h4p_alive_pkt *pkt;

	hdr = (struct hci_h4p_alive_hdr *)skb->data;
	if (hdr->dlen != sizeof(*pkt)) {
		dev_err(hu->tty->dev, "Corrupted alive message\n");
		h4p->init_error = -EIO;
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
	return;
}

/* Recv data */
static int h4p_reassembly(struct hci_dev *hdev, int type, void *data,
			  int count)
{
	int len = 0;
	int hlen = 0;
	int remain = count;
	struct sk_buff *skb;
	struct bt_skb_cb *scb;

	if (type < HCI_ACLDATA_PKT || (type > HCI_EVENT_PKT &&
	    type < HCI_H4P_NEG_PKT) || type > HCI_H4P_ALIVE_PKT) {
		BT_ERR("Invalid Package received, type=0x%02x\n", type);
		return -EILSEQ;
	}

	skb = hdev->reassembly[STREAM_REASSEMBLY];

	if (!skb) {
		switch (type) {
		case HCI_ACLDATA_PKT:
			len = HCI_MAX_FRAME_SIZE;
			hlen = HCI_ACL_HDR_SIZE;
			break;
		case HCI_EVENT_PKT:
			len = HCI_MAX_EVENT_SIZE;
			hlen = HCI_EVENT_HDR_SIZE;
			break;
		case HCI_SCODATA_PKT:
			len = HCI_MAX_SCO_SIZE;
			hlen = HCI_SCO_HDR_SIZE;
			break;
		case HCI_H4P_NEG_PKT:
			len = HCI_MAX_H4P_NEG_SIZE;
			hlen = HCI_H4P_NEG_HDR_SIZE;
			break;
		case HCI_H4P_ALIVE_PKT:
			len = HCI_MAX_H4P_ALIVE_SIZE;
			hlen = HCI_H4P_ALIVE_HDR_SIZE;
			break;
		}

		skb = bt_skb_alloc(len, GFP_ATOMIC);
		if (!skb)
			return -ENOMEM;

		scb = (void *) skb->cb;
		scb->expect = hlen;
		scb->pkt_type = type;

		hdev->reassembly[STREAM_REASSEMBLY] = skb;
	}

	while (count) {
		scb = (void *) skb->cb;
		len = min_t(uint, scb->expect, count);

		memcpy(skb_put(skb, len), data, len);

		count -= len;
		data += len;
		scb->expect -= len;
		remain = count;

		switch (type) {
		case HCI_EVENT_PKT:
			if (skb->len == HCI_EVENT_HDR_SIZE) {
				struct hci_event_hdr *h = hci_event_hdr(skb);
				scb->expect = h->plen;

				if (skb_tailroom(skb) < scb->expect) {
					kfree_skb(skb);
					hdev->reassembly[STREAM_REASSEMBLY]
						= NULL;
					return -ENOMEM;
				}
			}
			break;

		case HCI_ACLDATA_PKT:
			if (skb->len == HCI_ACL_HDR_SIZE) {
				struct hci_acl_hdr *h = hci_acl_hdr(skb);
				scb->expect = __le16_to_cpu(h->dlen);

				if (skb_tailroom(skb) < scb->expect) {
					kfree_skb(skb);
					hdev->reassembly[STREAM_REASSEMBLY]
						= NULL;
					return -ENOMEM;
				}
			}
			break;

		case HCI_SCODATA_PKT:
			if (skb->len == HCI_SCO_HDR_SIZE) {
				struct hci_sco_hdr *h = hci_sco_hdr(skb);
				scb->expect = h->dlen;

				if (skb_tailroom(skb) < scb->expect) {
					kfree_skb(skb);
					hdev->reassembly[STREAM_REASSEMBLY]
						= NULL;
					return -ENOMEM;
				}
			}
			break;

		case HCI_H4P_NEG_PKT:
			if (skb->len == HCI_H4P_NEG_HDR_SIZE) {
				struct hci_h4p_neg_hdr *h;
				h = hci_h4p_neg_hdr(skb);
				scb->expect = h->dlen;

				if (skb_tailroom(skb) < scb->expect) {
					kfree_skb(skb);
					hdev->reassembly[STREAM_REASSEMBLY]
						= NULL;
					return -ENOMEM;
				}
			}
			break;

		case HCI_H4P_ALIVE_PKT:
			if (skb->len == HCI_H4P_ALIVE_HDR_SIZE) {
				struct hci_h4p_alive_hdr *h;
				h = hci_h4p_alive_hdr(skb);
				scb->expect = h->dlen;

				if (skb_tailroom(skb) < scb->expect) {
					kfree_skb(skb);
					hdev->reassembly[STREAM_REASSEMBLY]
						= NULL;
					return -ENOMEM;
				}
			}
			break;

		}

		/* Complete frame */
		if (scb->expect == 0) {
			struct hci_uart *hu = hci_get_drvdata(hdev);

			/* H4+ devices send word aligned packets (incl. type) */
			//if ((skb->len % 2))
			//	remain--;

			bt_cb(skb)->pkt_type = type;

			dev_dbg(hu->tty->dev, "received packet of type=%d, len=%d\n", type, skb->len);
			print_hex_dump_bytes("received payload:", DUMP_PREFIX_NONE, skb->data, skb->len);

			if(type == HCI_EVENT_PKT) {
				if (skb->data[0] == HCI_EV_HARDWARE_ERROR)
					dev_warn(hu->tty->dev, "hardware error event!\n");
				else
					dev_warn(hu->tty->dev, "event code = %d!\n", skb->data[0]);
			}

			switch(type) {
			case HCI_H4P_ALIVE_PKT:
				h4p_recv_alive_packet(hu, skb);
				break;
			case HCI_H4P_NEG_PKT:
				h4p_recv_negotiation_packet(hu, skb);
				break;
			default:
				hci_recv_frame(hdev, skb);
				break;
			}

			hdev->reassembly[STREAM_REASSEMBLY] = NULL;
			return remain;
		}
	}

	return remain;
}

static int h4p_recv_stream_fragment(struct hci_dev *hdev, void *data, int count)
{
	int type;
	int rem = 0;

	while (count) {
		struct sk_buff *skb = hdev->reassembly[STREAM_REASSEMBLY];

		if (!skb) {
			struct { char type; } *pkt;

			/* Start of the frame */
			pkt = data;
			type = pkt->type;

			data++;
			count--;
		} else
			type = bt_cb(skb)->pkt_type;

		rem = h4p_reassembly(hdev, type, data, count);
		if (rem < 0)
			return rem;

		data += (count - rem);
		count = rem;
	}

	return rem;
}

static int h4p_recv(struct hci_uart *hu, void *data, int count)
{
	int ret;

	if (!test_bit(HCI_UART_REGISTERED, &hu->flags))
		return -EUNATCH;

	ret = h4p_recv_stream_fragment(hu->hdev, data, count);
	if (ret < 0) {
		BT_ERR("Frame Reassembly Failed: %d", ret);
		return ret;
	}

	return count;
}

static struct sk_buff *h4p_dequeue(struct hci_uart *hu)
{
	struct h4p_struct *h4p = hu->priv;
	return skb_dequeue(&h4p->txq);
}

static struct hci_uart_proto h4pp = {
	.id		= HCI_UART_H4P,
	.open		= h4p_open,
	.close		= h4p_close,
	.recv		= h4p_recv,
	.enqueue	= h4p_enqueue,
	.dequeue	= h4p_dequeue,
	.flush		= h4p_flush,
	.setup		= h4p_setup,
	// TODO: FIXME: setup, set_bdaddr
};

int __init h4p_init(void)
{
	int err = hci_uart_register_proto(&h4pp);

	if (!err)
		BT_INFO("HCI H4+ protocol initialized");
	else
		BT_ERR("HCI H4+ protocol registration failed");

	return err;
}

int __exit h4p_deinit(void)
{
	return hci_uart_unregister_proto(&h4pp);
}
