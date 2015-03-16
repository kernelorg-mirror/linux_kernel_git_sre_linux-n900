/*
 *  Copyright (C) 2015  Sebastian Reichel <sre@kernel.org>
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
 */

#ifndef __H4P_H
#define __H4P_H

struct h4p_dev_struct {
	struct device *dev;
	struct tty_port *port;
	struct gpio_desc *reset;
	struct gpio_desc *wakeup_host;
	struct gpio_desc *wakeup_bt;
	unsigned long sysclk_speed;
};

#define H4P_ID_CSR	0x02
#define H4P_ID_BCM2048	0x04
#define H4P_ID_TI1271	0x31

#define FIRMWARE_CSR "bc4fw.bin"
#define FIRMWARE_BCM2048 "bcmfw.bin"
#define FIRMWARE_TI1271 "ti1273.bin"

#define HCI_H4P_BCM_BDADDR	0xfc01

#define HCI_H4P_NEG_PKT		0x06
#define HCI_H4P_ALIVE_PKT	0x07
#define HCI_H4P_RADIO_PKT	0x08

#define HCI_H4P_NEG_HDR_SIZE	1
#define HCI_MAX_H4P_NEG_SIZE	255
#define HCI_H4P_ALIVE_HDR_SIZE	1
#define HCI_MAX_H4P_ALIVE_SIZE	255
#define HCI_H4P_RADIO_HDR_SIZE	2

#define H4P_PROTO_PKT		0x44
#define H4P_PROTO_BYTE		0x4c

#define H4P_NEG_REQ		0x00
#define H4P_NEG_ACK		0x20
#define H4P_NEG_NAK		0x40

#define H4_TYPE_SIZE		1

#define H4P_RECV_ACL \
	H4_RECV_ACL, \
	.wordaligned = true

#define H4P_RECV_SCO \
	H4_RECV_SCO, \
	.wordaligned = true

#define H4P_RECV_EVENT \
	H4_RECV_EVENT, \
	.wordaligned = true

#define H4P_RECV_ALIVE \
	.type = HCI_H4P_ALIVE_PKT, \
	.hlen = HCI_H4P_ALIVE_HDR_SIZE, \
	.loff = 0, \
	.lsize = 1, \
	.maxlen = HCI_MAX_H4P_ALIVE_SIZE, \
	.wordaligned = true

#define H4P_RECV_NEG \
	.type = HCI_H4P_NEG_PKT, \
	.hlen = HCI_H4P_NEG_HDR_SIZE, \
	.loff = 0, \
	.lsize = 1, \
	.maxlen = HCI_MAX_H4P_NEG_SIZE, \
	.wordaligned = true

struct hci_h4p_neg_hdr {
	__u8	dlen;
} __packed;

struct hci_h4p_neg_cmd {
	__u8	ack;
	__u16	baud;
	__u16	unused1;
	__u8	proto;
	__u16	sys_clk;
	__u16	unused2;
} __packed;

static inline struct hci_h4p_neg_hdr *hci_h4p_neg_hdr(const struct sk_buff *skb)
{
	return (struct hci_h4p_neg_hdr *) skb->data;
}

#define H4P_ALIVE_REQ   0x55
#define H4P_ALIVE_RESP  0xcc

struct hci_h4p_alive_hdr {
	__u8	dlen;
} __packed;

struct hci_h4p_alive_pkt {
	__u8	mid;
	__u8	unused;
} __packed;

static inline struct hci_h4p_alive_hdr *hci_h4p_alive_hdr(const struct sk_buff *skb)
{
	return (struct hci_h4p_alive_hdr *) skb->data;
}

struct hci_h4p_neg_evt {
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

struct hci_h4p_radio_hdr {
	__u8	evt;
	__u8	dlen;
} __packed;

#endif
