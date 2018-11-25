/*
 *  Shared Transport Header file
 *	To be included by the protocol stack drivers for
 *	Texas Instruments BT,FM and GPS combo chip drivers
 *	and also serves the sub-modules of the shared transport driver.
 *
 *  Copyright (C) 2009-2010 Texas Instruments
 *  Author: Pavan Savoy <pavan_savoy@ti.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
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

#ifndef TI_WILINK_ST_H
#define TI_WILINK_ST_H

#include <linux/skbuff.h>

void hci_ti_set_fm_handler(struct device *dev, void (*recv_handler) (void *, struct sk_buff *), void *drvdata);
int hci_ti_fm_send(struct device *dev, struct sk_buff *skb);

/*
 * BTS headers
 */
#define ACTION_SEND_COMMAND     1
#define ACTION_WAIT_EVENT       2
#define ACTION_SERIAL           3
#define ACTION_DELAY            4
#define ACTION_RUN_SCRIPT       5
#define ACTION_REMARKS          6

/**
 * struct bts_header - the fw file is NOT binary which can
 *	be sent onto TTY as is. The .bts is more a script
 *	file which has different types of actions.
 *	Each such action needs to be parsed by the KIM and
 *	relevant procedure to be called.
 */
struct bts_header {
	u32 magic;
	u32 version;
	u8 future[24];
	u8 actions[0];
} __attribute__ ((packed));

/**
 * struct bts_action - Each .bts action has its own type of
 *	data.
 */
struct bts_action {
	u16 type;
	u16 size;
	u8 data[0];
} __attribute__ ((packed));

struct bts_action_send {
	u8 data[0];
} __attribute__ ((packed));

struct bts_action_wait {
	u32 msec;
	u32 size;
	u8 data[0];
} __attribute__ ((packed));

struct bts_action_delay {
	u32 msec;
} __attribute__ ((packed));

struct bts_action_serial {
	u32 baud;
	u32 flow_control;
} __attribute__ ((packed));

/**
 * struct hci_command - the HCI-VS for intrepreting
 *	the change baud rate of host-side UART, which
 *	needs to be ignored, since UIM would do that
 *	when it receives request from KIM for ldisc installation.
 */
struct hci_command {
	u8 prefix;
	u16 opcode;
	u8 plen;
	u32 speed;
} __attribute__ ((packed));

/*
 * header information used by st_core.c for FM and GPS
 * packet parsing, the bluetooth headers are already available
 * at net/bluetooth/
 */

struct fm_event_hdr {
	u8 plen;
} __attribute__ ((packed));

#define FM_MAX_FRAME_SIZE 0xFF	/* TODO: */
#define FM_EVENT_HDR_SIZE 1	/* size of fm_event_hdr */
#define ST_FM_CH8_PKT 0x8

/* gps stuff */
struct gps_event_hdr {
	u8 opcode;
	u16 plen;
} __attribute__ ((packed));

#endif /* TI_WILINK_ST_H */
