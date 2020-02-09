/** @file
 *  @brief mouse Service sample
 */

/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stddef.h>
#include <string.h>
#include <sys/byteorder.h>
#include <sys/printk.h>
#include <zephyr.h>
#include <zephyr/types.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/conn.h>
#include <bluetooth/gatt.h>
#include <bluetooth/hci.h>
#include <bluetooth/uuid.h>

#include <usb/usb_device.h>
#include <usb/class/usb_hid.h>

#include "mouse.h"

enum {
  HIDS_REMOTE_WAKE = BIT(0),
  HIDS_NORMALLY_CONNECTABLE = BIT(1),
};

struct hids_info {
  u16_t version; /* version number of base USB HID Specification */
  u8_t code;     /* country HID Device hardware is localized for. */
  u8_t flags;
} __packed;

struct hids_report {
  u8_t id;   /* report id */
  u8_t type; /* report type */
} __packed;

static struct hids_info info = {
    .version = 0x0000,
    .code = 0x00,
    .flags = HIDS_NORMALLY_CONNECTABLE,
};

enum {
  HIDS_INPUT = 0x01,
  HIDS_OUTPUT = 0x02,
  HIDS_FEATURE = 0x03,
};

static struct hids_report input = {
    .id = 0x01,
    .type = HIDS_INPUT,
};

#define BCNT 3

static u8_t simulate_input;
static u8_t ctrl_point;
static u8_t report_map[] = {
    HID_GI_USAGE_PAGE,     USAGE_GEN_DESKTOP,
    HID_LI_USAGE,          USAGE_GEN_DESKTOP_MOUSE,
    HID_MI_COLLECTION,     COLLECTION_APPLICATION,
      HID_LI_USAGE,          USAGE_GEN_DESKTOP_POINTER,
      HID_MI_COLLECTION,     COLLECTION_PHYSICAL,
        HID_GI_USAGE_PAGE,     USAGE_GEN_BUTTON,
        HID_LI_USAGE_MIN(1),   1,
        HID_LI_USAGE_MAX(1),   BCNT,
        HID_GI_LOGICAL_MIN(1), 0,
        HID_GI_LOGICAL_MAX(1), 1,
        HID_GI_REPORT_SIZE,    1,
        HID_GI_REPORT_COUNT,   BCNT,
        HID_MI_INPUT,          0x02,
        HID_GI_REPORT_SIZE,    (8 - BCNT),
        HID_GI_REPORT_COUNT,   1,

        HID_MI_INPUT,          0x03,
        HID_GI_USAGE_PAGE,     USAGE_GEN_DESKTOP,
        HID_LI_USAGE,          USAGE_GEN_DESKTOP_X,
        HID_LI_USAGE,          USAGE_GEN_DESKTOP_Y,
        /* HID_LI_USAGE,          USAGE_GEN_DESKTOP_WHEEL, */
        HID_GI_LOGICAL_MIN(2), 0x01, 0x80, /* 2s comp -32767 */
        HID_GI_LOGICAL_MAX(2), 0xff, 0x7f, /* 2s comp 32767 */
        /* HID_GI_LOGICAL_MIN(2), 0xfe, 0x0c, /\* 2s comp -32767 *\/ */
        /* HID_GI_LOGICAL_MAX(2), 0xf4, 0x01, /\* 2s comp 32767 *\/ */
        HID_GI_REPORT_SIZE,    16,
        HID_GI_REPORT_COUNT,   2,
        HID_MI_INPUT,          0x06,
        /* HID_MI_INPUT,          0x02, */
      HID_MI_COLLECTION_END,
    HID_MI_COLLECTION_END,
};

static ssize_t read_info(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                         void *buf, u16_t len, u16_t offset) {
  return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data,
                           sizeof(struct hids_info));
}

static ssize_t read_report_map(struct bt_conn *conn,
                               const struct bt_gatt_attr *attr, void *buf,
                               u16_t len, u16_t offset) {
  return bt_gatt_attr_read(conn, attr, buf, len, offset, report_map,
                           sizeof(report_map));
}

static ssize_t read_report(struct bt_conn *conn,
                           const struct bt_gatt_attr *attr, void *buf,
                           u16_t len, u16_t offset) {
  return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data,
                           sizeof(struct hids_report));
}

static void input_ccc_changed(const struct bt_gatt_attr *attr, u16_t value) {
  simulate_input = (value == BT_GATT_CCC_NOTIFY) ? 1 : 0;
}

static ssize_t read_input_report(struct bt_conn *conn,
                                 const struct bt_gatt_attr *attr, void *buf,
                                 u16_t len, u16_t offset) {
  return bt_gatt_attr_read(conn, attr, buf, len, offset, NULL, 0);
}

static ssize_t write_ctrl_point(struct bt_conn *conn,
                                const struct bt_gatt_attr *attr,
                                const void *buf, u16_t len, u16_t offset,
                                u8_t flags) {
  u8_t *value = attr->user_data;

  if (offset + len > sizeof(ctrl_point)) {
    return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
  }

  memcpy(value + offset, buf, len);

  return len;
}

/* HID Service Declaration */
BT_GATT_SERVICE_DEFINE(
    mouse_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_HIDS),
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_INFO, BT_GATT_CHRC_READ,
                           BT_GATT_PERM_READ, read_info, NULL, &info),
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT_MAP, BT_GATT_CHRC_READ,
                           BT_GATT_PERM_READ, read_report_map, NULL, NULL),
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ_AUTHEN, read_input_report, NULL,
                           NULL),
    BT_GATT_CCC(input_ccc_changed,
                BT_GATT_PERM_READ_AUTHEN | BT_GATT_PERM_WRITE_AUTHEN),
    BT_GATT_DESCRIPTOR(BT_UUID_HIDS_REPORT_REF, BT_GATT_PERM_READ, read_report,
                       NULL, &input),
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_CTRL_POINT,
                           BT_GATT_CHRC_WRITE_WITHOUT_RESP, BT_GATT_PERM_WRITE,
                           NULL, write_ctrl_point, &ctrl_point), );

void serialize(struct mouse_status *status) {
  status->packet[0] = (s8_t)status->button;
  status->packet[1] = (s8_t)(status->pos[0]);
  status->packet[2] = (s8_t)(status->pos[0] >> 8);
  status->packet[3] = (s8_t)(status->pos[1]);
  status->packet[4] = (s8_t)(status->pos[1] >> 8);
}
/* void serialize(struct mouse_status *status) { */
/*   status->packet[0] = (s8_t)status->button; */
/*   status->packet[1] = 0x7f; */
/*   status->packet[2] = 0x7f; */
/*   status->packet[3] = 0x7f; */
/*   status->packet[4] = 0x7f; */
/* } */

void print_packet(struct mouse_status *status) {
  int i;
  for(i=0; i<sizeof(status->packet); i+=1) {
    printk("%x ",status->packet[i]);
  }
  printk("\n\n");
}

void mouse_init(void) {}

int mouse_notify(struct mouse_status *status) {
  int rc;
  static s8_t mouse[5];

  serialize(status);
  /* printk("%i %i\n", status->pos[0], status->pos[1]); */
  print_packet(status);

  rc = bt_gatt_notify(NULL, &mouse_svc.attrs[4], status->packet, sizeof(status->packet));

  return rc == -ENOTCONN ? 0 : rc;
}
