/* main.c - Application main entry point */

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

#include <settings/settings.h>

#include <device.h>
#include <drivers/gpio.h>
#include <drivers/sensor.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/conn.h>
#include <bluetooth/gatt.h>
#include <bluetooth/hci.h>
#include <bluetooth/uuid.h>

#include "bioport.h"
#include "mouse.h"

/* #define LOG_LEVEL LOG_LEVEL_DBG */
/* LOG_MODULE_REGISTER(main); */

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, 0x12, 0x18, /* HID Service */
                  0x0f, 0x18),                    /* Battery Service */
};

static void connected(struct bt_conn *conn, u8_t err) {
  char addr[BT_ADDR_LE_STR_LEN];

  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

  if (err) {
    printk("Failed to connect to %s (%u)\n", addr, err);
    return;
  }

  printk("Connected %s\n", addr);

  if (bt_conn_set_security(conn, BT_SECURITY_L2)) {
    printk("Failed to set security\n");
  }
}

static void disconnected(struct bt_conn *conn, u8_t reason) {
  char addr[BT_ADDR_LE_STR_LEN];

  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

  printk("Disconnected from %s (reason 0x%02x)\n", addr, reason);
}

static void security_changed(struct bt_conn *conn, bt_security_t level,
                             enum bt_security_err err) {
  char addr[BT_ADDR_LE_STR_LEN];

  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

  if (!err) {
    printk("Security changed: %s level %u", addr, level);
  } else {
    printk("Security failed: %s level %u err %d", addr, level, err);
  }
}

static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
    .security_changed = security_changed,
};

static void bt_ready(int err) {
  if (err) {
    printk("Bluetooth init failed (err %d)\n", err);
    return;
  }

  printk("Bluetooth initialized\n");

  mouse_init();

  if (IS_ENABLED(CONFIG_SETTINGS)) {
    settings_load();
  }

  err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
  if (err) {
    printk("Advertising failed to start (err %d)\n", err);
    return;
  }

  printk("Advertising successfully started\n");
}

static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey) {
  char addr[BT_ADDR_LE_STR_LEN];

  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

  printk("Passkey for %s: %06u\n", addr, passkey);
}

static void auth_cancel(struct bt_conn *conn) {
  char addr[BT_ADDR_LE_STR_LEN];

  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

  printk("Pairing cancelled: %s\n", addr);
}

static struct bt_conn_auth_cb auth_cb_display = {
    .passkey_display = auth_passkey_display,
    .passkey_entry = NULL,
    .cancel = auth_cancel,
};

void button_pressed(struct device *gpiob, struct gpio_callback *cb,
                    u32_t pins) {
  mouse_notify();
  printk("Button pressed at %" PRIu32 "\n", k_cycle_get_32());
}

static struct gpio_callback gpio_cb;

void get_printable_sensor_str(struct sensor_value channel, char *out,
                              size_t sz) {
  if ((channel.val1 < 0) || (channel.val2 < 0)) {
    snprintk(out, sz, "-%u.%u", -channel.val1, -channel.val2);
  } else {
    snprintk(out, sz, "%u.%u", channel.val1, channel.val2);
  }
  return;
}

static bool read_gyro(struct device *dev) {
  struct sensor_value val[3];
  int ret;

  ret = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, val);
  if (ret < 0) {
    printk("Cannot read sensor channels");
    return false;
  }

  /* For printing double we need to use printf with
   * printf("%10.6f\n", sensor_value_to_double(x));
   */
  printk("int parts: X %d Y %d", val[0].val1, val[1].val1);

  /* TODO: Add proper calculations */

  /* if (val[0].val1 != 0) { */
  /*   status[MOUSE_X_REPORT_POS] = val[0].val1 * 4; */
  /* } */

  /* if (val[1].val1 != 0) { */
  /*   status[MOUSE_Y_REPORT_POS] = val[1].val1 * 4; */
  /* } */

  /* if (val[0].val1 != 0 || val[1].val1 != 0) { */
  /*   return true; */
  /* } else { */
  /*   return false; */
  /* } */
}

static void trigger_handler(struct device *dev, struct sensor_trigger *tr) {
  ARG_UNUSED(tr);

  /* Always fetch the sample to clear the data ready interrupt in the
   * sensor.
   */
  if (sensor_sample_fetch(dev)) {
    printk("sensor_sample_fetch failed");
    return;
  }

  if (read_gyro(dev)) {
    /* k_sem_give(&sem); */
  }
}

void main(void) {
  int err;
  struct bt_le_conn_param param;
  struct device *gpiob;
  struct device *mpu_dev;
  /* struct sensor_value gyrox, gyroy, gyroz; */
  /* struct sensor_value magnx, magny, magnz; */
  /* char gxs[16]; */
  /* char gys[16]; */
  /* char gzs[16]; */
  /* char mxs[16]; */
  /* char mys[16]; */
  /* char mzs[16]; */

  err = bt_enable(bt_ready);
  if (err) {
    printk("Bluetooth init failed (err %d)\n", err);
    return;
  }

  param.interval_min = 10;
  param.interval_max = 50;
  param.latency = 0;
  param.timeout = 400;

  err = bt_conn_le_param_update(NULL, &param);

  bt_conn_cb_register(&conn_callbacks);
  bt_conn_auth_cb_register(&auth_cb_display);

  gpiob = device_get_binding(GPIO_PORT);
  if (!gpiob) {
    printk("error\n");
    return;
  }

  printk("1\n");
  gpio_pin_configure(gpiob, BTN_GPIO_PIN,
                     GPIO_DIR_IN | GPIO_INT | GPIO_PULL_UP | GPIO_EDGE);

  gpio_init_callback(&gpio_cb, button_pressed, BIT(BTN_GPIO_PIN));

  gpio_add_callback(gpiob, &gpio_cb);
  gpio_pin_enable_callback(gpiob, BTN_GPIO_PIN);

  mpu_dev = device_get_binding("MPU6050");

  struct sensor_trigger trig = {
      .type = SENSOR_TRIG_DATA_READY,
      .chan = SENSOR_CHAN_GYRO_XYZ,
  };

  printk("2\n");
  if (sensor_trigger_set(mpu_dev, &trig, trigger_handler)) {
    printk("Could not set trigger");
    return;
  }

  printk("3\n");
  /* while (1) { */
  /*   k_sleep(MSEC_PER_SEC / 4); */
  /*   sensor_sample_fetch(mpu_dev); */
  /*   sensor_channel_get(mpu_dev, SENSOR_CHAN_GYRO_X, &gyrox); */
  /*   get_printable_sensor_str(gyrox, gxs, sizeof(gxs)); */
  /*   sensor_channel_get(mpu_dev, SENSOR_CHAN_GYRO_Y, &gyroy); */
  /*   get_printable_sensor_str(gyroy, gys, sizeof(gys)); */
  /*   sensor_channel_get(mpu_dev, SENSOR_CHAN_GYRO_Z, &gyroz); */
  /*   get_printable_sensor_str(gyroz, gzs, sizeof(gzs)); */

  /*   sensor_channel_get(mpu_dev, SENSOR_CHAN_MAGN_X, &magnx); */
  /*   get_printable_sensor_str(magnx, mxs, sizeof(mxs)); */
  /*   sensor_channel_get(mpu_dev, SENSOR_CHAN_MAGN_Y, &magny); */
  /*   get_printable_sensor_str(magny, mys, sizeof(mys)); */
  /*   sensor_channel_get(mpu_dev, SENSOR_CHAN_MAGN_Z, &magnz); */
  /*   get_printable_sensor_str(magnz, mzs, sizeof(mzs)); */

  /*   /\* gx = gyrox.val1 + (gyrox.val2 / 1000000); *\/ */
  /*   /\* gy = gyroy.val1 + (gyroy.val2 / 1000000); *\/ */
  /*   /\* gz = gyroz.val1 + (gyroz.val2 / 1000000); *\/ */
  /*   /\* printk("ypr\t%s%u.%05u\t%s%u.%05u\t%s%u.%05u\n", gxs, gyrox.val1, *\/
   */
  /*   /\*        gyrox.val2, gys, gyroy.val1, gyroy.val2, gzs, gyroz.val1, *\/
   */
  /*   /\*        gyroz.val2); *\/ */
  /*   printk("gyro:\t%s\t%s\t%s\n", gxs, gys, gzs); */
  /*   printk("magn:\t%s\t%s\t%s\n", mxs, mys, mzs); */
  /*   /\* printk("gyroy: %i.%i\n", gyroy.val1, gyroy.val2); *\/ */
  /*   /\* printk("gyroz: %i.%i\n", gyroz.val1, gyroz.val2); *\/ */
  /*   /\* if ((gyrox.val1 != 0) || (gyrox.val2 != 0)) { *\/ */
  /*   /\*   mouse_notify(); *\/ */
  /*   /\* } *\/ */
  /* } */

  /* while (1) { */
  /*   k_sleep(MSEC_PER_SEC); */
  /* 	/\* k_sleep(500); *\/ */

  /* 	/\* Heartrate measurements simulation *\/ */
  /* 	/\* mouse_notify(); *\/ */

  /* 	/\* Battery level simulation *\/ */
  /* 	/\* bas_notify(); *\/ */
  /* } */
}
