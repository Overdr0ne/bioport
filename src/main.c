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

#include "analog.h"
#include "bioport.h"
#include "ble.h"
#include "mouse.h"
#include "mpu.h"

/* #define LOG_LEVEL LOG_LEVEL_DBG */
/* LOG_MODULE_REGISTER(main); */

static struct sensor_trigger gyro_trig = {
    .type = SENSOR_TRIG_DATA_READY,
    .chan = SENSOR_CHAN_GYRO_XYZ,
};

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

void main(void) {
  int err, ret;
  struct bt_le_conn_param param;
  struct device *gpiob;
  struct device *mpu_dev;
  struct device *adc_dev;
  static s16_t m_sample_buffer[ANALOG_BUFFER_SIZE];

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

  gpio_pin_configure(gpiob, BTN_GPIO_PIN,
                     GPIO_DIR_IN | GPIO_INT | GPIO_PULL_UP | GPIO_EDGE);

  gpio_init_callback(&gpio_cb, button_pressed, BIT(BTN_GPIO_PIN));

  gpio_add_callback(gpiob, &gpio_cb);
  gpio_pin_enable_callback(gpiob, BTN_GPIO_PIN);

  mpu_dev = device_get_binding("MPU6050");

  if (sensor_trigger_set(mpu_dev, &gyro_trig, gyro_handler)) {
    printk("Could not set trigger");
    return;
  }

  const struct adc_sequence sequence = {
      .channels = BIT(ADC_1ST_CHANNEL_ID),
      .buffer = m_sample_buffer,
      .buffer_size = sizeof(m_sample_buffer),
      .resolution = ADC_RESOLUTION,
  };
  adc_dev = init_adc(m_sample_buffer);
  if (!adc_dev) {
    return;
  }

  ret = adc_read(adc_dev, &sequence);
  if (!ret) {
    printk("adc_read: error %d\n", ret);
  }

  /* struct sensor_trigger temp_trig = { */
  /*     .type = SENSOR_TRIG_DATA_READY, */
  /*     .chan = SENSOR_CHAN_AMBIENT_TEMP, */
  /* }; */

  /* if (sensor_trigger_set(mpu_dev, &temp_trig, temp_handler)) { */
  /*   printk("Could not set trigger"); */
  /*   return; */
  /* } */

  while (1) {
    k_sleep(MSEC_PER_SEC / 4);
    read_temp(mpu_dev);
    // check_samples(1, m_sample_buffer);
    /* printk("gyroy: %i.%i\n", gyroy.val1, gyroy.val2); */
    /* printk("gyroz: %i.%i\n", gyroz.val1, gyroz.val2); */
    /* if ((gyrox.val1 != 0) || (gyrox.val2 != 0)) { */
    /*   mouse_notify(); */
    /* } */
  }

  /* while (1) { */
  /*   k_sleep(MSEC_PER_SEC); */
  /* 	/\* k_sleep(500); *\/ */

  /* 	/\* Heartrate measurements simulation *\/ */
  /* 	/\* mouse_notify(); *\/ */

  /* 	/\* Battery level simulation *\/ */
  /* 	/\* bas_notify(); *\/ */
  /* } */
}
