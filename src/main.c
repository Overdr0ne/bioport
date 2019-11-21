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
#include "button.h"
#include "mpu.h"

void main(void) {
  int ret;
  struct device *gpio_dev;
  struct device *mpu_dev;
  struct device *adc_dev;
  static s16_t m_sample_buffer[ANALOG_BUFFER_SIZE];

  ble_init();

  /*
    gpio_dev = device_get_binding(GPIO_PORT);
    if (!gpio_dev) {
    printk("error\n");
    return;
    }
    button_init(gpio_dev);
  */
  printk("initializing...\n");
  mpu_dev = device_get_binding("MPU6050");
  mpu_init(mpu_dev);

  adc_dev = init_adc(m_sample_buffer);
  if (!adc_dev) {
    return;
  }
  const struct adc_sequence sequence = {
      .channels = BIT(ADC_1ST_CHANNEL_ID),
      .buffer = m_sample_buffer,
      .buffer_size = sizeof(m_sample_buffer),
      .resolution = ADC_RESOLUTION,
  };
  ret = adc_read(adc_dev, &sequence);
  if (!ret) {
    printk("adc_read: error %d\n", ret);
  }

  /* while (1) { */
  /*   k_sleep(MSEC_PER_SEC / 4); */
  // read_temp(mpu_dev);
  // adc_print_samples(1, m_sample_buffer);
  /* printk("gyroy: %i.%i\n", gyroy.val1, gyroy.val2); */
  /* printk("gyroz: %i.%i\n", gyroz.val1, gyroz.val2); */
  /* if ((gyrox.val1 != 0) || (gyrox.val2 != 0)) { */
  /*   mouse_notify(); */
  /* } */
  /* } */
}
