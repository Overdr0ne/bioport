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

#include <logging/log.h>

#include "analog.h"
#include "bioport.h"
#include "ble.h"
#include "button.h"
/* #include "mpu.h" */

#include "invensense/driver/eMPL/inv_mpu.h"
#include "invensense/driver/eMPL/inv_mpu_dmp_motion_driver.h"
#include "invensense/mllite/invensense.h"
#include "invensense/mllite/mpl.h"
#include "invensense/mpl/invensense_adv.h"
#include "invensense/eMPL-hal/eMPL_outputs.h"
#include "invensense/driver/include/mltypes.h"
#include "invensense/driver/include/mpu.h"
#include "invensense/driver/include/log.h"
#include "logging/log_core.h"
/* #include "invensense/stm32L/packet.h" */

/* unsigned char *mpl_key = (unsigned char*)"eMPL 5.1"; */
/* #define ACCEL_ON        (0x01) */
/* #define GYRO_ON         (0x02) */
/* #define COMPASS_ON      (0x04) */

/* #define MOTION          (0) */
/* #define NO_MOTION       (1) */

/* /\* Starting sampling rate. *\/ */
/* #define DEFAULT_MPU_HZ  (20) */

/* /\* #define FLASH_SIZE      (512) *\/ */
/* /\* #define FLASH_MEM_START ((void*)0x1800) *\/ */

/* #define PEDO_READ_MS    (1000) */
/* #define TEMP_READ_MS    (500) */
/* #define COMPASS_READ_MS (100) */

void main(void) {
  int ret;
  struct device *gpio_dev;
  struct device *mpu_dev;
  struct device *adc_dev;
  static s16_t m_sample_buffer[ANALOG_BUFFER_SIZE];
  const struct adc_sequence sequence = {
      .channels = BIT(ADC_1ST_CHANNEL_ID),
      /* .channels = BIT(ADC_2ND_CHANNEL_ID), */
      .buffer = m_sample_buffer,
      .buffer_size = sizeof(m_sample_buffer),
      .resolution = ADC_RESOLUTION,
  };

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

  /* adc_dev = init_adc(m_sample_buffer); */
  /* if (!adc_dev) { */
  /*   return; */
  /* } */

  /* while (1) { */
  /*   ret = adc_read(adc_dev, &sequence); */
  /*   if (!ret) { */
  /*     printk("adc_read: error %d\n", ret); */
  /*   } */
  /*   k_sleep(MSEC_PER_SEC / 4); */
  /*   adc_print_samples(1, m_sample_buffer); */
  /* } */
}

/* void main() { */
/*   inv_error_t result; */
/*   unsigned char accel_fsr, new_temp = 0; */
/*   unsigned short gyro_rate, gyro_fsr; */
/*   unsigned long timestamp; */
/*   struct int_param_s int_param; */

/*   /\* LOG_MODULE_REGISTER(bioport, LOG_LEVEL_ERR); *\/ */

/*   result = mpu_init(&int_param); */
/*   if (result) { */
/*     MPL_LOGE("Could not initialize gyro.\n"); */
/*   } */

/*   /\* If you're not using an MPU9150 AND you're not using DMP features, this */
/*    * function will place all slaves on the primary bus. */
/*    * mpu_set_bypass(1); */
/*    *\/ */

/*   result = inv_init_mpl(); */
/*   if (result) { */
/*     MPL_LOGE("Could not initialize MPL.\n"); */
/*   } */

/*   /\* Compute 6-axis and 9-axis quaternions. *\/ */
/*   inv_enable_quaternion(); */
/*   inv_enable_9x_sensor_fusion(); */
/*   /\* The MPL expects compass data at a constant rate (matching the rate */
/*    * passed to inv_set_compass_sample_rate). If this is an issue for your */
/*    * application, call this function, and the MPL will depend on the */
/*    * timestamps passed to inv_build_compass instead. */
/*    * */
/*    * inv_9x_fusion_use_timestamps(1); */
/*    *\/ */

/*   /\* Update gyro biases when not in motion. */
/*    * WARNING: These algorithms are mutually exclusive. */
/*    *\/ */
/*   inv_enable_fast_nomot(); */
/*   /\* inv_enable_motion_no_motion(); *\/ */
/*   /\* inv_set_no_motion_time(1000); *\/ */

/*   /\* Update gyro biases when temperature changes. *\/ */
/*   inv_enable_gyro_tc(); */

/*   /\* This algorithm updates the accel biases when in motion. A more accurate */
/*    * bias measurement can be made when running the self-test (see case 't' in */
/*    * handle_input), but this algorithm can be enabled if the self-test can't */
/*    * be executed in your application. */
/*    * */
/*    * inv_enable_in_use_auto_calibration(); */
/*    *\/ */

/* #ifdef COMPASS_ENABLED */
/*   /\* Compass calibration algorithms. *\/ */
/*   inv_enable_vector_compass_cal(); */
/*   inv_enable_magnetic_disturbance(); */
/* #endif */
/*   /\* If you need to estimate your heading before the compass is calibrated, */
/*    * enable this algorithm. It becomes useless after a good figure-eight is */
/*    * detected, so we'll just leave it out to save memory. */
/*    * inv_enable_heading_from_gyro(); */
/*    *\/ */

/*   /\* Allows use of the MPL APIs in read_from_mpl. *\/ */
/*   inv_enable_eMPL_outputs(); */

/*   result = inv_start_mpl(); */
/*   if (result == INV_ERROR_NOT_AUTHORIZED) { */
/*     while (1) { */
/*       MPL_LOGE("Not authorized.\n"); */
/*     } */
/*   } */
/*   if (result) { */
/*     MPL_LOGE("Could not start the MPL.\n"); */
/*   } */

/*   /\* Get/set hardware configuration. Start gyro. *\/ */
/*   /\* Wake up all sensors. *\/ */
/* #ifdef COMPASS_ENABLED */
/*   mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS); */
/* #else */
/*   mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL); */
/* #endif */
/*   /\* Push both gyro and accel data into the FIFO. *\/ */
/*   mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL); */
/*   mpu_set_sample_rate(DEFAULT_MPU_HZ); */
/* #ifdef COMPASS_ENABLED */
/*   /\* The compass sampling rate can be less than the gyro/accel sampling rate. */
/*    * Use this function for proper power management. */
/*    *\/ */
/*   mpu_set_compass_sample_rate(1000 / COMPASS_READ_MS); */
/* #endif */
/*   /\* Read back configuration in case it was set improperly. *\/ */
/*   mpu_get_sample_rate(&gyro_rate); */
/*   mpu_get_gyro_fsr(&gyro_fsr); */
/*   mpu_get_accel_fsr(&accel_fsr); */
/* #ifdef COMPASS_ENABLED */
/*   mpu_get_compass_fsr(&compass_fsr); */
/* #endif */
/*   /\* Sync driver configuration with MPL. *\/ */
/*   /\* Sample rate expected in microseconds. *\/ */
/*   inv_set_gyro_sample_rate(1000000L / gyro_rate); */
/*   inv_set_accel_sample_rate(1000000L / gyro_rate); */
/* #ifdef COMPASS_ENABLED */
/*   /\* The compass rate is independent of the gyro and accel rates. As long as */
/*    * inv_set_compass_sample_rate is called with the correct value, the 9-axis */
/*    * fusion algorithm's compass correction gain will work properly. */
/*    *\/ */
/*   inv_set_compass_sample_rate(COMPASS_READ_MS * 1000L); */
/* #endif */
/*   /\* Set chip-to-body orientation matrix. */
/*    * Set hardware units to dps/g's/degrees scaling factor. */
/*    *\/ */
/*   inv_set_gyro_orientation_and_scale( */
/*       inv_orientation_matrix_to_scalar(gyro_pdata.orientation), */
/*       (long)gyro_fsr << 15); */
/*   inv_set_accel_orientation_and_scale( */
/*       inv_orientation_matrix_to_scalar(gyro_pdata.orientation), */
/*       (long)accel_fsr << 15); */
/* #ifdef COMPASS_ENABLED */
/*   inv_set_compass_orientation_and_scale( */
/*       inv_orientation_matrix_to_scalar(compass_pdata.orientation), */
/*       (long)compass_fsr << 15); */
/* #endif */
/*   /\* Initialize HAL state variables. *\/ */
/* #ifdef COMPASS_ENABLED */
/*   hal.sensors = ACCEL_ON | GYRO_ON | COMPASS_ON; */
/* #else */
/*   hal.sensors = ACCEL_ON | GYRO_ON; */
/* #endif */
/*   hal.dmp_on = 0; */
/*   hal.report = 0; */
/*   hal.rx.cmd = 0; */
/*   hal.next_pedo_ms = 0; */
/*   hal.next_compass_ms = 0; */
/*   hal.next_temp_ms = 0; */

/* } */
