#include "mpu.h"

void get_printable_sensor_str(struct sensor_value channel, char *out,
                              size_t sz) {
  if ((channel.val1 < 0) || (channel.val2 < 0)) {
    snprintk(out, sz, "-%u.%u", -channel.val1, -channel.val2);
  } else {
    snprintk(out, sz, "%u.%u", channel.val1, channel.val2);
  }
  return;
}

bool read_gyro(struct device *dev) {
  struct sensor_value val[3];
  int ret;
  char gxs[16];
  char gys[16];
  char gzs[16];

  ret = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, val);
  if (ret < 0) {
    printk("Cannot read sensor channels");
    return false;
  }

  /* For printing double we need to use printf with
   * printf("%10.6f\n", sensor_value_to_double(x));
   */
  get_printable_sensor_str(val[0], gxs, sizeof(gxs));
  get_printable_sensor_str(val[1], gys, sizeof(gys));
  get_printable_sensor_str(val[2], gzs, sizeof(gzs));
  /* printk("gyro: X %s Y %s Z %s", gxs, gys, gzs); */

  /* TODO: Add proper calculations */

  /* if (val[0].val1 != 0) { */
  /*   status[MOUSE_X_REPORT_POS] = val[0].val1 * 4; */
  /* } */

  /* if (val[1].val1 != 0) { */
  /*   status[MOUSE_Y_REPORT_POS] = val[1].val1 * 4; */
  /* } */

  if (val[0].val1 != 0 || val[1].val1 != 0 || val[2].val1 != 0)
    return true;
  return false;
}

void gyro_handler(struct device *dev, struct sensor_trigger *tr) {
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

bool read_temp(struct device *dev) {
  struct sensor_value val;
  char ts[16];
  int ret;

  get_printable_sensor_str(val, ts, sizeof(ts));
  ret = sensor_channel_get(dev, SENSOR_CHAN_DIE_TEMP, &val);
  if (ret < 0) {
    printk("Cannot read sensor channels");
    return false;
  }
  // printk("temp: %s\n", ts);

  return true;
}
