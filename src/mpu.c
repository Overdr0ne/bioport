#include "mpu.h"
#include "mouse.h"
#include "zephyr/types.h"
#include <string.h>

static const int X_RES = 1920;
static const int Y_RES = 1080;
static const int X_MAX_ROT_INT_PT = 10;
static const int Y_MAX_ROT_INT_PT = 10;
/* static const int INT_STPS = 3; */
static const int FRAC_RES = 10000000;

static float X_PIX_PER_INT_DIV, Y_PIX_PER_INT_DIV;
static float X_PIX_PER_FRAC_DIV, Y_PIX_PER_FRAC_DIV;
static struct sensor_value prev_val[3];
static struct sensor_value center_val[3];
static bool start = true;
static int int_cnt = 0;

static struct sensor_trigger gyro_trig = {
    .type = SENSOR_TRIG_DATA_READY,
    .chan = SENSOR_CHAN_GYRO_XYZ,
};

inline float GET_PIX_PER_DIV(const int res, const int max_divs) {
  return (float)res / 2 / max_divs;
}

bool read_gyro(struct device *dev, struct sensor_value *val) {
  float dval[3];
  int ret;

  ret = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, val);
  if (ret < 0) {
    printk("Cannot read sensor channels");
    return false;
  }

  if (val[0].val1 != 0 || val[1].val1 != 0 || val[2].val1 != 0)
    return true;
  return false;
}

void get_printable_sensor_str(struct sensor_value channel, char *out,
                              size_t sz) {
  if ((channel.val1 < 0) || (channel.val2 < 0)) {
    snprintk(out, sz, "-%u.%u", -channel.val1, -channel.val2);
  } else {
    snprintk(out, sz, "%u.%u", channel.val1, channel.val2);
  }
  return;
}

void print_gyro(struct sensor_value *val) {
  char gxs[16];
  char gys[16];
  char gzs[16];

  get_printable_sensor_str(val[0], gxs, sizeof(gxs));
  get_printable_sensor_str(val[1], gys, sizeof(gys));
  get_printable_sensor_str(val[2], gzs, sizeof(gzs));
  /* printk("gyro: X %s Y %s Z %s\n", gxs, gys, gzs); */

  /* For printing double we need to use printf with
   * printf("%10.6f\n", sensor_value_to_double(x));
   */
  // dval[0] = sensor_value_to_double(&val[0]);
  // dval[1] = sensor_value_to_double(&val[1]);
  // dval[2] = sensor_value_to_double(&val[2]);
  // printk("gyro: X %f Y %f Z %f\n", dval[0], dval[1], dval[2]);
}

void set_mouse_pos(struct sensor_value *val, s16_t *mouse_pos) {
  s32_t x, y;

  mouse_pos[MOUSE_BTN_REPORT_POS] = 0x00;

  x = ((val[0].val1 - center_val[0].val1) * X_PIX_PER_INT_DIV) +
      ((val[0].val2 - center_val[0].val2) * X_PIX_PER_FRAC_DIV);
  /* x = (val[0].val1 - center_val[0].val1) + */
  /*     (val[0].val2 - center_val[0].val2) / 1000000; */
  /* x = x * X_SCALE; */
  mouse_pos[MOUSE_X_REPORT_POS] = (s16_t)x;
  /* printk("x: %i\n", x); */

  /* y = ((val[1].val1 - center_val[1].val1) * Y_PIX_PER_INT_DIV) + */
  /*     ((val[1].val2 - center_val[1].val2) * Y_PIX_PER_FRAC_DIV); */
  y = (val[1].val1 - center_val[1].val1) +
      (val[1].val2 - center_val[1].val2) / 1000000;
  /* y = y * Y_SCALE; */
  mouse_pos[MOUSE_Y_REPORT_POS] = (s16_t)y;
  /* printk("y: %i\n", y); */

  /* printk("size: %i %i\n", sizeof(val[1].val1), sizeof(int)); */

  /* printk("val: x:: %i,%i y:: %i,%i", (val[0].val1 - center_val[0].val1), */
  /*        (val[0].val2 - center_val[0].val2), (val[1].val1 -
   * center_val[1].val1), */
  /*        (val[1].val2 - center_val[1].val2)); */
}

void copy_sensor_val(struct sensor_value *from, struct sensor_value *to) {
  int i;

  for (i = 0; i < 3; i += 1) {
    to[i].val1 = from[i].val1;
    to[i].val2 = from[i].val2;
  }
}

/* void pre_filter_gyro(struct sensor_value *gyro_val) {} */

void gyro_handler(struct device *dev, struct sensor_trigger *tr) {
  struct sensor_value val[3];
  struct mouse_status ms;

  ARG_UNUSED(tr);

  k_sleep(MSEC_PER_SEC / 10);

  /* Always fetch the sample to clear the data ready interrupt in the
   * sensor.
   */
  if (sensor_sample_fetch(dev)) {
    printk("sensor_sample_fetch failed");
    return;
  }
  /* printk("1\n"); */

  read_gyro(dev, val);
  if (start) {
    k_sleep(MSEC_PER_SEC);
    read_gyro(dev, val);
    copy_sensor_val(val, center_val);
    start = false;
    goto finish;
  }
  set_mouse_pos(val, ms.pos);
  ms.button = 0;
  /* ms.position = mouse_pos; */
  /* printk("dx %i dy %i\n", ms.pos[MOUSE_X_REPORT_POS], */
  /*        ms.pos[MOUSE_Y_REPORT_POS]); */
  mouse_notify(&ms);
finish:
  /* int_cnt = (int_cnt + 1) % INT_STPS; */
  return;
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

void mpu_init(struct device *mpu_dev) {
  X_PIX_PER_INT_DIV = GET_PIX_PER_DIV(X_RES, X_MAX_ROT_INT_PT);
  Y_PIX_PER_INT_DIV = GET_PIX_PER_DIV(Y_RES, Y_MAX_ROT_INT_PT);
  X_PIX_PER_FRAC_DIV = GET_PIX_PER_DIV(X_RES, X_MAX_ROT_INT_PT * FRAC_RES);
  Y_PIX_PER_FRAC_DIV = GET_PIX_PER_DIV(Y_RES, Y_MAX_ROT_INT_PT * FRAC_RES);

  if (sensor_trigger_set(mpu_dev, &gyro_trig, gyro_handler)) {
    printk("Could not set trigger");
    return;
  }
}
