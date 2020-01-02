#include "mpu.h"
#include "mouse.h"
#include "sensor.h"
#include "zephyr/types.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

static const int MPU_RES = 65536;
static const int X_RES = 65536;
static const int Y_RES = 65536;
static const int Z_RES = 65536;
static const int X_MAX_ROT_INT_PT = 10;
static const int Y_MAX_ROT_INT_PT = 10;
static const int Z_MAX_ROT_INT_PT = 10;
/* static const int INT_STPS = 3; */
static const int FRAC_RES = 10000000;
static const int M_PI = 3.14159;

static float X_PIX_PER_INT_DIV, Y_PIX_PER_INT_DIV, Z_PIX_PER_INT_DIV;
static float X_PIX_PER_FRAC_DIV, Y_PIX_PER_FRAC_DIV, Z_PIX_PER_FRAC_DIV;
static float X_DIV_PER_PIX, Y_DIV_PER_PIX;
static struct sensor_value prev_val[3];
static struct sensor_value center_val[3];
static bool start = true;
static int int_cnt = 0;

struct mpu_t {
  s16_t gyro[3];
  s16_t acc[3];
};

static struct sensor_trigger acc_trig = {
    .type = SENSOR_TRIG_DATA_READY,
    .chan = SENSOR_CHAN_ACCEL_XYZ,
};
static struct sensor_trigger gyro_trig = {
    .type = SENSOR_TRIG_DATA_READY,
    .chan = SENSOR_CHAN_GYRO_XYZ,
};

inline float GET_PIX_PER_DIV(const int res, const int max_divs) {
  return (float)res / 2 / max_divs;
}

bool read_channel(struct device *dev, struct sensor_value *val, unsigned int channel) {
  float dval[3];
  int ret;

  ret = sensor_channel_get(dev, channel, val);
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

void print_sensor(struct sensor_value *val) {

  printk("X: %i.%i\nY: %i.%i\nZ: %i.%i\n", val[0].val1, val[0].val2,
         val[1].val1, val[1].val2, val[2].val1, val[2].val2);
}

void scale_2_u16(struct sensor_value *val, s16_t *scaled) {
  s32_t x, y, z;

  x = ((val[0].val1 - center_val[0].val1) * X_PIX_PER_INT_DIV) +
      ((val[0].val2 - center_val[0].val2) * X_PIX_PER_FRAC_DIV);
  scaled[0] = (s16_t)val[0].val1 / X_DIV_PER_PIX;
  scaled[0] *= 1.5;
  /* printk("x: %i\n", x); */

  y = ((val[1].val1 - center_val[1].val1) * Y_PIX_PER_INT_DIV) +
      ((val[1].val2 - center_val[1].val2) * Y_PIX_PER_FRAC_DIV);
  scaled[1] = (s16_t)val[1].val1 / Y_DIV_PER_PIX;
  scaled[1] *= 1.5;
  /* printk("y: %i\n", y); */

  z = ((val[2].val1 - center_val[2].val1) * Z_PIX_PER_INT_DIV) +
      ((val[2].val2 - center_val[2].val2) * Z_PIX_PER_FRAC_DIV);
  scaled[2] = (s16_t)val[2].val1;
  scaled[2] *= 1.5;
  /* printk("z: %i\n", z); */
}

void set_mouse_pos(struct mpu_t *mpu, struct mouse_status *ms) {
  ms->pos[0] = mpu->gyro[0];
  ms->pos[1] = mpu->gyro[1];
}

void copy_sensor_val(struct sensor_value *from, struct sensor_value *to) {
  int i;

  for (i = 0; i < 3; i += 1) {
    to[i].val1 = from[i].val1;
    to[i].val2 = from[i].val2;
  }
}

void complementary_filter(struct mpu_t *mpu) {
  float pitch, roll, pitchAcc, rollAcc;
  float dt = .1;

  // Integrate the gyroscope data -> int(angularSpeed) = angle
  pitch += (float)mpu->gyro[0] * dt; // Angle around the X-axis
  roll -= (float)mpu->gyro[1] * dt; // Angle around the Y-axis

  // Compensate for drift with accelerometer data if !bullshit
  // Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
  int forceMagnitudeApprox =
      abs(mpu->acc[0]) + abs(mpu->acc[1]) + abs(mpu->acc[2]);
  if (forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768) {
    // Turning around the X axis results in a vector on the Y-axis
    pitchAcc = tan((float)mpu->acc[1], (float)mpu->acc[2]) * 180 / M_PI;
    pitch = pitch * 0.98 + pitchAcc * 0.02;

    // Turning around the Y axis results in a vector on the X-axis
    rollAcc = atan2f((float)mpu->acc[0], (float)mpu->acc[2]) * 180 / M_PI;
    roll = roll * 0.98 + rollAcc * 0.02;
  }

  mpu->gyro[0] = pitch;
  mpu->gyro[1] = roll;
}

void mpu_filter(struct mpu_t *mpu) {
  complementary_filter(mpu);
}

void mpu_print(struct mpu_t *mpu) {
  printk("gyro: x: %i\ny: %i\nz: %i\n", mpu->gyro[0],mpu->gyro[1],mpu->gyro[2]);
}

/* void pre_filter_gyro(struct sensor_value *gyro_val) {} */

void gyro_handler(struct device *dev, struct sensor_trigger *tr) {
  struct mpu_t mpu;
  struct sensor_value gyro_raw[3];
  struct sensor_value acc_raw[3];
  struct mouse_status ms;
	/* struct mpu6050_data *drv_data = dev->driver_data; */

  ARG_UNUSED(tr);

  /* k_sleep(MSEC_PER_SEC); */

  /* Always fetch the sample to clear the data ready interrupt in the
   * sensor.
   */
  if (sensor_sample_fetch(dev)) {
    printk("sensor_sample_fetch failed");
    return;
  }

  /* printk("raw: %i\n",drv_data->gyro_x); */
  read_channel(dev, gyro_raw, SENSOR_CHAN_GYRO_XYZ);
  /* print_sensor(gyro_raw); */
  /* read_channel(dev, acc_raw, SENSOR_CHAN_ACCEL_XYZ); */
  if (start) {
    k_sleep(MSEC_PER_SEC);
    read_channel(dev, gyro_raw, SENSOR_CHAN_GYRO_XYZ);
    copy_sensor_val(gyro_raw, center_val);
    start = false;
    goto finish;
  }
  scale_2_u16(gyro_raw, mpu.gyro);
  mpu_print(&mpu);
  /* scale_2_u16(acc_raw, mpu.acc); */
  /* mpu_filter(&mpu); */

  set_mouse_pos(&mpu, &ms);
  ms.button = 0;
  mouse_notify(&ms);
finish:
  /* int_cnt = (int_cnt + 1) % INT_STPS; */
  return;
}

/* void acc_handler(struct device *dev, struct sensor_trigger *tr) { */
/*   struct sensor_value val[3]; */
/*   struct mouse_status ms; */

/*   ARG_UNUSED(tr); */

/*   k_sleep(MSEC_PER_SEC); */

/*   /\* Always fetch the sample to clear the data ready interrupt in the */
/*    * sensor. */
/*    *\/ */
/*   if (sensor_sample_fetch(dev)) { */
/*     printk("sensor_sample_fetch failed"); */
/*     return; */
/*   } */

/*   read_sensor(dev, val, SENSOR_CHAN_ACCEL_XYZ); */
/*   print_sensor(val); */
/*   if (start) { */
/*     k_sleep(MSEC_PER_SEC); */
/*     read_gyro(dev, val); */
/*     copy_sensor_val(val, center_val); */
/*     start = false; */
/*     goto finish; */
/*   } */
/*   set_mouse_pos(val, ms.pos); */
/*   ms.button = 0; */
/*   /\* ms.position = mouse_pos; *\/ */
/*   /\* printk("dx %i dy %i\n", ms.pos[MOUSE_X_REPORT_POS], *\/ */
/*   /\*        ms.pos[MOUSE_Y_REPORT_POS]); *\/ */
/*   mouse_notify(&ms); */
/* finish: */
/*   /\* int_cnt = (int_cnt + 1) % INT_STPS; *\/ */
/*   return; */
/* } */

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
  Z_PIX_PER_INT_DIV = GET_PIX_PER_DIV(Z_RES, Z_MAX_ROT_INT_PT);
  X_PIX_PER_FRAC_DIV = GET_PIX_PER_DIV(X_RES, X_MAX_ROT_INT_PT * FRAC_RES);
  Y_PIX_PER_FRAC_DIV = GET_PIX_PER_DIV(Y_RES, Y_MAX_ROT_INT_PT * FRAC_RES);
  Z_PIX_PER_FRAC_DIV = GET_PIX_PER_DIV(Z_RES, Z_MAX_ROT_INT_PT * FRAC_RES);
  X_DIV_PER_PIX = MPU_RES / 1920;
  Y_DIV_PER_PIX = MPU_RES / 1080;

  if (sensor_trigger_set(mpu_dev, &gyro_trig, gyro_handler)) {
    printk("Could not set trigger");
    return;
  }
}
