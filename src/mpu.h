#ifndef __MPU_H_
#define __MPU_H_

#include <drivers/sensor.h>

#define X_SCALE 10
#define Y_SCALE 10

void get_printable_sensor_str(struct sensor_value channel, char *out,
                              size_t sz);
bool read_gyro(struct device *dev, struct sensor_value *val);
void gyro_handler(struct device *dev, struct sensor_trigger *tr);
bool read_temp(struct device *dev);
void mpu_init();

#endif // __MPU_H_
