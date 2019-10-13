#ifndef __MPU_H_
#define __MPU_H_

#include <drivers/sensor.h>

void get_printable_sensor_str(struct sensor_value channel, char *out,
                              size_t sz);
bool read_gyro(struct device *dev);
void gyro_handler(struct device *dev, struct sensor_trigger *tr);
bool read_temp(struct device *dev);

#endif // __MPU_H_
