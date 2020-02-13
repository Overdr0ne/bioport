#ifndef __BUTTON_H_
#define __BUTTON_H_

#include <device.h>
#include <drivers/gpio.h>
#include <zephyr.h>

#include "bioport.h"
#include "mouse.h"

void button_init(struct device *gpio_dev);

#endif // __BUTTON_H_
