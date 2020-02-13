#include "button.h"

static void button_pressed(struct device *gpio_dev, struct gpio_callback *cb,
                           u32_t pins) {
  // mouse_notify();
  printk("Button pressed at %" PRIu32 "\n", k_cycle_get_32());
}

void button_init(struct device *gpio_dev) {
  struct gpio_callback *gpio_cb;

  gpio_pin_configure(gpio_dev, BTN_GPIO_PIN,
                     GPIO_DIR_IN | GPIO_INT | GPIO_PULL_UP | GPIO_EDGE);

  gpio_init_callback(gpio_cb, button_pressed, BIT(BTN_GPIO_PIN));

  gpio_add_callback(gpio_dev, gpio_cb);
  gpio_pin_enable_callback(gpio_dev, BTN_GPIO_PIN);
}
