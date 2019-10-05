#ifndef __BIOPORT_H_
#define __BIOPORT_H_

/* change this to use another GPIO port */
#define GPIO_PORT DT_ALIAS_SW0_GPIOS_CONTROLLER

/* change this to use another GPIO pin */
#define BTN_GPIO_PIN DT_ALIAS_SW0_GPIOS_PIN

/* change to use another GPIO pin interrupt config */
#define GPIO_EDGE (DT_ALIAS_SW0_GPIOS_FLAGS | GPIO_INT_EDGE)

/* change this to enable pull-up/pull-down */
#define GPIO_PULL_UP DT_ALIAS_SW0_GPIOS_FLAGS

#endif // __BIOPORT_H_
