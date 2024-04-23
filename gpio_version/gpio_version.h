/*
 * gpio_version.h - GPIO version
 */
#ifndef _LINUX_GPIO_VER_H
#define _LINUX_GPIO_VER_H

#include <linux/gpio/consumer.h>

#define MAX_VERSION_NAME_SIZE 10

struct gpio_version_platform_data {
	struct gpio_descs *input_gpios;
	const char **version_names;
	int max_versions;
};

#endif /* _LINUX_GPIO_VER_H */
