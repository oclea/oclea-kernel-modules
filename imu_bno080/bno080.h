#ifndef BNO080_H_
#define BNO080_H_

#include "bno080_api.h"

struct bno080_data {
	struct bno080_api_desc *api_desc;
	struct delayed_work post_init_work;
	struct delayed_work post_calib_work;
	const char *dev_name;
	unsigned int drdy_irq;
	unsigned int drdy_gpio;
	uint32_t acc_freq;
	uint32_t gyr_freq;
	uint32_t mag_freq;
	uint32_t rot_vect_freq;
	uint8_t acc_calib_status;
	uint8_t gyr_calib_status;
	uint8_t mag_calib_status;
	uint8_t rot_calib_status;
};

int bno080_core_probe(struct device *dev, struct bno080_api_hal_ops *hal_ops,
		      const char *name);
void bno080_core_remove(struct device *dev);

#endif /* BNO080_H_ */
