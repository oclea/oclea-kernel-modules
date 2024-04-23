#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>

#include "bno080.h"

#define BNO080_I2C_READ_RETRIES 3
#define BNO080_I2C_READ_DELAY 10

int bno080_i2c_read_multiple_byte(struct device *dev, uint8_t *data, int len)
{
	struct i2c_msg in;
	struct i2c_client *client;
	int ret, retries;

	client = to_i2c_client(dev);
	retries = BNO080_I2C_READ_RETRIES;

	do {
		mdelay(BNO080_I2C_READ_DELAY);

		in.addr = client->addr;
		in.flags = I2C_M_RD | I2C_M_IGNORE_NAK;
		in.len = len;
		in.buf = data;

		ret = i2c_transfer(client->adapter, &in, 1);
		if (ret < 0) {
			dev_err(dev, "i2c read failed with code [%d], retry\n",
				ret);
		} else {
			return len;
		}
	} while (--retries > 0);

	return -EINVAL;
}

int bno080_i2c_write_multiple_byte(struct device *dev, uint8_t *data, int len)
{
	struct i2c_msg out;
	struct i2c_client *client;
	int ret;

	client = to_i2c_client(dev);

	out.addr = client->addr;
	out.flags = client->flags | I2C_M_IGNORE_NAK;
	out.len = len;
	out.buf = data;

	ret = i2c_transfer(client->adapter, &out, 1);
	if (ret < 0) {
		dev_err(dev, "i2c write failed\n");
	} else {
		ret = len;
	}

	return ret;
}

static struct bno080_api_hal_ops bno080_i2c_hal_ops = {
	.read_multiple_byte = bno080_i2c_read_multiple_byte,
	.write_multiple_byte = bno080_i2c_write_multiple_byte,
	.read_len_limit = 128,
};

static int bno080_i2c_probe(struct i2c_client *client,
			    const struct i2c_device_id *id)
{
	const char *name = NULL;

	if (id)
		name = id->name;

	return bno080_core_probe(&client->dev, &bno080_i2c_hal_ops, name);
}

static int bno080_i2c_remove(struct i2c_client *client)
{
	bno080_core_remove(&client->dev);

	return 0;
}

static const struct i2c_device_id bno080_i2c_id[] = { { "bno080", 0 }, {} };
MODULE_DEVICE_TABLE(i2c, bno080_i2c_id);

static const struct of_device_id bno080_dt_ids[] = {
	{
		.compatible = "oclea,bno080",
	},
	{},
};

static struct i2c_driver bno080_i2c_driver = {
	.driver =
		{
			.name = "bno080",
			.of_match_table = bno080_dt_ids,
		},
	.probe = bno080_i2c_probe,
	.remove = bno080_i2c_remove,
	.id_table = bno080_i2c_id,
};
module_i2c_driver(bno080_i2c_driver);

MODULE_AUTHOR("Gym Ok Cho <kimogi@teknique.com>");
MODULE_DESCRIPTION("Hillcrest Labs BNO080 I2C driver");
MODULE_LICENSE("GPL v2");
