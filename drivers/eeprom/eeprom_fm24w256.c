/*
 * Copyright (c) 2021 Caspar Friedrich <c.s.w.friedrich@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <devicetree.h>
#include <devicetree/gpio.h>
#include <drivers/eeprom.h>
#include <drivers/gpio.h>
#include <drivers/i2c.h>
#include <logging/log.h>

#define DT_DRV_COMPAT cypress_fm24w256

#define LEVEL POST_KERNEL
#define PRIORITY CONFIG_SPI_INIT_PRIORITY

// 256-Kbit ferroelectric random access memory logically organized as 32K × 8 bit
#define SIZE KB(32)

LOG_MODULE_REGISTER(fm24w256, CONFIG_EEPROM_LOG_LEVEL);

struct config {
	const char *i2c_label;
	uint16_t i2c_addr;
	const char *wp_label;
	gpio_pin_t wp_pin;
	gpio_dt_flags_t wp_flags;
};

struct data {
	const struct device *i2c_dev;
	const struct device *wp_dev;
};

static int init(const struct device *dev)
{
	const struct config *config = dev->config;
	struct data *data = dev->data;

	data->i2c_dev = device_get_binding(config->i2c_label);
	if (!data->i2c_dev) {
		LOG_ERR("Failed get i2c device binding");
		return -ENODEV;
	}

	if (!config->wp_label) {
		LOG_INF("No wp-gpio configured");
		return 0;
	}

	data->wp_dev = device_get_binding(config->wp_label);
	if (!data->wp_dev) {
		LOG_ERR("Failed get wp-gpio device binding");
		return -ENODEV;
	}

	LOG_DBG("Write Protect pin: {%s, %d, %d}", config->wp_label, config->wp_pin,
		config->wp_flags);

	/*
	 * Write protect is not supported by this driver. If it's configured
	 * make sure it's inactive.
	 */
	return gpio_pin_configure(data->wp_dev, config->wp_pin,
				  config->wp_flags | GPIO_OUTPUT_INACTIVE);
}

static int memory_operation(const struct device *dev, off_t offset, const void *buf, size_t buf_len,
			    bool is_write)
{
	if (offset > SIZE) {
		LOG_ERR("Offset out of range");
		return -ENOMEM;
	}

	if (offset + buf_len > SIZE) {
		LOG_WRN("Memory roll over");
	}

	uint8_t addr_buf[] = {
		(offset >> 8) & 0xff,
		offset & 0xff,
	};

	struct i2c_msg msg[] = {
		{
			.buf = addr_buf,
			.len = ARRAY_SIZE(addr_buf),
			.flags = I2C_MSG_WRITE,
		},
		{
			.buf = (uint8_t *)buf,
			.len = buf_len,
			.flags = (is_write ? I2C_MSG_WRITE : I2C_MSG_RESTART | I2C_MSG_READ) |
				 I2C_MSG_STOP,
		},
	};

	const struct config *config = dev->config;
	struct data *data = dev->data;

	return i2c_transfer(data->i2c_dev, msg, ARRAY_SIZE(msg), config->i2c_addr);
}

static int read(const struct device *dev, off_t offset, void *buf, size_t buf_len)
{
	LOG_DBG("About to read %u bytes from 0x%04x", buf_len, offset);
	return memory_operation(dev, offset, buf, buf_len, false);
}

static int write(const struct device *dev, off_t offset, const void *buf, size_t buf_len)
{
	LOG_DBG("About to write %u bytes to 0x%04x", buf_len, offset);
	return memory_operation(dev, offset, buf, buf_len, true);
}

static size_t size(const struct device *dev)
{
	return SIZE;
}

static struct eeprom_driver_api api = {
	.read = read,
	.write = write,
	.size = size,
};

#define INIT(n)                                                                                    \
	static struct data inst_##n##_data = { 0 };                                                \
	static const struct config inst_##n##_config = {                                           \
		.i2c_label = DT_INST_BUS_LABEL(n),                                                 \
		.i2c_addr = DT_INST_REG_ADDR(n),                                                   \
		.wp_label = DT_INST_GPIO_LABEL_OR(n, wp_gpios, NULL),                              \
		.wp_pin = DT_INST_GPIO_PIN_OR(n, wp_gpios, 0),                                     \
		.wp_flags = DT_INST_GPIO_FLAGS_OR(n, wp_gpios, 0),                                 \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, &init, NULL, &inst_##n##_data, &inst_##n##_config, LEVEL,         \
			      PRIORITY, &api);

DT_INST_FOREACH_STATUS_OKAY(INIT)
