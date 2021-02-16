/*
 * Copyright (c) 2021 Caspar Friedrich <c.s.w.friedrich@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <devicetree.h>
#include <devicetree/gpio.h>
#include <drivers/eeprom.h>
#include <drivers/gpio.h>
#include <drivers/spi.h>
#include <logging/log.h>

#define DT_DRV_COMPAT cypress_fm25w256

#define LEVEL POST_KERNEL
#define PRIORITY CONFIG_SPI_INIT_PRIORITY

// 256-Kbit ferroelectric random access memory logically organized as 32K × 8 bit
#define SIZE KB(32)
#define WORD_SIZE 8

LOG_MODULE_REGISTER(fm25w256, CONFIG_EEPROM_LOG_LEVEL);

// Opcode Commands
enum {
	OPCODE_WREN = 0b00000110,
	OPCODE_WRDI = 0b00000100,
	OPCODE_RDSR = 0b00000101,
	OPCODE_WRSR = 0b00000001,
	OPCODE_READ = 0b00000011,
	OPCODE_WRITE = 0b00000010,
};

struct config {
	const char *spi_label;
	uint16_t spi_slave;
	uint32_t spi_frequency;
	const char *cs_label;
	gpio_pin_t cs_pin;
	gpio_dt_flags_t cs_flags;
	const char *wp_label;
	gpio_pin_t wp_pin;
	gpio_dt_flags_t wp_flags;
	const char *hold_label;
	gpio_pin_t hold_pin;
	gpio_dt_flags_t hold_flags;
};

struct data {
	const struct device *spi_dev;
	const struct device *wp_dev;
	const struct device *hold_dev;
	struct spi_config spi_config;
	struct spi_cs_control spi_cs_control;
};

static int init_write_protect(const struct config *config, struct data *data)
{
	data->wp_dev = device_get_binding(config->wp_label);
	if (!data->wp_dev) {
		LOG_ERR("Failed get wp-gpio device binding");
		return -ENODEV;
	}

	LOG_DBG("WP pin: {%s, %d, %d}", config->wp_label, config->wp_pin, config->wp_flags);

	/*
	 * Write protect is not yet supported by this driver. If it's configured
	 * make sure it's inactive.
	 */
	return gpio_pin_configure(data->wp_dev, config->wp_pin,
				  config->wp_flags | GPIO_OUTPUT_INACTIVE);
}

static int init_hold(const struct config *config, struct data *data)
{
	data->hold_dev = device_get_binding(config->hold_label);
	if (!data->hold_dev) {
		LOG_ERR("Failed get hold-gpio device binding");
		return -ENODEV;
	}

	LOG_DBG("HOLD pin: {%s, %d, %d}", config->hold_label, config->hold_pin, config->hold_flags);

	/*
	 * Hold is not yet supported by this driver. If it's configured make
	 * sure it's inactive.
	 */
	return gpio_pin_configure(data->hold_dev, config->hold_pin,
				  config->hold_flags | GPIO_OUTPUT_INACTIVE);
}

static int init(const struct device *dev)
{
	int err;

	const struct config *config = dev->config;
	struct data *data = dev->data;

	data->spi_dev = device_get_binding(config->spi_label);
	if (!data->spi_dev) {
		LOG_ERR("Failed to get spi device binding");
		return -ENODEV;
	}

	data->spi_config.cs = &data->spi_cs_control;
	data->spi_config.frequency = config->spi_frequency;
	data->spi_config.slave = config->spi_slave;
	data->spi_config.operation = SPI_WORD_SET(WORD_SIZE) | SPI_TRANSFER_MSB;

	if (IS_ENABLED(CONFIG_EEPROM_FM25W256_SPI_MODE_3)) {
		data->spi_config.operation |= SPI_MODE_CPOL | SPI_MODE_CPHA;
	}

	data->spi_cs_control.gpio_dev = device_get_binding(config->cs_label);
	if (!data->spi_cs_control.gpio_dev) {
		LOG_ERR("Failed get cs-gpio device binding");
		return -ENODEV;
	}

	data->spi_cs_control.gpio_pin = config->cs_pin;
	data->spi_cs_control.gpio_dt_flags = config->cs_flags;
	data->spi_cs_control.delay = 0;

	if (config->wp_label) {
		err = init_write_protect(config, data);
		if (err) {
			LOG_ERR("Unable to configure WP pin: %d", err);
			return err;
		}
	} else {
		LOG_WRN("No WP pin configured");
	}

	if (config->hold_label) {
		err = init_hold(config, data);
		if (err) {
			LOG_ERR("Unable to configure HOLD pin: %d", err);
			return err;
		}
	} else {
		LOG_WRN("No HOLD pin configured");
	}

	return 0;
}

static int write_enable(const struct device *dev)
{
	uint8_t opcode[] = { OPCODE_WREN };

	const struct spi_buf buf[] = {
		{
			.buf = opcode,
			.len = sizeof(opcode),
		},
	};

	struct spi_buf_set tx = {
		.buffers = buf,
		.count = 1,
	};

	struct data *data = dev->data;

	return spi_write(data->spi_dev, &data->spi_config, &tx);
}

static int memory_operation(const struct device *dev, off_t offset, void *buf, size_t buf_len,
			    bool is_write)
{
	if (offset > SIZE) {
		LOG_ERR("Offset out of range");
		return -ENOMEM;
	}

	if (offset + buf_len > SIZE) {
		LOG_WRN("Memory roll over");
	}

	LOG_DBG("About to %s %u bytes %s 0x%04x", is_write ? "write" : "read", buf_len,
		is_write ? "to" : "from", offset);

	uint8_t opcode[] = {
		is_write ? OPCODE_WRITE : OPCODE_READ,
	};

	uint8_t mem_addr[] = {
		(offset >> 8) & 0xff,
		offset & 0xff,
	};

	const struct spi_buf spi_buf[] = {
		{
			.buf = opcode,
			.len = sizeof(opcode),
		},
		{
			.buf = mem_addr,
			.len = sizeof(mem_addr),
		},
		{
			.buf = buf,
			.len = buf_len,
		},
	};

	struct spi_buf_set tx_buf_set = {
		.buffers = spi_buf,
		.count = is_write ? ARRAY_SIZE(spi_buf) : ARRAY_SIZE(spi_buf) - 1,
	};

	struct spi_buf_set rx_buf_set = {
		.buffers = spi_buf,
		.count = ARRAY_SIZE(spi_buf),
	};

	if (is_write) {
		int err = write_enable(dev);
		if (err) {
			LOG_ERR("Unable to set write enable latch: %d", err);
			return err;
		}
	}

	struct data *data = dev->data;

	return spi_transceive(data->spi_dev, &data->spi_config, &tx_buf_set,
			      is_write ? NULL : &rx_buf_set);
}

static int read(const struct device *dev, off_t offset, void *buf, size_t buf_len)
{
	LOG_DBG("About to read %u bytes from 0x%04x", buf_len, offset);
	return memory_operation(dev, offset, buf, buf_len, false);
}

static int write(const struct device *dev, off_t offset, const void *buf, size_t buf_len)
{
	LOG_DBG("About to write %u bytes to 0x%04x", buf_len, offset);
	return memory_operation(dev, offset, (void *)buf, buf_len, true);
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
		.spi_label = DT_INST_BUS_LABEL(n),                                                 \
		.spi_slave = DT_INST_REG_ADDR(n),                                                  \
		.spi_frequency = DT_INST_PROP(n, spi_max_frequency),                               \
		.cs_label = DT_INST_SPI_DEV_CS_GPIOS_LABEL(n),                                     \
		.cs_pin = DT_INST_SPI_DEV_CS_GPIOS_PIN(n),                                         \
		.cs_flags = DT_INST_SPI_DEV_CS_GPIOS_FLAGS(n),                                     \
		.wp_label = DT_INST_GPIO_LABEL_OR(n, wp_gpios, NULL),                              \
		.wp_pin = DT_INST_GPIO_PIN_OR(n, wp_gpios, 0),                                     \
		.wp_flags = DT_INST_GPIO_FLAGS_OR(n, wp_gpios, 0),                                 \
		.hold_label = DT_INST_GPIO_LABEL_OR(n, hold_gpios, NULL),                          \
		.hold_pin = DT_INST_GPIO_PIN_OR(n, hold_gpios, 0),                                 \
		.hold_flags = DT_INST_GPIO_FLAGS_OR(n, hold_gpios, 0),                             \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, &init, NULL, &inst_##n##_data, &inst_##n##_config, LEVEL,         \
			      PRIORITY, &api);

DT_INST_FOREACH_STATUS_OKAY(INIT)
