#include <stdio.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "../../../include/zephyr/drivers/radio.h"

#define Z_RADIOLIB_SX127X_REG_VERSION 0x42
#define SX1280_INIT_PRIORITY 90

#if DT_HAS_COMPAT_STATUS_OKAY(semtech_sx1280)
#else
#error No sx1278 instance in device tree.
#endif

#define DT_DRV_COMPAT semtech_sx1280

LOG_MODULE_REGISTER(sx1280, CONFIG_LOG_DEFAULT_LEVEL);

#define SX1280_DIO_GPIO_LEN(inst) DT_INST_PROP_LEN(inst, dio_gpios)

#define SX1280_DIO_GPIO_ELEM(idx, inst) \
  GPIO_DT_SPEC_INST_GET_BY_IDX(inst, dio_gpios, idx)

#define SX1280_DIO_GPIO_INIT(n) \
  LISTIFY(SX1280_DIO_GPIO_LEN(n), SX1280_DIO_GPIO_ELEM, (, ), n)

const struct gpio_dt_spec sx1280_dios[] = {SX1280_DIO_GPIO_INIT(0)};

#define SX1280_MAX_DIO ARRAY_SIZE(sx1280_dios)

static const struct radio_device_config dev_config = {
    .bus = SPI_DT_SPEC_INST_GET(
        0, SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0),
    .reset = GPIO_DT_SPEC_INST_GET(0, reset_gpios),
    .cs = GPIO_DT_SPEC_INST_GET(0, manual_cs_gpios),
    .dio_count = SX1280_MAX_DIO,
    .dios = {SX1280_DIO_GPIO_INIT(0)},
};

const struct gpio_dt_spec *get_gpio_from_pin(uint32_t pin) {
  const struct gpio_dt_spec *gpio;
  for (int i = 0; i < dev_config.dio_count; i++) {
    if (dev_config.dios[i].pin == pin) {
      gpio = &dev_config.dios[i];
    }
  }
  if (dev_config.reset.pin == pin) {
    gpio = &dev_config.reset;
  }
  if (dev_config.cs.pin == pin) {
    gpio = &dev_config.cs;
  }
  return gpio;
}

int sx1278_get_config(const struct device *dev,
                      const struct radio_device_config **config) {
  LOG_INF("sx1278_get_config %p", &dev_config);
  *config = &dev_config;
  return 0;
}

int sx1278_configure_pin(const struct device *dev, uint32_t pin,
                         uint32_t mode) {
  __ASSERT(mode <= DIO_OUTPUT, "Invalid Mode %d", mode);
  // TODO: Check Pin Number is valid

  const struct gpio_dt_spec *gpio = get_gpio_from_pin(pin);
  LOG_INF("Configuring gpio %s, pin %d with mode 0x%02x", gpio->port->name,
          gpio->pin, mode);

  if (gpio != NULL) {
    int err;
    if (mode == DIO_INPUT) {
      err = gpio_pin_configure_dt(gpio, GPIO_INPUT);
    } else if (mode == DIO_OUTPUT) {
      err = gpio_pin_configure_dt(gpio, GPIO_OUTPUT);
    }
    if (err) {
      LOG_ERR("Cannot configure gpio %s %d, Error Code: %d", gpio->port->name,
              gpio->pin, err);
    }
  } else {
    LOG_ERR("Cannot get gpio for pin %d", pin);
    return -EINVAL;
  }
  LOG_INF("Pin configured.");
  return 0;
}

int sx1278_digital_write(const struct device *dev, uint32_t pin,
                         uint32_t value) {
  const struct gpio_dt_spec *gpio = get_gpio_from_pin(pin);

  int pin_val = 0;
  if (value == DIO_PIN_HIGH) {
    pin_val = 1;
  }
  LOG_INF("Pin: %2d:%d", pin, pin_val);
  return gpio_pin_set_dt(gpio, pin_val);
}

int sx1278_digital_read(const struct device *dev, uint32_t pin) {
  const struct gpio_dt_spec *gpio = get_gpio_from_pin(pin);
  return gpio_pin_get_dt(gpio);
}

// TODO Remove write
int sx1278_transceive(const struct device *dev, uint8_t *out, bool write,
                      uint8_t *in, size_t length) {
  int ret;

  const struct spi_buf tx_buf[] = {{
      .buf = out,
      .len = length,
  }};
  const struct spi_buf rx_buf[] = {{
      .buf = in,
      .len = length,
  }};
  const struct spi_buf_set tx = {
      .buffers = tx_buf,
      .count = ARRAY_SIZE(tx_buf),
  };
  const struct spi_buf_set rx = {
      .buffers = rx_buf,
      .count = ARRAY_SIZE(rx_buf),
  };

  ret = spi_transceive_dt(&dev_config.bus, &tx, &rx);

  if (ret < 0) {
    LOG_ERR("SPI transaction failed: %i", ret);
  }

  return ret;
}

static int sx1278_read(uint8_t out, uint8_t *in, uint8_t len) {
  return sx1278_transceive(NULL, &out, false, in, len);
}

static int sx1278_radio_init(const struct device *dev) {
  LOG_INF("sx1278_radio_init");
  int ret;
  uint8_t regval;

  if (!spi_is_ready_dt(&dev_config.bus)) {
    LOG_ERR("SPI device not ready");
    return -ENODEV;
  }

  gpio_pin_set_dt(&dev_config.reset, 0);
  k_sleep(K_MSEC(5));
  gpio_pin_set_dt(&dev_config.reset, 1);
  k_sleep(K_MSEC(5));

  // ret = sx1278_read(Z_RADIOLIB_SX127X_REG_VERSION, &regval, 1);
  // if (ret < 0) {
  //   LOG_ERR("Unable to read version info");
  //   return -EIO;
  // } else {
  //   LOG_INF("Radio Module Version: %d", regval);
  // }
  // k_sleep(K_MSEC(500));

  return 0;
}

static const struct radio_device_api sx1278_radio_api = {
    .get_config = sx1278_get_config,
    .configure_pin = sx1278_configure_pin,
    .transceive = sx1278_transceive,
    .gpio_read = sx1278_digital_read,
    .gpio_write = sx1278_digital_write,
};

DEVICE_DT_INST_DEFINE(0, &sx1278_radio_init, NULL, NULL, NULL, POST_KERNEL,
                      SX1280_INIT_PRIORITY, &sx1278_radio_api);