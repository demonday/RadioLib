#ifndef _RADIO_H_
#define _RADIO_H_

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

struct radio_device_config {
  struct spi_dt_spec bus;
  struct gpio_dt_spec reset;
  struct gpio_dt_spec cs;
  struct gpio_dt_spec busy;
  uint8_t dio_count;
  struct gpio_dt_spec dios[];
};

enum DIO {
  DIO_INPUT = 0,
  DIO_OUTPUT = 1 << 1,
  DIO_PIN_LOW = 1 << 2,
  DIO_PIN_HIGH = 1 << 3,
  DIO_TRIGGER_RISING_EDGE = 1 << 4,
  DIO_TRIGGER_FALLING_EDGE = 1 << 5
};

/**
 * @cond INTERNAL_HIDDEN
 *
 * For radio driver implementation. Radio clients should only use the public
 * apis.
 */

typedef int (*radio_dev_init)(const struct device *dev);

typedef int (*radio_dev_config)(const struct device *dev,
                                const struct radio_device_config **config);

typedef int (*radio_dev_configure_pin)(const struct device *dev, uint32_t pin,
                                       uint32_t mode);

typedef int (*radio_dev_spi_transceive)(const struct device *dev, uint8_t *out,
                                        bool write, uint8_t *in, size_t length);

typedef int (*radio_dev_gpio_read)(const struct device *dev, uint32_t pin);
typedef int (*radio_dev_gpio_write)(const struct device *dev, uint32_t pin,
                                    uint32_t value);

struct radio_device_api {
  radio_dev_init init;
  radio_dev_config get_config;
  radio_dev_configure_pin configure_pin;
  radio_dev_spi_transceive transceive;
  radio_dev_gpio_read gpio_read;
  radio_dev_gpio_write gpio_write;
};

/** @endcond */

static inline int radio_init(const struct device *dev) {
  const struct radio_device_api *api =
      (const struct radio_device_api *)dev->api;

  if (api->init == NULL) {
    return -ENOSYS;
  }

  return api->init(dev);
}

static inline int radio_get_config(const struct device *dev,
                                   const struct radio_device_config **config) {
  const struct radio_device_api *api =
      (const struct radio_device_api *)dev->api;

  if (api->get_config == NULL) {
    return -ENOSYS;
  }
  return api->get_config(dev, config);
}

static inline int radio_configure_pin(const struct device *dev, uint32_t pin,
                                      uint32_t mode) {
  const struct radio_device_api *api =
      (const struct radio_device_api *)dev->api;

  if (api->configure_pin == NULL) {
    return -ENOSYS;
  }
  return api->configure_pin(dev, pin, mode);
}

static inline int radio_transceive(const struct device *dev, uint8_t *out,
                                   bool write, uint8_t *in, size_t length) {
  const struct radio_device_api *api =
      (const struct radio_device_api *)dev->api;

  if (api->transceive == NULL) {
    return -ENOSYS;
  }
  return api->transceive(dev, out, write, in, length);
}

static inline int radio_gpio_read(const struct device *dev, uint32_t pin) {
  const struct radio_device_api *api =
      (const struct radio_device_api *)dev->api;

  if (api->gpio_read == NULL) {
    return -ENOSYS;
  }
  return api->gpio_read(dev, pin);
}

static inline int radio_gpio_write(const struct device *dev, uint32_t pin,
                                   uint32_t value) {
  const struct radio_device_api *api =
      (const struct radio_device_api *)dev->api;

  if (api->gpio_write == NULL) {
    return -ENOSYS;
  }
  return api->gpio_write(dev, pin, value);
}

#endif
