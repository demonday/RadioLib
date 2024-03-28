#include "ZephyrHal.h"

#include <errno.h>
#include <stdbool.h>
#include <stdlib.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "../include/zephyr/drivers/radio.h"

LOG_MODULE_REGISTER(zradiolib, 4);

static struct gpio_callback gpio_cb_data;
static void (*interruptCb)(void);

ZephyrHal::ZephyrHal(const struct device* radio_dev)
    : RadioLibHal(DIO_INPUT, DIO_OUTPUT, DIO_PIN_LOW, DIO_PIN_HIGH,
                  DIO_TRIGGER_RISING_EDGE, DIO_TRIGGER_FALLING_EDGE),
      _radio(radio_dev) {}

void ZephyrHal::pinMode(uint32_t pin, uint32_t mode) {
  LOG_DBG("pinMode called with pin: %u, mode: %u", pin, mode);
  if (pin == RADIOLIB_NC) {
    return;
  }
  __ASSERT(mode <= DIO_OUTPUT, "Invalid Mode %d", mode);
  radio_configure_pin(_radio, pin, mode);
}

void ZephyrHal::digitalWrite(uint32_t pin, uint32_t value) {
  // LOG_DBG("digitalWrite called with pin: %u, value: %u", pin, value);
  if (pin == RADIOLIB_NC) {
    // LOG_DBG("digitalWrite for pin %d not connected.", pin);
    return;
  }
  radio_gpio_write(_radio, pin, value);
}

uint32_t ZephyrHal::digitalRead(uint32_t pin) {
  // LOG_DBG("digitalRead called with pin: %u", pin);
  return radio_gpio_read(_radio, pin);
}

static void gpio_callback_function(const struct device* dev,
                                   struct gpio_callback* cb, uint32_t pins) {
  LOG_DBG("Interrupt triggered on GPIO46\n");
  if (interruptCb != NULL) {
    interruptCb();
  }
}

// TODO
void ZephyrHal::attachInterrupt(uint32_t interruptNum,
                                void (*interruptCbFn)(void), uint32_t mode) {
  LOG_DBG("attachInterrupt called with interruptNum: %u, mode: %u",
          interruptNum, mode);

  int ret;

  const struct radio_device_config* config = NULL;
  radio_get_config(_radio, &config);

  // Configure the IRQ (DIO[0]) pin as an input
  ret = gpio_pin_configure_dt(&config->dios[0], GPIO_INPUT);
  if (ret != 0) {
    LOG_DBG("Failed to configure GPIO pin\n");
    return;
  }

  // Store the callback reference
  interruptCb = interruptCbFn;

  // Initialize the callback
  gpio_init_callback(&gpio_cb_data, gpio_callback_function,
                     BIT(config->dios[0].pin));
  gpio_add_callback(config->dios[0].port, &gpio_cb_data);
  if (ret != 0) {
    LOG_DBG("Failed to add callback\n");
    return;
  }
  // Configure the interrupt
  ret = gpio_pin_interrupt_configure_dt(&config->dios[0], GPIO_INT_EDGE_RISING);
  if (ret != 0) {
    LOG_DBG("Failed to configure GPIO interrupt\n");
    return;
  }
  LOG_DBG("GPIO interrupt configured successfully\n");
}

// TODO
uint32_t ZephyrHal::pinToInterrupt(uint32_t pin) {
  LOG_DBG("pinToInterrupt called with pin: %u", pin);
  // Convert pin number to interrupt number in Zephyr
  return 1;
}

// TODO
void ZephyrHal::detachInterrupt(uint32_t interruptNum) {
  LOG_DBG("detachInterrupt called with interruptNum: %u", interruptNum);
  // Disable the GPIO pin interrupt
  const struct radio_device_config* config = NULL;
  radio_get_config(_radio, &config);

  int ret;
  ret = gpio_pin_interrupt_configure_dt(&config->dios[0], GPIO_INT_DISABLE);
  if (ret != 0) {
    LOG_DBG("Failed to disable GPIO interrupt\n");
    return;
  }

  // Remove the callback
  gpio_remove_callback(config->dios[0].port, &gpio_cb_data);
  interruptCb = NULL;
  LOG_DBG("GPIO interrupt and callback removed successfully\n");
}

void ZephyrHal::delay(unsigned long ms) {
  // LOG_DBG("delay called with ms: %lu", ms);
  k_msleep(ms);
}

void ZephyrHal::delayMicroseconds(unsigned long us) {
  // LOG_DBG("delayMicroseconds called with us: %lu", us);
  k_usleep(us);
}

unsigned long ZephyrHal::millis() {
  // LOG_DBG("millis called");
  return k_uptime_get();  // Example return value
}

unsigned long ZephyrHal::micros() {
  // LOG_DBG("micros called");
  return k_ticks_to_us_floor64(k_uptime_ticks());
}

long ZephyrHal::pulseIn(uint32_t pin, uint32_t state, unsigned long timeout) {
  LOG_DBG("pulseIn called with pin: %u, state: %u, timeout: %lu", pin, state,
          timeout);
  __ASSERT(false, "pulseIn not implemented");

  return 0;  // Example return value
}

void ZephyrHal::spiBegin() {
  // LOG_DBG("spiBegin called");
  // Not Needed in Zephyr Implementation
}

void ZephyrHal::spiBeginTransaction() {
  // LOG_DBG("spiBeginTransaction called");
  // Not Needed in Zephyr Implementation
}

// void ArduinoHal::spiTransfer(uint8_t* out, size_t len, uint8_t* in) {
//   for (size_t i = 0; i < len; i++) {
//     serialPrintf("RLB_SPI: BF: i:%d, %02x, %02x\n", i, in[i], out[i]);
//     in[i] = spi->transfer(out[i]);
//     serialPrintf("RLB_SPI: AF: i:%d, %02x, %02x\n", i, in[i], out[i]);
//   }
// }

void ZephyrHal::spiTransfer(uint8_t* out, size_t len, uint8_t* in) {
  // LOG_DBG("spiTransfer called");
  if ((out[0] & 0b10000000) == 0b10000000) {
    radio_transceive(_radio, out, true, in, len);
  } else {
    radio_transceive(_radio, out, false, in, len);
  }
}

void ZephyrHal::spiEndTransaction() {
  // LOG_DBG("spiEndTransaction called");
  // Not Needed in Zephyr Implementation
}

void ZephyrHal::spiEnd() {
  // LOG_DBG("spiEnd called");
  // Not Needed in Zephyr Implementation
}

// Implementations of virtual methods that may or may not be needed

void ZephyrHal::init() {
  LOG_DBG("init called");
  // Initialization code, if needed
}

void ZephyrHal::term() {
  // LOG_DBG("term called");
  // Termination code, if needed
}

void ZephyrHal::tone(uint32_t pin, unsigned int frequency,
                     unsigned long duration) {
  LOG_DBG("tone called with pin: %u, frequency: %u, duration: %lu", pin,
          frequency, duration);
  __ASSERT(false, "tone not implemented");
}

void ZephyrHal::noTone(uint32_t pin) {
  LOG_DBG("noTone called with pin: %u", pin);
  // Zephyr-specific tone stopping
  __ASSERT(false, "pulseIn not implemented");
}

void ZephyrHal::yield() {
  // LOG_DBG("yield called");
  //  Zephyr-specific yield
  k_yield();
}

void ZephyrHal::readPersistentStorage(uint32_t addr, uint8_t* buff,
                                      size_t len) {
  LOG_DBG("readPersistentStorage called with addr: %u, len: %zu", addr, len);
  // Zephyr-specific persistent storage read
  __ASSERT(false, "readPersistentStorage not implemented");
}

void ZephyrHal::writePersistentStorage(uint32_t addr, uint8_t* buff,
                                       size_t len) {
  LOG_DBG("writePersistentStorage called with addr: %u, len: %zu", addr, len);
  // Zephyr-specific persistent storage write
  __ASSERT(false, "writePersistentStorage not implemented");
}

void ZephyrHal::wipePersistentStorage() {
  LOG_DBG("wipePersistentStorage called");
  // Zephyr-specific storage wipe
  __ASSERT(false, "wipePersistentStorage not implemented");
}

uint32_t ZephyrHal::getPersistentAddr(uint32_t id) {
  LOG_DBG("getPersistentAddr called with id: %u", id);
  // Conversion from persistent parameter ID to physical address
  return -ENOSYS;
}

struct pins ZephyrHal::getPins() {
  const struct radio_device_config* config = NULL;
  radio_get_config(_radio, &config);

  struct pins pins_hal = {.reset = config->reset.pin,
                          .cs = config->cs.pin,
                          .dio_size = config->dio_count};

  for (int i = 0; i < config->dio_count; i++) {
    pins_hal.dio[i] = config->dios[i].pin;
  }
  return pins_hal;
}