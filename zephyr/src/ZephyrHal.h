#ifndef ZEPHYR_HAL_H
#define ZEPHYR_HAL_H

#include <zephyr/device.h>

#include "../../src/RadioLib.h"

#define DIO_SIZE 5
struct pins {
  uint8_t reset;
  uint8_t cs;
  uint8_t dio_size;
  uint8_t dio[DIO_SIZE];
};

class ZephyrHal : public RadioLibHal {
 public:
  ZephyrHal(const struct device* device);

  // Implementations of pure virtual methods from RadioLibHal
  virtual void pinMode(uint32_t pin, uint32_t mode) override;
  virtual void digitalWrite(uint32_t pin, uint32_t value) override;
  virtual uint32_t digitalRead(uint32_t pin) override;
  virtual void attachInterrupt(uint32_t interruptNum, void (*interruptCb)(void),
                               uint32_t mode) override;
  virtual void detachInterrupt(uint32_t interruptNum) override;
  virtual void delay(unsigned long ms) override;
  virtual void delayMicroseconds(unsigned long us) override;
  virtual unsigned long millis() override;
  virtual unsigned long micros() override;
  virtual long pulseIn(uint32_t pin, uint32_t state,
                       unsigned long timeout) override;
  virtual void spiBegin() override;
  virtual void spiBeginTransaction() override;
  virtual void spiTransfer(uint8_t* out, size_t len, uint8_t* in) override;
  virtual void spiEndTransaction() override;
  virtual void spiEnd() override;

  // Other virtual methods
  virtual void init() override;
  virtual void term() override;
  virtual void tone(uint32_t pin, unsigned int frequency,
                    unsigned long duration = 0) override;
  virtual void noTone(uint32_t pin) override;
  virtual void yield() override;
  virtual uint32_t pinToInterrupt(uint32_t pin) override;

  virtual void readPersistentStorage(uint32_t addr, uint8_t* buff, size_t len);
  virtual void writePersistentStorage(uint32_t addr, uint8_t* buff, size_t len);
  virtual void wipePersistentStorage();
  virtual uint32_t getPersistentAddr(uint32_t id);
  struct pins getPins();

 private:
  const struct device* _radio;  // Zephyr device structure
};

#endif  // ZEPHYR_HAL_H