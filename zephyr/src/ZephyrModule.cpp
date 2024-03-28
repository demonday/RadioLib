#include "ZephyrModule.h"

#include "../../src/Hal.h"
#include "ZephyrHal.h"

/**
 * Wrapper for the RadioLib Module class, that hides the pins needed in the
 * Module constructor, and instead derives them from the radio device from
 * the device tree.
 *
 * Note: With this Zephyr based implementation, CS is taken care of by SPI
 * directly and so is set to RADIOLIB_NC
 */
ZephyrModule::ZephyrModule(ZephyrHal* hal)
    : Module(hal, RADIOLIB_NC, getPins(hal).irq, getPins(hal).rst,
             getPins(hal).gpio) {}

/**
 * Get the pins from the Hal and match them to those needed by the Module
 */
ModulePins ZephyrModule::getPins(ZephyrHal* hal) {
  pins pins_hal = hal->getPins();

  ModulePins pins_module;
  pins_module.cs = RADIOLIB_NC;  // TODO deal with RADIOLIB NC
  pins_module.rst = pins_hal.reset;
  pins_module.irq = pins_hal.dio[0];
  pins_module.gpio = pins_hal.dio[1];
  return pins_module;
}
