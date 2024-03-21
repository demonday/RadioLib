#ifndef ZMODULE_H
#define ZMODULE_H

#include "../../src/Module.h"
#include "ZephyrHal.h"

struct ModulePins {
  uint32_t cs;
  uint32_t irq;
  uint32_t rst;
  uint32_t gpio;
};

class ZephyrModule : public Module {
 public:
  // Constructor that takes only a Hal argument
  ZephyrModule(ZephyrHal* hal);

 private:
  static ModulePins getPins(ZephyrHal* hal);
};

#endif  // ZMODULE_H