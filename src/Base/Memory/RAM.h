#pragma once
#include <Arduino.h>

namespace V2Base::Memory {
  class RAM {
  public:
    static constexpr uint32_t getSize() {
      return HSRAM_SIZE;
    }
    static uint32_t getFree();
  };
};
