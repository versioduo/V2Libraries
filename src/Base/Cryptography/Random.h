#pragma once
#include <Arduino.h>

namespace V2Base::Cryptography {
  class Random {
  public:
    static uint32_t read();
  };
};
