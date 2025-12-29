#pragma once
#include <Arduino.h>

namespace V2Base::Text {
  class Base64 {
  public:
    static uint32_t encode(const uint8_t input[], uint32_t length, uint8_t output[]);
    static uint32_t decode(const uint8_t input[], uint8_t output[]);
  };
};
