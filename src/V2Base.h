#pragma once
#include "Base/Analog/ADC.h"
#include "Base/Cryptography/Random.h"
#include "Base/Cryptography/SHA1.h"
#include "Base/GPIO/GPIO.h"
#if defined(I2S)
#include "Base/I2S/I2S.h"
#endif
#include "Base/Memory/EEPROM.h"
#include "Base/Memory/Firmware.h"
#include "Base/Memory/Flash.h"
#include "Base/Memory/RAM.h"
#include "Base/Power/Power.h"
#include "Base/Text/Base64.h"
#include "Base/Timer/PWM.h"
#include "Base/Timer/Periodic.h"
#include "Base/USB/Device.h"

namespace V2Base {
  template <class T, size_t N> constexpr size_t countof(T (&)[N]) {
    return N;
  }

  static inline uint32_t getUsec() {
    return micros();
  }

  static inline uint32_t getUsecSince(uint32_t since) {
    return (uint32_t)(micros() - since);
  }
};
