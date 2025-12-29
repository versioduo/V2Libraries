#pragma once
#include "3D/Attitude.h"
#include "3D/Euler.h"
#include "3D/Quaternion.h"
#include "3D/Vector3.h"
#include <numbers>

namespace V23D {
  constexpr auto radToDeg(float rad) -> float {
    return rad * 180.f / std::numbers::pi_v<float>;
  }

  constexpr auto degToRad(float deg) -> float {
    return deg * std::numbers::pi_v<float> / 180.f;
  }
};
