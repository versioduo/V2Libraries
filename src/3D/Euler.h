#pragma once
#include "Quaternion.h"
#include <numbers>

namespace V23D {
  class Euler {
  public:
    float yaw{};
    float pitch{};
    float roll{};

    constexpr Euler(float y = 0, float p = 0, float r = 0) : yaw(y), pitch(p), roll(r) {}

    static constexpr auto quaternion(Quaternion q) -> Euler {
      Euler e;

      // X
      auto sinr_cosp{2.f * (q.w * q.x + q.y * q.z)};
      auto cosr_cosp{1.f - 2.f * (q.x * q.x + q.y * q.y)};
      e.roll = std::atan2(sinr_cosp, cosr_cosp);

      // Y
      auto sinp{std::sqrt(1.f + 2.f * (q.w * q.y - q.x * q.z))};
      auto cosp{std::sqrt(1.f - 2.f * (q.w * q.y - q.x * q.z))};
      e.pitch = 2.f * std::atan2(sinp, cosp) - std::numbers::pi_v<float> / 2.f;

      // Z
      auto siny_cosp{2.f * (q.w * q.z + q.x * q.y)};
      auto cosy_cosp{1.f - 2.f * (q.y * q.y + q.z * q.z)};
      e.yaw = std::atan2(siny_cosp, cosy_cosp);

      return e;
    }
  };
};
