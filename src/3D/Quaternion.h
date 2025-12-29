#pragma once
#include "Vector3.h"

namespace V23D {
  class Quaternion {
  public:
    float w{1};
    float x{};
    float y{};
    float z{};

    constexpr Quaternion() = default;
    constexpr Quaternion(float w, float x, float y, float z) : w(w), x(x), y(y), z(z) {}

    auto operator*(const Quaternion& q) const -> Quaternion {
      return Quaternion(w * q.w - x * q.x - y * q.y - z * q.z,
                        w * q.x + x * q.w + y * q.z - z * q.y,
                        w * q.y - x * q.z + y * q.w + z * q.x,
                        w * q.z + x * q.y - y * q.x + z * q.w);
    }

    auto normalize() -> Quaternion {
      if (auto l{length()}; l > 0.0001f) {
        w *= 1.f / l;
        x *= 1.f / l;
        y *= 1.f / l;
        z *= 1.f / l;
      }

      return Quaternion(w, x, y, z);
    }

    auto length() const -> float {
      return std::sqrtf(w * w + x * x + y * y + z * z);
    }

    auto conjugate() const -> Quaternion {
      return Quaternion(w, -x, -y, -z);
    }

    auto equal(const Quaternion q) const -> bool {
      if (std::fabs(w - q.w) > 0.0001f)
        return false;

      if (std::fabs(x - q.x) > 0.0001f)
        return false;

      if (std::fabs(y - q.y) > 0.0001f)
        return false;

      if (std::fabs(z - q.z) > 0.0001f)
        return false;

      return true;
    }
  };
};
