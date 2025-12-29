#pragma once

namespace V23D {
  class Vector3 {
  public:
    float x{};
    float y{};
    float z{};

    constexpr Vector3(float vx = 0, float vy = 0, float vz = 0) : x(vx), y(vy), z(vz) {}

    auto normalize() -> Vector3 {
      if (auto l{length()}; l > 0.0001f) {
        x *= 1.f / l;
        y *= 1.f / l;
        z *= 1.f / l;
      }

      return Vector3(x, y, z);
    }

    auto length() const -> float {
      return std::sqrtf(x * x + y * y + z * z);
    }

    auto cross(const Vector3 v) const -> Vector3 {
      return Vector3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
    }

    auto dot(const Vector3 v) const -> float {
      return x * v.x + y * v.y + z * v.z;
    }

    auto angleBetween(const Vector3 v) const -> float {
      return std::acosf(dot(v));
    }
  };
};
