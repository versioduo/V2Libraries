#pragma once
#include <cmath>
#include <cstdint>

class V2Colour {
public:
  enum Hue {
    Red     = 0,
    Orange  = 20,
    Yellow  = 60,
    Green   = 120,
    Cyan    = 180,
    Blue    = 240,
    Magenta = 300,
  };

  // CIE 1931 - psychometric lightness / human color vision
  //   L* = 903.3 ∙ Y,            if Y ≤ 0.008856
  //   L* = 116   ∙ Y^1/3 – 16,   if Y > 0.008856
  static constexpr float toCIE1931(float v) {
    const float brightness = v * 100.f;
    if (brightness < 8.f)
      return brightness / 903.3f;

    return powf((brightness + 16.f) / 116.f, 3);
  }

  // Hue, Saturation, Value
  static constexpr void HSVtoRGB(float h, float s, float v, uint8_t& r, uint8_t& g, uint8_t& b) {
    if (h < 0.f || h >= 360.f)
      h = 0;

    h /= 60.f;
    const uint8_t i = h;
    const float   f = h - i;
    const float   p = v * (1.f - s);
    const float   q = v * (1.f - f * s);
    const float   t = v * (1.f - (1.f - f) * s);

    switch (i) {
      case 0:
        r = ceilf(v * 255.f);
        g = ceilf(t * 255.f);
        b = ceilf(p * 255.f);
        return;

      case 1:
        r = ceilf(q * 255.f);
        g = ceilf(v * 255.f);
        b = ceilf(p * 255.f);
        return;

      case 2:
        r = ceilf(p * 255.f);
        g = ceilf(v * 255.f);
        b = ceilf(t * 255.f);
        return;

      case 3:
        r = ceilf(p * 255.f);
        g = ceilf(q * 255.f);
        b = ceilf(v * 255.f);
        return;

      case 4:
        r = ceilf(t * 255.f);
        g = ceilf(p * 255.f);
        b = ceilf(v * 255.f);
        return;

      case 5:
        r = ceilf(v * 255.f);
        g = ceilf(p * 255.f);
        b = ceilf(q * 255.f);
        return;
    }
  }
};
