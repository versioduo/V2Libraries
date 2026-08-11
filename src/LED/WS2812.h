#pragma once
#include "V2Colour.h"
#include <SPI.h>
#include <V2Base.h>
#include <ranges>

namespace V2LED {
  template <uint16_t capacity> class WS2812 {
  public:
    struct HSV {
      float h{};
      float s{};
      float v{};
    };

    struct RGB {
      uint8_t r{};
      uint8_t g{};
      uint8_t b{};
    };

    WS2812() = delete;
    constexpr WS2812(uint8_t pin, SERCOM& sercom, SercomSpiTXPad padTX, EPioType pinFunc) :
      _sercom{.pin{pin}, .sercom{sercom}, .padTX{padTX}, .pinFunc{pinFunc}},
      _spi{&_sercom.sercom, _sercom.pin, _sercom.pin, _sercom.pin, _sercom.padTX, SERCOM_RX_PAD_3} {}

    auto begin() -> void;
    auto reset() -> void;

    // Encodes the DMA bit stream and fires a DMA transaction. If there
    // is a pending update and no current DMA transfer active, a new
    // transaction is started immediately.
    auto loop() -> void;

    // LEDs. The number becomes important when the direction is reversed and the last LED
    // becomes index number zero.
    auto size() const -> uint16_t;
    auto resize(uint16_t count);

    auto reverse(bool reverse);
    auto brightnessMax(float fraction) -> void;
    auto hsv(HSV c, uint16_t first, uint16_t count = 1) -> void;
    auto hsv(HSV c) -> void;
    auto brightness(float v, uint16_t first, uint16_t count = 1) -> void;
    auto brightness(float v) -> void;
    auto rgb(RGB c, uint16_t first, uint16_t count = 1) -> void;

    // Overlay a flash for the specified duration.
    auto flash(HSV c, float seconds, uint16_t first, uint16_t count = 1) -> void;
    auto flash(HSV c, float seconds) -> void;

    // Draw a rainbow, cycles specifies how many cycles through the colors are
    // visible at the same time across all LEDs, seconds is duration for one LED
    // to rotate through one cycle of the colors.
    auto rainbow(uint8_t cycles, float seconds = 1, float brightness = 1, bool reverse = false) -> void;
    auto rainbow() const -> bool;

  private:
    float    _maxBrightness{1};
    uint16_t _size{capacity};
    bool     _reverse{};

    struct {
      const uint8_t        pin{};
      SERCOM&              sercom;
      const SercomSpiTXPad padTX{};
      const EPioType       pinFunc{};
    } _sercom;

    SPIClass _spi;

    struct Pixel {
      struct {
        uint32_t usec{};
        uint32_t durationUsec{};
        RGB      colour;
      } flash;
      RGB colour;

      bool update{};
    };
    static_assert(sizeof(Pixel) == 16);

    std::array<Pixel, capacity> _pixels{};

    struct DMA {
      struct Pixel {
        uint8_t g[3]{};
        uint8_t r[3]{};
        uint8_t b[3]{};
      };
      static_assert(sizeof(Pixel) == 9);

      // 2.4 MBit SPI clock / 8 == 300 kByte/s == 3.33 usec / Byte.
      // - lead-in of ~300 usec to settle the signal at logic low
      // - pixel data
      // - ~300 usec latch
      struct Buffer : std::array<Pixel, 10 + capacity + 10> {
        auto bytes(size_t size) -> size_t {
          return (10 + size + 10) * sizeof(Pixel);
        }
      } buffer{};
      std::span<Pixel> pixels{buffer.begin() + 10, capacity};

      bool update{};
    } _dma;

    struct {
      uint8_t  cycleSteps{};
      uint8_t  moveSteps{};
      float    brightness{};
      bool     reverse{};
      uint16_t colour{};
      uint32_t usec{};
    } _rainbow;

    static auto convertWS2812(HSV hsv, RGB& rgb);
    static auto update(Pixel& p, DMA::Pixel& dp) -> bool;
  };
}

template <uint16_t capacity> auto V2LED::WS2812<capacity>::begin() -> void {
  _spi.begin();

  // SPIClass.begin() applies the board config to all given pins. We just passed
  // one and the same pin to all of them, to make sure we do not touch unrelated
  // ones.
  pinPeripheral(_sercom.pin, _sercom.pinFunc);

  // The transaction will never stop.
  _spi.beginTransaction(SPISettings(2400000, MSBFIRST, SPI_MODE0));
  reset();
}

template <uint16_t capacity> auto V2LED::WS2812<capacity>::reset() -> void {
  while (_spi.isBusy())
    yield();

  _rainbow    = {};
  _dma.update = false;
  std::fill(_pixels.begin(), _pixels.end(), {});

  for (auto& dp : _dma.pixels) {
    Pixel p{.update{true}};
    update(p, dp);
  }

  _spi.transfer(_dma.buffer.data(), nullptr, _dma.buffer.bytes(capacity), false);
}

template <uint16_t capacity> auto V2LED::WS2812<capacity>::update(Pixel& p, DMA::Pixel& dp) -> bool {
  auto encode{[](const struct RGB& rgb, DMA::Pixel& drgb) {
    auto frame{[](uint8_t b, uint8_t f[3]) {
      union {
        uint32_t bits;
        uint8_t  bytes[4];
      } data{.bits{0b100100100100100100100100}};

      // Encode 1 bit into 3 bits frame data, 0b100 == 0, 0b110 == 1
      if (b & 1)
        data.bits |= 1 << 1;

      if (b & 2)
        data.bits |= 1 << 4;

      if (b & 4)
        data.bits |= 1 << 7;

      if (b & 8)
        data.bits |= 1 << 10;

      if (b & 16)
        data.bits |= 1 << 13;

      if (b & 32)
        data.bits |= 1 << 16;

      if (b & 64)
        data.bits |= 1 << 19;

      if (b & 128)
        data.bits |= 1 << 22;

      f[0] = data.bytes[2];
      f[1] = data.bytes[1];
      f[2] = data.bytes[0];
    }};

    frame(rgb.r, drgb.r);
    frame(rgb.g, drgb.g);
    frame(rgb.b, drgb.b);
  }};

  if (!p.update && p.flash.usec == 0)
    return false;

  if (p.flash.usec > 0) {
    if (V2Base::getUsecSince(p.flash.usec) < p.flash.durationUsec) {
      if (p.update) {
        encode(p.flash.colour, dp);
        p.update = false;
      }

      return true;
    }

    p.flash.usec = 0;
  }

  encode(p.colour, dp);
  p.update = false;
  return false;
}

template <uint16_t capacity> auto V2LED::WS2812<capacity>::loop() -> void {
  if (_rainbow.cycleSteps > 0 && uint32_t(micros() - _rainbow.usec) > 25 * 1000) {
    _rainbow.usec = micros();

    for (auto c{int16_t(_rainbow.colour)}; auto& p : std::span{_pixels}.first(_size)) {
      convertWS2812({float(c), 1, _rainbow.brightness * _maxBrightness}, p.colour);
      p.update = true;

      if (_rainbow.reverse) {
        c += _rainbow.cycleSteps;
        if (c > 359)
          c -= 360;

      } else {
        c -= _rainbow.cycleSteps;
        if (c < 0)
          c += 360;
      }
    }

    _rainbow.colour += _rainbow.moveSteps;
    if (_rainbow.colour > 359)
      _rainbow.colour -= 360;

    _dma.update = true;
  }

  if (!_dma.update)
    return;

  if (_spi.isBusy())
    return;

  _dma.update = false;

  if (!_reverse) {
    for (auto&& [p, dp] : std::views::zip(std::span{_pixels}.first(_size), _dma.pixels))
      if (update(p, dp))
        _dma.update = true;

  } else {
    for (auto&& [p, dp] : std::views::zip(std::span{_pixels}.first(_size) | std::views::reverse, _dma.pixels))
      if (update(p, dp))
        _dma.update = true;
  }

  _spi.transfer(_dma.buffer.data(), nullptr, _dma.buffer.bytes(_size), false);
}

template <uint16_t capacity> auto V2LED::WS2812<capacity>::size() const -> uint16_t {
  return _size;
}

template <uint16_t capacity> auto V2LED::WS2812<capacity>::resize(uint16_t count) {
  reset();
  _size = count;
}

template <uint16_t capacity> auto V2LED::WS2812<capacity>::reverse(bool reverse) {
  reset();
  _reverse = reverse;
}

template <uint16_t capacity> auto V2LED::WS2812<capacity>::brightnessMax(float fraction) -> void {
  _maxBrightness = fraction;
  _dma.update    = true;
}

template <uint16_t capacity> auto V2LED::WS2812<capacity>::hsv(HSV c, uint16_t first, uint16_t count) -> void {
  if (rainbow())
    return;

  c.v *= _maxBrightness;

  for (auto& p : std::span{_pixels.begin() + first, count}) {
    convertWS2812(c, p.colour);
    p.update = true;
  }

  _dma.update = true;
}

template <uint16_t capacity> auto V2LED::WS2812<capacity>::hsv(HSV c) -> void {
  hsv(c, 0, _size);
}

template <uint16_t capacity> auto V2LED::WS2812<capacity>::brightness(float v, uint16_t first, uint16_t count) -> void {
  hsv({0, 0, v}, first, count);
}

template <uint16_t capacity> auto V2LED::WS2812<capacity>::brightness(float v) -> void {
  hsv({0, 0, v});
}

template <uint16_t capacity> auto V2LED::WS2812<capacity>::rgb(RGB c, uint16_t first, uint16_t count) -> void {
  if (rainbow())
    return;

  for (auto& p : std::span{_pixels.begin() + first, count}) {
    p.colour.r = float(c.r) * _maxBrightness;
    p.colour.g = float(c.g) * _maxBrightness;
    p.colour.b = float(c.b) * _maxBrightness;
    p.update   = true;
  }

  _dma.update = true;
}

template <uint16_t capacity> auto V2LED::WS2812<capacity>::flash(HSV c, float seconds, uint16_t first, uint16_t count) -> void {
  c.v *= _maxBrightness;

  for (auto& p : std::span{_pixels.begin() + first, count}) {
    p.flash.usec         = V2Base::getUsec();
    p.flash.durationUsec = seconds * 1000.f * 1000.f;
    convertWS2812(c, p.flash.colour);
    p.update = true;
  }

  _dma.update = true;
}

template <uint16_t capacity> auto V2LED::WS2812<capacity>::flash(HSV c, float seconds) -> void {
  flash(c, seconds, 0, _size);
}

template <uint16_t capacity> auto V2LED::WS2812<capacity>::rainbow(uint8_t cycles, float seconds, float brightness, bool reverse) -> void {
  if (cycles == 0) {
    _rainbow = {};
    return;
  }

  _rainbow.cycleSteps = (360 / _size) * cycles;
  _rainbow.moveSteps  = (360.f / 40.f) / seconds;
  _rainbow.brightness = brightness;
  _rainbow.reverse    = reverse;
}

template <uint16_t capacity> auto V2LED::WS2812<capacity>::rainbow() const -> bool {
  return _rainbow.cycleSteps > 0;
}

template <uint16_t capacity> auto V2LED::WS2812<capacity>::convertWS2812(HSV hsv, RGB& rgb) {
  if (hsv.v <= 0.f) {
    rgb = {};
    return;
  }

  if (hsv.s > 0.f)
    V2Colour::HSVtoRGB(hsv.h, hsv.s, V2Colour::toCIE1931(hsv.v), rgb.r, rgb.g, rgb.b);
  else
    rgb.r = rgb.g = rgb.b = ceilf(255.f * V2Colour::toCIE1931(hsv.v));
}
