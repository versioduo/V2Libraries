#pragma once
#include "V2Colour.h"
#include <SPI.h>
#include <V2Base.h>

namespace V2LED {
  // Simple digital port driver driven by a timer.
  class Basic {
  public:
    Basic() = delete;
    constexpr Basic(uint8_t pin, V2Base::Timer::Periodic* timer) : _pin(pin), _timer(timer) {}
    auto tick() -> void;
    auto brightness(float fraction) -> void;
    auto flash(float seconds, float brightness = 1) -> void;
    auto loop() -> void;
    auto reset() -> void;

  private:
    V2Base::GPIO             _pin;
    V2Base::Timer::Periodic* _timer;

    struct {
      uint32_t startUsec{};
      uint32_t durationUsec{};
    } _flash{};
  };

  // Daisy-chained intelligent RGB LEDs.
  class WS2812 {
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
    constexpr WS2812(uint16_t capacity, SPIClass* spi) : _capacity(capacity), _spi{spi} {}

    // Build SPI bus from SERCOM.
    constexpr WS2812(uint16_t capacity, uint8_t pin, SERCOM* sercom, SercomSpiTXPad padTX, EPioType pinFunc) :
      _capacity(capacity),
      _sercom{.pin{pin}, .sercom{sercom}, .padTX{padTX}, .pinFunc{pinFunc}} {}

    auto begin() -> void;
    auto reset() -> void;

    // Encodes the DMA bit stream and fires a DMA transaction. If there
    // is a pending update and no current DMA transfer active, a new
    // transaction is started immediately.
    auto loop() -> void;

    // The logical number of LEDs to drive; it might differ from the number of connected
    // LEDs. The number becomes important when the direction is reversed and the last LED
    // becomes index number zero.
    auto size() const -> uint16_t {
      return _leds.size;
    }

    auto resize(uint16_t count) {
      reset();
      _leds.size = count;
    }

    auto reverse(bool reverse) {
      _leds.reverse = reverse;
    }

    auto brightnessMax(float fraction) -> void;

    auto hsv(HSV c, uint16_t first, uint16_t count = 1) -> void;
    auto hsv(HSV c) -> void {
      hsv(c, 0, size());
    }

    auto brightness(float v, uint16_t first, uint16_t count = 1) -> void {
      hsv({0, 0, v}, first, count);
    }

    auto brightness(float v) -> void {
      hsv({0, 0, v});
    }

    auto rgb(RGB c, uint16_t first, uint16_t count = 1) -> void;

    // Overlay a flash for the specified duration.
    auto flash(HSV c, float seconds, uint16_t first, uint16_t count = 1) -> void;
    auto flash(HSV c, float seconds) -> void {
      flash(c, seconds, 0, size());
    }

    // Draw a rainbow, cycles specifies how many cycles through the colors are
    // visible at the same time across all LEDs, seconds is duration for one LED
    // to rotate through one cycle of the colors.
    auto rainbow(uint8_t cycles, float seconds = 1, float brightness = 1, bool reverse = false) -> void;
    auto rainbow() const -> bool {
      return _rainbow.cycleSteps > 0;
    }

  private:
    const uint16_t _capacity{};

    struct {
      uint16_t size{};
      bool     reverse{};
      float    maxBrightness{1};
    } _leds;

    struct {
      const uint8_t        pin{};
      SERCOM*              sercom{};
      const SercomSpiTXPad padTX{};
      const EPioType       pinFunc{};
    } _sercom;

    SPIClass* _spi{};

    struct {
      uint8_t* buffer{};
      uint16_t bufferSize{};
      bool     update{};
    } _dma;

    struct PixelRGB {
      uint8_t r{};
      uint8_t g{};
      uint8_t b{};
    }* _pixelRGB{};

    struct PixelDMA {
      uint8_t g[3]{};
      uint8_t r[3]{};
      uint8_t b[3]{};
    }* _pixelDMA{};

    struct {
      PixelRGB pixel;
      uint16_t first;
      uint16_t count;
      uint32_t startUsec{};
      uint32_t durationUsec{};
    } _flash;

    struct {
      uint8_t  cycleSteps{};
      uint8_t  moveSteps{};
      float    brightness{};
      bool     reverse{};
      uint16_t color{};
      uint32_t updateUsec{};
      uint32_t lastUsec{};
    } _rainbow;

    auto update(uint16_t index, HSV c) -> void;
    auto encodePixel(const struct PixelRGB& rgb, struct PixelDMA& dma) -> void;
  };
};
