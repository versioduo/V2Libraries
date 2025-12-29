#pragma once
#include "V2Colour.h"
#include <SPI.h>
#include <V2Base.h>

namespace V2LED {
  // Simple digital port driver driven by a timer.
  class Basic {
  public:
    constexpr Basic(uint8_t pin, V2Base::Timer::Periodic* timer) : _pin(pin), _timer(timer) {}
    auto tick() -> void;
    auto setBrightness(float fraction) -> void;
    auto flash(float seconds, float brightness = 1) -> void;
    auto loop() -> void;
    auto reset() -> void;

  private:
    V2Base::GPIO             _pin;
    V2Base::Timer::Periodic* _timer;

    struct {
      unsigned long startUsec{};
      unsigned long durationUsec{};
    } _flash{};
  };

  // Daisy-chained intelligent RGB-LEDs.
  class WS2812 {
  public:
    constexpr WS2812(uint16_t nLEDs, SPIClass* spi) : _nLEDsMax(nLEDs), _spi{spi} {}

    // Build SPI bus from SERCOM.
    constexpr WS2812(uint16_t nLEDs, uint8_t pin, SERCOM* sercom, SercomSpiTXPad padTX, EPioType pinFunc) :
      _nLEDsMax(nLEDs),
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
    auto getNumLEDs() -> uint16_t const {
      return _leds.count;
    }

    auto setNumLEDs(uint16_t count) {
      reset();
      _leds.count = count;
    }

    auto setDirection(bool reverse) {
      _leds.reverse = reverse;
    }

    // The fraction of the brightness to apply. The value is applied with
    // the next call to loop().
    auto setMaxBrightness(float fraction) -> void;

    // Set white color brightness for one or all LEDs.
    auto setBrightness(uint16_t index, float v) {
      if (isRainbow())
        return;

      setLED(index, 0, 0, v);
    }

    auto setBrightness(float v) {
      for (uint16_t i = 0; i < _leds.count; i++)
        setBrightness(i, v);
    }

    // Set HSV color for one or all LEDs.
    auto setHSV(uint16_t index, float h, float s, float v) {
      if (isRainbow())
        return;

      setLED(index, h, s, v);
    }

    auto setHSV(float h, float s, float v) {
      for (uint16_t i = 0; i < _leds.count; i++)
        setHSV(i, h, s, v);
    }

    auto setRGB(uint16_t index, uint8_t r, uint8_t g, uint8_t b) -> void;
    auto setRGB(uint8_t r, uint8_t g, uint8_t b) -> void {
      for (uint16_t i = 0; i < _leds.count; i++)
        setRGB(i, r, g, b);
    }

    // Overlay a timed splash. Sets the color of n LEDs; loop() restores the
    // buffered state after the specified duration.
    auto splashHSV(float seconds, uint16_t start, uint16_t count, float h, float s, float v) -> void;
    auto splashHSV(float seconds, float h, float s, float v) -> void {
      splashHSV(seconds, 0, _leds.count, h, s, v);
    }

    // Draw a rainbow, cycles specifies how many cycles through the colors are
    // visible at the same time across all LEDs, seconds is duration for one LED
    // to rotate through one cycle of the colors.
    auto rainbow(uint8_t cycles = 1, float seconds = 1, float brightness = 1, bool reverse = false) -> void;
    auto isRainbow() -> bool {
      return _rainbow.cycleSteps > 0;
    }

  private:
    const uint16_t _nLEDsMax{};
    struct {
      uint16_t count{};
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
      PixelRGB      pixel;
      uint16_t      start{};
      uint16_t      count{};
      unsigned long startUsec{};
      unsigned long durationUsec{};
    } _splash;

    struct {
      uint8_t       cycleSteps{};
      uint8_t       moveSteps{};
      float         brightness{};
      bool          reverse{};
      uint16_t      color{};
      unsigned long updateUsec{};
      unsigned long lastUsec{};
    } _rainbow{};

    auto setLED(uint16_t index, float h, float s, float v) -> void;
    auto encodePixel(const struct PixelRGB* rgb, struct PixelDMA* dma) -> void;
  };
};
