#include "V2LED.h"
#include <wiring_private.h>

auto V2LED::WS2812::begin() -> void {
  // Lead-in of ~300 usec to settle the signal at logic low + pixel data + ~300
  // usec latch 2.4 MBit SPI clock / 8 == 300 kByte/s == 3.33 usec / Byte.
  _dma.bufferSize = 90 + (sizeof(struct PixelDMA) * _nLEDsMax) + 90;
  _dma.buffer     = (uint8_t*)calloc(_dma.bufferSize, 1);

  // Pointer to start of encoded pixel data.
  _pixelDMA = (struct PixelDMA*)(_dma.buffer + 90);

  // RGB buffer to draw DMA pixel data from.
  _pixelRGB = (struct PixelRGB*)calloc(sizeof(struct PixelRGB), _nLEDsMax);

  // Build SPI bus from SERCOM.
  //
  // SPIClass.begin() applies the board config to all given pins, which might not
  // match our configuration. Just pass the same pin to all of them, to make sure
  // we do not touch anything else. Our pin will be switched to the SERCOM after
  // begin().
  if (!_spi)
    _spi = new SPIClass(_sercom.sercom, _sercom.pin, _sercom.pin, _sercom.pin, _sercom.padTX, SERCOM_RX_PAD_3);

  if (_sercom.sercom)
    pinPeripheral(_sercom.pin, _sercom.pinFunc);

  // Configure SPI, the transaction will never stop.
  _spi->begin();
  _spi->beginTransaction(SPISettings(2400000, MSBFIRST, SPI_MODE0));

  _leds.count = _nLEDsMax;
  reset();
}

auto V2LED::WS2812::reset() -> void {
  while (_spi->isBusy())
    yield();

  _splash  = {};
  _rainbow = {};
  setBrightness(0);
}

auto V2LED::WS2812::loop() -> void {
  // Remove timed splash.
  if (_splash.startUsec > 0 && (unsigned long)(micros() - _splash.startUsec) > _splash.durationUsec) {
    _dma.update       = true;
    _splash.startUsec = 0;
  }

  // Draw rainbow.
  if (_rainbow.cycleSteps > 0 && (unsigned long)(micros() - _rainbow.lastUsec) > 25 * 1000) {
    _rainbow.lastUsec = micros();

    int16_t color = _rainbow.color;
    for (uint16_t i = 0; i < _leds.count; i++) {
      setLED(i, color, 1, _rainbow.brightness);

      if (_rainbow.reverse) {
        color += _rainbow.cycleSteps;
        if (color > 359)
          color -= 360;

      } else {
        color -= _rainbow.cycleSteps;
        if (color < 0)
          color += 360;
      }
    }

    _rainbow.color += _rainbow.moveSteps;
    if (_rainbow.color > 359)
      _rainbow.color -= 360;
  }

  if (!_dma.update)
    return;

  if (_spi->isBusy())
    return;

  // Draw splash overlay.
  if (_splash.startUsec > 0) {
    PixelRGB pixel;

    for (uint16_t i = 0; i < _leds.count; i++) {
      PixelDMA* pixelDMA = _leds.reverse ? &_pixelDMA[_leds.count - 1 - i] : &_pixelDMA[i];
      if (i >= _splash.start && i < (_splash.start + _splash.count))
        encodePixel(&_splash.pixel, pixelDMA);
      else
        encodePixel(&pixel, _pixelDMA);
    }
  } else {
    for (uint16_t i = 0; i < _leds.count; i++) {
      PixelDMA* pixelDMA = _leds.reverse ? &_pixelDMA[_leds.count - 1 - i] : &_pixelDMA[i];
      encodePixel(&_pixelRGB[i], pixelDMA);
    }
  }

  _spi->transfer(_dma.buffer, NULL, _dma.bufferSize, false);
  _dma.update = false;
}

static auto convertWS2812(float h, float s, float v, uint8_t* rp, uint8_t* gp, uint8_t* bp) {
  uint8_t r, g, b;

  if (v <= 0.f) {
    *rp = *gp = *bp = 0;
    return;
  }

  if (s > 0.f)
    V2Colour::HSVtoRGB(h, s, V2Colour::toCIE1931(v), r, g, b);

  else
    r = g = b = ceilf(255.f * V2Colour::toCIE1931(v));

  *rp = r;
  *gp = g;
  *bp = b;
}

auto V2LED::WS2812::setLED(uint16_t index, float h, float s, float v) -> void {
  convertWS2812(h, s, v * _leds.maxBrightness, &_pixelRGB[index].r, &_pixelRGB[index].g, &_pixelRGB[index].b);
  _dma.update = true;
}

static auto encodeByteFrame(uint8_t b, uint8_t f[3]) {
  union {
    uint32_t bits;
    uint8_t  bytes[4];
  } frame{.bits{0b100100100100100100100100}};

  // Encode 1 bit into 3 bits frame data, 0b100 == 0, 0b110 == 1
  if (b & 1)
    frame.bits |= 1 << 1;

  if (b & 2)
    frame.bits |= 1 << 4;

  if (b & 4)
    frame.bits |= 1 << 7;

  if (b & 8)
    frame.bits |= 1 << 10;

  if (b & 16)
    frame.bits |= 1 << 13;

  if (b & 32)
    frame.bits |= 1 << 16;

  if (b & 64)
    frame.bits |= 1 << 19;

  if (b & 128)
    frame.bits |= 1 << 22;

  f[0] = frame.bytes[2];
  f[1] = frame.bytes[1];
  f[2] = frame.bytes[0];
}

auto V2LED::WS2812::encodePixel(const struct PixelRGB* rgb, struct PixelDMA* dma) -> void {
  encodeByteFrame(rgb->r, dma->r);
  encodeByteFrame(rgb->g, dma->g);
  encodeByteFrame(rgb->b, dma->b);
}

auto V2LED::WS2812::setMaxBrightness(float fraction) -> void {
  _leds.maxBrightness = fraction;
  _dma.update         = true;
}

auto V2LED::WS2812::setRGB(uint16_t index, uint8_t r, uint8_t g, uint8_t b) -> void {
  if (isRainbow())
    return;

  _pixelRGB[index].r = (float)r * _leds.maxBrightness;
  _pixelRGB[index].g = (float)g * _leds.maxBrightness;
  _pixelRGB[index].b = (float)b * _leds.maxBrightness;
  _dma.update        = true;
}

auto V2LED::WS2812::splashHSV(float seconds, uint16_t start, uint16_t count, float h, float s, float v) -> void {
  convertWS2812(h, s, v, &_splash.pixel.r, &_splash.pixel.g, &_splash.pixel.b);
  _splash.start        = start;
  _splash.count        = count;
  _splash.durationUsec = seconds * 1000.f * 1000.f;
  _splash.startUsec    = micros();
  _dma.update          = true;
}

auto V2LED::WS2812::rainbow(uint8_t cycles, float seconds, float brightness, bool reverse) -> void {
  _rainbow.cycleSteps = (360 / _leds.count) * cycles;
  _rainbow.moveSteps  = (360.f / 40.f) / seconds;
  _rainbow.brightness = brightness;
  _rainbow.reverse    = reverse;
}
