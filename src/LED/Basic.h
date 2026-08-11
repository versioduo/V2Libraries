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
}

inline auto V2LED::Basic::tick() -> void {
  if (!_timer->isFraction())
    _pin.high();

  else
    _pin.low();
}

inline auto V2LED::Basic::brightness(float fraction) -> void {
  if (fraction <= 0) {
    _flash = {};
    _timer->setFraction(0);
    _timer->disable();
    _pin.low();
    return;
  }

  if (fraction >= 1) {
    _timer->setFraction(0);
    _timer->disable();
    _pin.high();
    return;
  }

  _timer->setFraction(fraction);
  _timer->enable();
}

inline auto V2LED::Basic::flash(float seconds, float brightness) -> void {
  _flash.startUsec    = micros();
  _flash.durationUsec = seconds * 1000.f * 1000.f;
  this->brightness(brightness);
}

inline auto V2LED::Basic::loop() -> void {
  if (_flash.durationUsec == 0)
    return;

  if ((unsigned long)(micros() - _flash.startUsec) < _flash.durationUsec)
    return;

  _flash.durationUsec = 0;
  brightness(0);
}

inline auto V2LED::Basic::reset() -> void {
  _flash = {};
  _timer->setFraction(0);
  _timer->disable();
  _pin.low();
}
