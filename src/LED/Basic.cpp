#include "V2LED.h"
#include <wiring_private.h>

auto V2LED::Basic::tick() -> void {
  if (!_timer->isFraction())
    _pin.high();

  else
    _pin.low();
}

auto V2LED::Basic::setBrightness(float fraction) -> void {
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

auto V2LED::Basic::flash(float seconds, float brightness) -> void {
  _flash.startUsec    = micros();
  _flash.durationUsec = seconds * 1000.f * 1000.f;
  setBrightness(brightness);
}

auto V2LED::Basic::loop() -> void {
  if (_flash.durationUsec == 0)
    return;

  if ((unsigned long)(micros() - _flash.startUsec) < _flash.durationUsec)
    return;

  _flash.durationUsec = 0;
  setBrightness(0);
}

auto V2LED::Basic::reset() -> void {
  _flash = {};
  _timer->setFraction(0);
  _timer->disable();
  _pin.low();
}
