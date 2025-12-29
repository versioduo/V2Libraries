#pragma once
#include "Fader.h"

namespace V2Audio {
  class Phasor {
  public:
    void reset() {
      _phase = 0;
      _increment.reset();
    }

    float get() const {
      return _phase;
    }

    operator float() const {
      return get();
    }

    void set(float phase) {
      _phase = phase;
    }

    // Set the time it will take to transition from one frequency to the other. Smaller
    // deltas will take a fraction of the given range.
    void setFaderSpeed(float durationSec, float fromFrequency, float toFrequency) {
      const float fromSamples = getClockFrequency() / fromFrequency;
      const float fromTarget  = 1.f / fromSamples;
      const float toSamples   = getClockFrequency() / toFrequency;
      const float toTarget    = 1.f / toSamples;

      const float range = toTarget - fromTarget;
      const float steps = durationSec * getClockFrequency();
      _increment.setStepsRange(steps, range);
    }

    void setFrequencyTarget(float frequency) {
      const float nSamples = getClockFrequency() / frequency;
      _increment.setTarget(1.f / nSamples);
    }

    void setFrequency(float frequency) {
      const float nSamples = getClockFrequency() / frequency;
      _increment.set(1.f / nSamples);
    }

    bool step() {
      bool wrap{};

      _phase += _increment;
      if (_phase >= 1.f) {
        _phase -= 1.f;
        wrap = true;
      }

      _increment.step();
      return wrap;
    }

  protected:
    virtual float getClockFrequency() = 0;

  private:
    float _phase{};
    Fader _increment;
  };
}
