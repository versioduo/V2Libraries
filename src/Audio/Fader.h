#pragma once

namespace V2Audio {
  class Fader {
  public:
    constexpr Fader(float value = 0, float steps = 1000) : _default({.value{value}, .steps{steps}}) {}

    void reset() {
      set(_default.value);
      setStepsRange(_default.steps);
    }

    // The current value; step() brings it closer to the target value.
    float get() const {
      return _now;
    }

    operator float() const {
      return get();
    }

    void set(float value) {
      _now       = value;
      _target    = value;
      _adjusting = false;
    }

    float getTarget() const {
      return _target;
    }

    void setTarget(float target) {
      _target    = target;
      _adjusting = true;
    }

    // Set the number of steps to take for a given range.
    void setStepsRange(float steps, float range = 1) {
      _delta = range / steps;
    }

    // Adjust the current value one delta towards the target value. Return if an adjustment was made.
    bool step() {
      if (!_adjusting)
        return false;

      const float distance = _target - _now;
      if (fabs(_delta) > fabs(distance)) {
        // Reached the target.
        _now       = _target;
        _adjusting = false;
        return true;
      }

      _now += copysignf(_delta, distance);
      return true;
    }

  private:
    const struct {
      const float value;
      const float steps;
    } _default;

    bool  _adjusting{};
    float _now{};
    float _target{};
    float _delta{};
  };
};
