#pragma once
#include <V2Base.h>

// Intelligent solenoid power controller, adjusts to changes in supply voltage or
// different solenoid parameters. The trigger pulse is specified by its length in
// seconds and the electrical power in watts. The PWM duty cycle is calculated
// using the measured solenoid resistance and the supplied voltage.
//
// For longer pulses, the solenoid is held by a fraction of the power that is
// used to move it.
//
// The trigger / release of the solenoid power can be ramped up / down to move
// the solenoids with less noise.
template <uint8_t nPorts> class V2Solenoids {
public:
  // Example:
  //   static const Configuration config{
  //     .current{.max{3}, .alpha{0.001}},
  //     .resistance{.min{6}, .max{60}},
  //     .fade{.inSec{0.35}, .outSec{0.35}},
  //     .hold{.peakUsec{100 * 1000}, .fraction{0.5}}
  //   };
  struct Configuration {
    struct {
      float max{};
      float alpha{};
    } current;

    struct {
      float min{};
      float max{};
    } resistance;

    // Time to to fade-in/out, adjusted one step per millisecond.
    struct {
      float inSec{};
      float outSec{};
    } fade;

    // Reduced holding power after peak period.
    struct {
      uint32_t peakUsec{};
      float    fraction{};
    } hold;
  };

  constexpr V2Solenoids(Configuration config) : _config(config) {}

  void reset() {
    setPower(PowerState::Off);

    _loopUsec    = 0;
    _powerUsec   = 0;
    _timeoutUsec = V2Base::getUsec();
    _probe       = {};
    _current     = 0;

    for (uint8_t i = 0; i < nPorts; i++) {
      _ports[i] = {.measure{.resistance{-1}, .voltage{-1}}};
      setPWMDuty(i, 0);
      updateLED(i);
    }
  }

  void loop() {
    bool busy{};

    if (V2Base::getUsecSince(_loopUsec) < 1000)
      return;

    _loopUsec = V2Base::getUsec();

    // Switch off the LEDs.
    if (_timeoutUsec > 0 && V2Base::getUsecSince(_timeoutUsec) > 60 * 1000 * 1000) {
      _timeoutUsec = 0;

      for (uint8_t i = 0; i < nPorts; i++)
        updateLED(i);
    }

    for (uint8_t i = 0; i < nPorts; i++) {
      if (_ports[i].state == DriverState::Idle)
        continue;

      busy = true;

      switch (_ports[i].state) {
        case DriverState::FadeIn:
          _ports[i].duty += _ports[i].pulse.duty.delta;
          setPWMDuty(i, _ports[i].duty);

          if (_ports[i].duty >= _ports[i].pulse.duty.target) {
            _ports[i].pulse.peekUsec = V2Base::getUsec();
            _ports[i].state          = DriverState::Peak;
          }
          break;

        case DriverState::Peak:
          // Limit the peek/actuation period, reduce to the power to hold.
          if (V2Base::getUsecSince(_ports[i].pulse.peekUsec) > _config.hold.peakUsec) {
            _ports[i].duty *= _config.hold.fraction;
            setPWMDuty(i, _ports[i].duty);
            _ports[i].state = DriverState::Hold;
            break;
          }

          if (V2Base::getUsecSince(_ports[i].pulse.startUsec) < _ports[i].pulse.durationUsec)
            break;

          if (!fadeOutPort(i))
            releasePort(i);
          break;

        case DriverState::Hold:
          if (V2Base::getUsecSince(_ports[i].pulse.startUsec) < _ports[i].pulse.durationUsec)
            break;

          if (!fadeOutPort(i))
            releasePort(i);
          break;

        case DriverState::FadeOut:
          _ports[i].duty -= _ports[i].pulse.duty.delta;
          if (_ports[i].duty <= 0) {
            releasePort(i);
            break;
          }

          setPWMDuty(i, _ports[i].duty);
          break;
      }
    }

    measureCurrent();

    // Warn about too much load, and reset/switch-off all ports.
    if (_current > _config.current.max) {
      for (uint8_t i = 0; i < nPorts; i++)
        releasePort(i);

      // Clear the ready flag and force the resistance measurement, short-circuit
      // ports will be isolated.
      _probe = {};
      setLED(LEDMode::OverCurrent);
      return;
    }

    // The main power is still active, we cannot measure the resistance.
    if (busy)
      return;

    if (_powerUsec > 0) {
      // The power needs to be switched-on immediately on incoming packets; to avoid
      // possible high frequent switching, delay the switch-off.
      if (V2Base::getUsecSince(_powerUsec) < 200 * 1000)
        return;

      setPower(PowerState::Off);
      _powerUsec = 0;
    }

    switch (_probe.state) {
      case ProbeState::Init:
        _probe.settleUsec = V2Base::getUsec();
        _probe.state      = ProbeState::Settle;

        if (!_probe.ready) {
          _probe.cycle = 0;
          setLED(LEDMode::Initialize);
        }
        break;

      case ProbeState::Settle:
        // Delay the measurement to let mechanical movements settle; a moving plunger
        // disturbs the measurement by changing the magnetic field / inducing energy.
        if (V2Base::getUsecSince(_probe.settleUsec) < 200 * 1000)
          break;

        setPWMDuty(_probe.port, 1);
        _probe.measureUsec = V2Base::getUsec();
        _probe.state       = ProbeState::Measure;
        break;

      case ProbeState::Measure:
        // Charge the coil; the magnetic field in the solenoid needs to be stable.
        // A too short measurement span does not measure the coil's resistance, it
        // measures the current flow to establish the magnetic field.
        if (V2Base::getUsecSince(_probe.measureUsec) < 10 * 1000)
          break;

        measureResistance();
        setPWMDuty(_probe.port, 0);

        _probe.port++;
        if (_probe.port == nPorts) {
          _probe.port = 0;

          if (!_probe.ready) {
            _probe.cycle++;

            if (_probe.cycle > 10) {
              _probe.ready = true;
              setLED(LEDMode::Ready);
            }
          }
        }

        // Slow down the measurement when idle.
        if (_timeoutUsec == 0) {
          _probe.sleepUsec = V2Base::getUsec();
          _probe.state     = ProbeState::Sleep;
          break;
        }

        setPWMDuty(_probe.port, 1);
        _probe.measureUsec = V2Base::getUsec();
        break;

      case ProbeState::Sleep:
        if (V2Base::getUsecSince(_probe.sleepUsec) < 1000 * 1000)
          break;

        setPWMDuty(_probe.port, 1);
        _probe.measureUsec = V2Base::getUsec();
        _probe.state       = ProbeState::Measure;
        break;
    }
  }

  void triggerPort(uint8_t port, float watts, float seconds, bool fadeIn = false, bool fadeOut = false) {
    if (!_probe.ready)
      return;

    if (watts <= 0 || seconds <= 0) {
      if (!fadeOut || !fadeOutPort(port))
        releasePort(port);

      return;
    }

    if (_ports[port].measure.state != CoilState::Connected)
      return;

    if (_current > _config.current.max)
      return;

    // Stop resistance measurement.
    if (_probe.state != ProbeState::Init) {
      _probe.state = ProbeState::Init;
      releasePort(_probe.port);
    }

    _ports[port].pulse.startUsec    = V2Base::getUsec();
    _ports[port].pulse.durationUsec = seconds * 1000.f * 1000.f;
    _ports[port].pulse.fadeIn       = fadeIn;
    _ports[port].pulse.fadeOut      = fadeOut;
    _ports[port].pulse.duty.target  = wattsToPWMDuty(port, watts);

    // If the LEDs switched off after a timeout, refresh them.
    if (_timeoutUsec == 0)
      for (uint8_t i = 0; i < nPorts; i++)
        updateLED(i, true);

    _timeoutUsec = V2Base::getUsec();

    if (!setPower(PowerState::On))
      return;

    // Delay the next measurement/power-off.
    _powerUsec = V2Base::getUsec();

    if (fadeIn && _ports[port].pulse.duty.target > 0.01f && _ports[port].duty < _ports[port].pulse.duty.target) {
      // The duty cycle will be adjusted one step / delta per millisecond.
      float msec = _config.fade.inSec * 1000.f;
      if (msec > seconds * 1000.f)
        msec = seconds * 1000.f;

      _ports[port].pulse.duty.delta = _ports[port].pulse.duty.target / msec;
      _ports[port].state            = DriverState::FadeIn;

    } else {
      // Immediate switch-on, or fade-in take-over from the current duty cycle.
      _ports[port].duty = _ports[port].pulse.duty.target;
      setPWMDuty(port, _ports[port].duty);
      _ports[port].pulse.peekUsec = V2Base::getUsec();
      _ports[port].state          = DriverState::Peak;
    }

    setLED(LEDMode::Power, port, watts);
  }

  float getCurrent() {
    return _current;
  }

  float getResistance(uint8_t port) {
    switch (_ports[port].measure.state) {
      case CoilState::NotConnected:
        return -1;

      case CoilState::Connected:
        return _ports[port].measure.resistance;

      case CoilState::ShortCircuit:
        return 0;
    }

    return -1;
  }

protected:
  enum class PowerState { On, Off };
  virtual bool  setPower(PowerState state)           = 0;
  virtual float readVoltage()                        = 0;
  virtual float readCurrent()                        = 0;
  virtual float readResistanceVoltage()              = 0;
  virtual void  setPWMDuty(uint8_t port, float duty) = 0;

  enum class LEDMode { Off, Initialize, Ready, Resistance, Power, ShortCircuit, OverCurrent };
  virtual void setLED(LEDMode state, uint8_t port = 0, float value = -1) {}

private:
  const Configuration _config;

  // Limit the adjustment frequency.
  uint32_t _loopUsec{};

  // The last time the power was switched-on.
  uint32_t _powerUsec{};

  // Slow down the measurement, and switch off the channel LEDs.
  uint32_t _timeoutUsec{};

  enum class ProbeState { Init, Settle, Measure, Sleep };
  struct {
    ProbeState state{};

    // Time after power-off for mechanical parts to settle.
    uint32_t settleUsec{};

    // The next port to probe.
    uint8_t port{};

    // The duration of being powered-up to measure the resistance.
    uint32_t measureUsec{};

    // Slow down resistance measurement when idle.
    uint32_t sleepUsec{};

    // Measure the resistance a few times before ready.
    uint8_t cycle;

    // All ports have been measured.
    bool ready{};
  } _probe;

  // Measured current flow.
  float _current{};

  enum class DriverState { Idle, FadeIn, Peak, Hold, FadeOut };
  enum class CoilState { NotConnected, Connected, ShortCircuit };

  struct {
    DriverState state;

    // Current duty cycle. May fade-in/out to/from the target duty cycle.
    float duty;

    struct {
      CoilState state;

      // Measured coil resistance.
      float resistance;

      // Storage of the raw measurement to implements a low-pass filter.
      float voltage;
    } measure;

    // Current pulse parameters.
    struct {
      uint32_t startUsec;
      uint32_t peekUsec;
      uint32_t durationUsec;
      bool     fadeIn;
      bool     fadeOut;

      struct {
        float target;
        float delta;
      } duty;
    } pulse;
  } _ports[nPorts]{};

  void measureResistance() {
    // When the power supply is switched-off, a single port can be switched-on,
    // and 3.3V are connected to a 100Ω voltage divider. It measures the
    // resistance of the connected load.
    //
    // Voltage divider R1 = 100Ω, Vin = 3.3V, R2 has a diode with a drop of ~0.3V:
    //   ∞ = Vout 3.3V
    //  0Ω = Vout 0.3V
    const float voltage = readResistanceVoltage();

    // Start with the current measurement, do not signal a short-circuit after a reset.
    if (_ports[_probe.port].measure.voltage < 0.f)
      _ports[_probe.port].measure.voltage = voltage;

    // Low-pass filter.
    const float alpha = 0.3;
    _ports[_probe.port].measure.voltage *= 1.f - alpha;
    _ports[_probe.port].measure.voltage += voltage * alpha;

    // Calculate voltage divider, R2 resistance.
    const float r1                         = 100;
    const float vIn                        = 3.3;
    const float vOut                       = _ports[_probe.port].measure.voltage;
    // Account for the ~0.3V drop of the R2 diode.
    _ports[_probe.port].measure.resistance = ((vOut - 0.3f) * r1) / (vIn - vOut);

    // Not connected or resistance too high to be useful.
    CoilState state;
    if (_ports[_probe.port].measure.resistance > _config.resistance.max) {
      state = CoilState::NotConnected;
    }

    // Short-circuit or resistance too low.
    else if (_ports[_probe.port].measure.resistance < _config.resistance.min) {
      state = CoilState::ShortCircuit;
    }

    else {
      state = CoilState::Connected;
    }

    // Wakeup the LED display.
    if (state != _ports[_probe.port].measure.state) {
      _ports[_probe.port].measure.state = state;
      _timeoutUsec                      = V2Base::getUsec();
    }

    // Set the LED brightness according to the measured resistance.
    updateLED(_probe.port);
  }

  void measureCurrent() {
    if (_config.current.alpha > 0.f) {
      // Low-pass filter.
      _current *= 1.f - _config.current.alpha;
      _current += readCurrent() * _config.current.alpha;

    } else {
      _current = readCurrent();
    }
  }

  // Convert the watts to a PWM duty cycle, depending on the measured
  // resistance and current voltage.
  float wattsToPWMDuty(uint8_t port, float watts) {
    const float supply  = readVoltage();
    float       voltage = sqrtf(watts * _ports[port].measure.resistance);
    if (voltage > supply)
      voltage = supply;

    return voltage / supply;
  }

  void releasePort(uint8_t port) {
    setPWMDuty(port, 0);
    _ports[port].state = DriverState::Idle;
    _ports[port].duty  = 0;
    _ports[port].pulse = {};
    updateLED(port);
  }

  bool fadeOutPort(uint8_t port) {
    if (!_ports[port].pulse.fadeOut)
      return false;

    if (_ports[port].duty < 0.01f)
      return false;

    _ports[port].pulse.duty.delta = _ports[port].duty / (_config.fade.outSec * 1000.f);
    _ports[port].state            = DriverState::FadeOut;
    return true;
  }

  void updateLED(uint8_t port, bool force = false) {
    if (!force && _timeoutUsec == 0) {
      setLED(LEDMode::Off, port);
      return;
    }

    switch (_ports[port].measure.state) {
      case CoilState::NotConnected:
        setLED(LEDMode::Off, port);
        break;

      case CoilState::Connected:
        setLED(LEDMode::Resistance, port, 1.f - (_ports[port].measure.resistance / _config.resistance.max));
        break;

      case CoilState::ShortCircuit:
        setLED(LEDMode::ShortCircuit, port);
        break;
    }
  }
};
