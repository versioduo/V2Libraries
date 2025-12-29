#pragma once
#include <Arduino.h>

class V2Buttons {
public:
  // No config or zero values will not count clicks and not detect long-presses.
  struct Config {
    // Time to wait count multiple clicks, usually ~200 milliseconds.
    unsigned long clickUsec;

    // Time to detect a long-press, usually ~500 milliseconds.
    unsigned long holdUsec;
  };

  class Button {
  public:
    // The the internal pulldown/up resistor will be enabled if pushpull is false.
    constexpr Button(const Config* config, uint8_t pin, bool high = false, bool pushpull = false) :
      _config(config),
      _pin(pin),
      _high{high},
      _pushpull{pushpull} {}

    void begin();
    bool loop();

  protected:
    virtual void handleDown() {}
    virtual void handleUp() {}

    // The button was pressed and is released, a click; 'count' indicates the
    // number of clicks immedeately before. A single click is 0, double-click
    // is 1.
    virtual void handleClick(uint8_t count) {}

    // The button is pressed and held, a long-press; 'count' indicates the
    // number of clicks immedeately before. Just pressing and holding the button
    // is 0.
    virtual void handleHold(uint8_t count) {}
    virtual void handleRelease() {}

    // Return event number to match Down/Up/Click/Hold/Release calls.
    uint32_t getEvent() {
      return _event;
    }

  private:
    friend class V2Buttons;
    const Config* _config;
    const uint8_t _pin;
    bool          _high;
    bool          _pushpull;
    Button*       _next{};
    enum class State { Idle, WaitDown, Down, Hold, Up, Reset } _state{State::Idle};
    uint8_t       _clicks{};
    unsigned long _usec{};

    // Event number assigned when we wake up from idle. All calls for the same
    // sequence will have the same event number.
    uint32_t _event{};

    // Are we processing or wait for a new interrupt.
    bool _busy{};
  };

  // Check for pending events.
  //
  // The pins of the buttons needs to support external interrupts to trigger
  // the measurement.
  static void loop();
};
