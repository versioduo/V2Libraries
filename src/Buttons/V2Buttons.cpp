#include "V2Buttons.h"

static struct {
  // Time of last check.
  unsigned long usec;

  // Current event sequence number.
  uint32_t event;

  // The list of all created buttons
  V2Buttons::Button* list;

  volatile bool busy{true};
} _buttons{};

static void pinInterrupt() {
  _buttons.busy = true;
}

void V2Buttons::Button::begin() {
  if (!_pushpull)
    pinMode(_pin, _high ? INPUT_PULLDOWN : INPUT_PULLUP);
  else
    pinMode(_pin, INPUT);

  attachInterrupt(digitalPinToInterrupt(_pin), pinInterrupt, CHANGE);

  this->_next   = _buttons.list;
  _buttons.list = this;
}

void V2Buttons::loop() {
  if (!_buttons.busy)
    return;

  // Limit the frequency of port checks during state transitions.
  if ((unsigned long)(micros() - _buttons.usec) < 1000)
    return;

  _buttons.usec = micros();

  _buttons.busy = false;
  for (Button* button = _buttons.list; button; button = button->_next)
    if (button->loop())
      _buttons.busy = true;
}

bool V2Buttons::Button::loop() {
  const bool down = digitalRead(_pin) == _high ? HIGH : LOW;

  switch (_state) {
    case Button::State::Idle:
      if (down) {
        // The button was pressed, record the down time.
        _state = Button::State::WaitDown;
        _usec  = micros();
        _event = _buttons.event++;
        _busy  = true;
      }
      break;

    case Button::State::WaitDown:
      // Wait for the button state to settle.
      if ((unsigned long)(micros() - _usec) < 5 * 1000)
        break;

      if (down) {
        _state = Button::State::Down;
        handleDown();
        break;
      }

      _state = Button::State::Reset;
      break;

    case Button::State::Down:
      if (down) {
        // The button is pressed long enough, raise a 'pressed' event.
        if (!_config || _config->holdUsec == 0) {

          // Wait for next interrupt.
          _busy = false;
          break;
        }

        if ((unsigned long)(micros() - _usec) < _config->holdUsec)
          break;

        _state = Button::State::Hold;
        handleHold(_clicks);
        break;
      }

      // The button was released, a click, not long enough to raise a
      // 'Hold' event.
      _state = Button::State::Up;

      _usec = micros();
      _busy = true;
      break;

    case Button::State::Hold:
      // Wait for next interrupt until the button is released.
      if (down) {
        _busy = false;
        break;
      }

      _state = Button::State::Reset;
      handleRelease();
      handleUp();
      _busy = true;
      break;

    case Button::State::Up:
      if (_config && _config->clickUsec > 0) {
        // If the button was already pressed for a short time and is now pressed
        // again, record the release time and start counting the clicks.
        if (down) {
          _state = Button::State::Down;

          // Count the clicks, the number of transitions between 'Up' and
          // 'Down'.
          _clicks++;
          _usec = micros();
          break;
        }

        // Stay in 'Up' for a short time, detect (check above) a possible down.
        if ((unsigned long)(micros() - _usec) < _config->clickUsec)
          break;
      }

      // The time has passed, it was the last click in this sequence.
      handleClick(_clicks);
      handleUp();
      _state = Button::State::Reset;
      break;

    case Button::State::Reset:
      _state  = Button::State::Idle;
      _clicks = 0;
      _busy   = false;
      _event  = 0;
      break;
  }

  return _busy;
}
