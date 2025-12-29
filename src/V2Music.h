#pragma once
#include "Music/Brass.h"
#include "Music/Keyboard.h"
#include "Music/Scale.h"
#include "Music/String.h"
#include <V2MIDI.h>

namespace V2Music {
  class Frequency {
  public:
    static float fromNote(uint8_t note) {
      return 440.f * powf(2.f, static_cast<float>(note - 69.f) / 12.f);
    }

    // Adjust the given frequency by cents of a note; used for tuning / calibration.
    static float adjustFrequency(float frequency, float cents) {
      const float noteDistance = cents / 100.f;
      return frequency * powf(2.f, -noteDistance / 12.f);
    }
  };

  // A stack of currently playing notes with their velocity. A velocity of 0 will
  // remove the note.
  template <uint8_t size> class Playing {
  public:
    void update(uint8_t note, uint8_t velocity) {
      // Remove note.
      for (uint8_t i = 0; i < _n; i++) {
        if (_list[i].note != note)
          continue;

        for (uint8_t k = i; k < _n - 1; k++)
          _list[k] = _list[k + 1];

        _n--;
        break;
      }

      if (velocity == 0)
        return;

      // Add note.
      _list[_n].note     = note;
      _list[_n].velocity = velocity;
      _n++;
    }

    bool getLast(uint8_t& note, uint8_t& velocity) {
      if (_n == 0)
        return false;

      note     = _list[_n - 1].note;
      velocity = _list[_n - 1].velocity;
      return true;
    }

    void reset() {
      _n = 0;
    }

  private:
    struct {
      uint8_t note;
      uint8_t velocity;
    } _list[size]{};

    uint8_t _n{};
  };

  // A list of values with a priority. The value with the highest prority wins.
  template <uint8_t size> class Priority {
  public:
    int8_t get() {
      // Return the value with the highest priority;
      for (uint8_t i = 0; i < size; i++)
        if (_values[size - 1 - i] >= 0)
          return _values[size - 1 - i];

      return -1;
    }

    bool set(int8_t value, uint8_t priority) {
      _values[priority] = value;

      for (uint8_t i = priority + 1; i < size; i++)
        if (_values[i] >= 0)
          return false;

      return true;
    }

    void reset() {
      for (uint8_t i = 0; i < size; i++)
        _values[i] = -1;
    }

  private:
    int8_t _values[size];
  };

  // Pressing 'Stop' in the audio workstation usually sends the control message AllNotesOff/AllSoundOff.
  // Pressing it manually multiple times in a row, triggers a forced stop, disables all devices; it behaves
  // like a panic button. Bursts of triggers in a very short time frame only, do not cause a forced stop.
  // They might be sent by multiple active tracks, not by the user hitting 'Stop' repeatedly.
  class ForcedStop {
  public:
    void reset() {
      _state = State::Idle;
      _usec  = 0;
    }

    bool trigger() {
      switch (_state) {
        case State::Idle:
          _state = State::Trigger;
          _usec  = micros();
          break;

        case State::Trigger:
          // Do not trigger right away for bursts in a very short time frame.
          if ((unsigned long)(micros() - _usec) < 100 * 1000)
            break;

          // The triggers are too far apart from each other.
          if ((unsigned long)(micros() - _usec) > 500 * 1000) {
            reset();
            break;
          }

          // Repeated triggers over a longer period of time.
          reset();
          return true;
      }

      return false;
    }

  private:
    enum State { Idle, Trigger } _state{};
    unsigned long _usec{};
  };
};
