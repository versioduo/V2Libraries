#pragma once
#include "Scale.h"

namespace V2Music {
  class Keyboard {
  public:
    // Get the key position in an octave, starting at zero.
    static uint8_t getKey(uint8_t note) {
      return note % 12;
    }

    // Get the octave number, starting at zero.
    static uint8_t getOctave(uint8_t note) {
      return note / 12;
    }

    // Return if the given note is a black piano key.
    static bool isBlackKey(uint8_t note) {
      return !Scale::isUsed(Scale::Major, getKey(note));
    }

    // Get the number of octaves and the fraction of the last octave where the key is located.
    static float getKeyOffset(uint8_t note) {
      // The center of the 12 keys of an octave in piano black/white layout. The black keys are
      // located in-between the white keys, but spread-out a bit towards the space where a black
      // keys is "missing". Only the G# key is right at the center between the white keys.
      constexpr float        shift = 0.02;
      static constexpr float offset[12]{
        1.f * (1.f / 14.f),
        2.f * (1.f / 14.f) - shift,
        3.f * (1.f / 14.f),
        4.f * (1.f / 14.f) + shift,
        5.f * (1.f / 14.f),
        7.f * (1.f / 14.f),
        8.f * (1.f / 14.f) - shift,
        9.f * (1.f / 14.f),
        10.f * (1.f / 14.f), // centered
        11.f * (1.f / 14.f),
        12.f * (1.f / 14.f) + shift,
        13.f * (1.f / 14.f),
      };

      return (float)getOctave(note) + offset[getKey(note)];
    }

    // Get the distance of the given key, starting with the piano's first key A(-1).
    // The standard octave length is 16.5 cm.
    static float getKeyDistance(uint8_t note, float octaveWidth) {
      const float start = getKeyOffset(V2MIDI::A(-1));
      return (getKeyOffset(note) - start) * octaveWidth;
    }
  };
}
