#pragma once
#include <cinttypes>

namespace V2Music {
  class Scale {
  public:
    enum Type {
      Chromatic,
      Major,
      Minor,
      HarmonicMinor,
      PentatonicMinor,
      BluesMinor,
      MelodicMinor,
      Dorian,
      Phrygian,
      PhrygianMajor,
      Lydian,
      Diminished,
      _count
    };

    // Return human readable name.
    static const char* getName(Type type);

    // The total number of notes used in the scale.
    static uint8_t getCount(Type type);

    // All scales start at C.
    static uint8_t getNote(Type type, uint8_t index);

    // Return if the given index of the note is used in this scale.
    static bool isUsed(Type type, uint8_t index);
  };
}
