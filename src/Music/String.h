#pragma once

namespace V2Music {
  class String {
  public:
    // Get the fraction of the length of the vibrating string, for the given n-th
    // note above the base note of the string.
    static float getFraction(uint8_t note) {
      return powf(0.5, (float)note / 12.f);
    }

    // The the length to shorten the string by (the fret position from the shoulder)
    // for the n-th note above the base note of the string.
    static float getNoteDistance(uint8_t note, float length) {
      // Get the string length for the note.
      const float l = length * getFraction(note);

      // The distance from the end.
      return length - l;
    }
  };
}
