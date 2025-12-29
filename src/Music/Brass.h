#pragma once
#include <V2MIDI.h>

namespace V2Music::Brass {
  class Valve {
  public:
    void play(uint8_t note) {
      switch (note) {
        case V2MIDI::Fs(1)... V2MIDI::C(2):
          setPosition(V2MIDI::C(2) - note);
          break;

        case V2MIDI::Cs(2)... V2MIDI::F(2):
          setPosition(V2MIDI::F(2) - note);
          break;

        case V2MIDI::Fs(2)... V2MIDI::C(3):
          setPosition(V2MIDI::C(3) - note);
          break;

        case V2MIDI::Cs(3)... V2MIDI::G(3):
          setPosition(V2MIDI::G(3) - note);
          break;

        case V2MIDI::Gs(3)... V2MIDI::C(4):
          setPosition(V2MIDI::C(4) - note);
          break;

        case V2MIDI::Cs(4)... V2MIDI::E(4):
          setPosition(V2MIDI::E(4) - note);
          break;

        case V2MIDI::F(4)... V2MIDI::G(4):
          setPosition(V2MIDI::G(4) - note);
          break;

        case V2MIDI::Gs(4)... V2MIDI::C(5):
          setPosition(V2MIDI::C(5) - note);
          break;
      }
    }

    // The number of semitones to lower the pitch.
    void setPosition(uint8_t position) {
      bool valves[3]{};

      switch (position) {
        case 1:
          valves[1] = true;
          break;

        case 2:
          valves[0] = true;
          break;

        case 3:
          valves[0] = true;
          valves[1] = true;
          break;

        case 4:
          valves[1] = true;
          valves[2] = true;
          break;

        case 5:
          valves[0] = true;
          valves[2] = true;
          break;

        case 6:
          valves[0] = true;
          valves[1] = true;
          valves[2] = true;
          break;
      }

      handleSetValve(valves);
    }

  protected:
    virtual void handleSetValve(bool valves[3]) {}
  };
}
