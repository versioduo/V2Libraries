#include "Scale.h"

static const struct {
  const char* name;
  uint8_t     count;
  uint8_t     note[12];
  bool        used[12];
} _notes[V2Music::Scale::_count]{
  [V2Music::Scale::Chromatic]       = {.name  = "Chromatic",
                                       .count = 12,
                                       .note{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11},
                                       .used{true, true, true, true, true, true, true, true, true, true, true, true}},
  [V2Music::Scale::Major]           = {.name  = "Major",
                                       .count = 7,
                                       .note{0, 2, 4, 5, 7, 9, 11},
                                       .used{true, false, true, false, true, true, false, true, false, true, false, true}},
  [V2Music::Scale::Minor]           = {.name  = "Minor",
                                       .count = 7,
                                       .note{0, 2, 3, 5, 7, 8, 10},
                                       .used{true, false, true, true, false, true, false, true, true, false, true, false}},
  [V2Music::Scale::HarmonicMinor]   = {.name  = "Harmonic Minor",
                                       .count = 7,
                                       .note{0, 2, 3, 5, 7, 8, 11},
                                       .used{true, false, true, false, true, true, false, true, true, false, false, true}},
  [V2Music::Scale::PentatonicMinor] = {.name  = "Pentatonic Minor",
                                       .count = 5,
                                       .note{0, 3, 5, 7, 10},
                                       .used{true, false, false, true, false, true, false, true, false, false, true, false}},
  [V2Music::Scale::BluesMinor]      = {.name  = "Blues",
                                       .count = 6,
                                       .note{0, 3, 5, 6, 7, 10},
                                       .used{true, false, false, true, false, true, true, true, false, false, true, false}},
  [V2Music::Scale::MelodicMinor]    = {.name  = "Melodic",
                                       .count = 7,
                                       .note{0, 2, 3, 5, 7, 9, 11},
                                       .used{true, false, true, true, false, true, false, true, true, false, false, true}},
  [V2Music::Scale::Dorian]          = {.name  = "Dorian",
                                       .count = 7,
                                       .note{0, 2, 3, 5, 7, 9, 10},
                                       .used{true, false, true, true, false, true, false, true, false, true, true, false}},
  [V2Music::Scale::Phrygian]        = {.name  = "Phrygian",
                                       .count = 7,
                                       .note{0, 1, 3, 5, 7, 8, 10},
                                       .used{true, true, false, true, false, true, false, true, true, false, true, false}},
  [V2Music::Scale::PhrygianMajor]   = {.name  = "Phrygian Major",
                                       .count = 7,
                                       .note{0, 1, 4, 5, 7, 8, 10},
                                       .used{true, true, false, false, true, true, false, true, true, false, true, false}},
  [V2Music::Scale::Lydian]          = {.name  = "Lydian",
                                       .count = 7,
                                       .note{0, 2, 4, 6, 7, 9, 11},
                                       .used{true, false, true, false, true, false, true, true, false, true, false, true}},
  [V2Music::Scale::Diminished]      = {.name  = "Diminished",
                                       .count = 7,
                                       .note{0, 1, 3, 4, 6, 7, 9},
                                       .used{true, true, false, true, true, false, true, true, false, true, false}}};

const char* V2Music::Scale::getName(V2Music::Scale::Type type) {
  return _notes[type].name;
}

uint8_t V2Music::Scale::getCount(V2Music::Scale::Type type) {
  return _notes[type].count;
}

uint8_t V2Music::Scale::getNote(V2Music::Scale::Type type, uint8_t index) {
  return _notes[type].note[index];
}

bool V2Music::Scale::isUsed(V2Music::Scale::Type type, uint8_t index) {
  return _notes[type].used[index];
}
