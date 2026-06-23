#pragma once
#include <cstdint>
#include <cstring>

namespace V2MIDI {
  class Packet {
  public:
    // USB MIDI packet - every packet is 4 bytes long.
    // 1. header (4 bits virtual port/wire number + 4 bits code index number)
    // 2. status (7th bit set + 3 bits type + 4 bits channel / system number)
    // 3. data byte 1 (7 bit)
    // 4. data byte 2 (7 bit)
    enum class CodeIndex : uint8_t {
      Reserved             = 0,
      Cable                = 1,
      SystemCommon2        = 2,
      SystemCommon3        = 3,
      SystemExclusiveStart = 4,
      SystemExclusiveEnd1  = 5,
      SystemExclusiveEnd2  = 6,
      SystemExclusiveEnd3  = 7,
      NoteOff              = 8,
      NoteOn               = 9,
      Aftertouch           = 10,
      ControlChange        = 11,
      ProgramChange        = 12,
      AftertouchChannel    = 13,
      PitchBend            = 14,
      SingleByte           = 15
    };

    // MIDI status byte (bit 4 to 7, bit 7 is always set).
    enum class Status : uint8_t {
      NoteOff           = 0x80 | (0 << 4), // [note, velocity]
      NoteOn            = 0x80 | (1 << 4), // [note, velocity]
      Aftertouch        = 0x80 | (2 << 4), // [note, pressure]
      ControlChange     = 0x80 | (3 << 4), // [control function, value]
      ProgramChange     = 0x80 | (4 << 4), // [program]
      AftertouchChannel = 0x80 | (5 << 4), // [pressure]
      PitchBend         = 0x80 | (6 << 4), // [value LSB, value MSB]
      System            = 0x80 | (7 << 4),

      // 'System' messages are device global, the channel number
      // indentifies the type of system message.
      SystemExclusive            = System | 0,  // [stream of 7-bit bytes terminated with 'ExclusiveEnd']
      SystemTimeCodeQuarterFrame = System | 1,  // [4 bits of timecodeIndex fragment]
      SystemSongPosition         = System | 2,  // [value LSB, value MSB]
      SystemSongSelect           = System | 3,  // [song]
      SystemTuneRequest          = System | 6,  // n/a
      SystemExclusiveEnd         = System | 7,  // n/a
      SystemClock                = System | 8,  // n/a
      SystemStart                = System | 10, // n/a
      SystemContinue             = System | 11, // n/a
      SystemStop                 = System | 12, // n/a
      SystemActiveSensing        = System | 14, // n/a
      SystemReset                = System | 15  // n/a
    };

    // LE bit order: [port | codeIndex]
    CodeIndex              codeIndex : 4 {};
    uint8_t                port : 4 {};
    std::array<uint8_t, 3> data{};

    auto channel() const -> uint8_t {
      return data[0] & 0x0f;
    }

    auto channel(uint8_t channel) {
      data[0] &= 0xf0;
      data[0] |= channel;
    }

    static auto status(uint8_t b) -> Status {
      // Remove channel number.
      if (auto status{Status(b & 0xf0)}; status != Status::System)
        return status;

      // 'System' messages carry their message type.
      return Status(b);
    }

    auto type() const -> Status {
      return status(data[0]);
    }

    auto getNote() const -> uint8_t {
      return data[1];
    }

    auto getNoteVelocity() const -> uint8_t {
      return data[2];
    }

    auto getAftertouchNote() const -> uint8_t {
      return data[1];
    }

    auto getAftertouch() const -> uint8_t {
      return data[2];
    }

    auto getController() const -> uint8_t {
      return data[1];
    }

    auto getControllerValue() const {
      return data[2];
    }

    auto getProgram() const -> uint8_t {
      return data[1];
    }

    auto getAftertouchChannel() const -> uint8_t {
      return data[1];
    }

    auto getPitchBend() const -> uint16_t {
      // 14 bit – 8192..8191.
      auto value{int16_t(data[2] << 7 | data[1])};
      return value - 8192;
    }

    auto getSongPosition() const -> uint16_t {
      return data[2] << 7 | data[1];
    }

    auto getSongSelect() const -> uint16_t {
      return data[1];
    }

    auto set(uint8_t data0, uint8_t data1 = 0, uint8_t data2 = 0) -> Packet* {
      switch (status(data0)) {
        case Status::NoteOff:
          codeIndex = CodeIndex::NoteOff;
          break;

        case Status::NoteOn:
          codeIndex = CodeIndex::NoteOn;
          break;

        case Status::Aftertouch:
          codeIndex = CodeIndex::Aftertouch;
          break;

        case Status::ControlChange:
          codeIndex = CodeIndex::ControlChange;
          break;

        case Status::ProgramChange:
          codeIndex = CodeIndex::ProgramChange;
          break;

        case Status::AftertouchChannel:
          codeIndex = CodeIndex::AftertouchChannel;
          break;

        case Status::PitchBend:
          codeIndex = CodeIndex::PitchBend;
          break;

        case Status::SystemSongSelect:
        case Status::SystemTimeCodeQuarterFrame:
          codeIndex = CodeIndex::SystemCommon2;
          break;

        case Status::SystemSongPosition:
          codeIndex = CodeIndex::SystemCommon3;
          break;

        case Status::SystemTuneRequest:
        case Status::SystemClock:
        case Status::SystemStart:
        case Status::SystemContinue:
        case Status::SystemStop:
        case Status::SystemActiveSensing:
        case Status::SystemReset:
          codeIndex = CodeIndex::SingleByte;
          break;
      }

      data[0] = data0;
      data[1] = data1;
      data[2] = data2;
      return this;
    }

    auto set(Status type, uint8_t channel, uint8_t data1 = 0, uint8_t data2 = 0) -> Packet* {
      switch (type) {
        case Status::NoteOff:
          codeIndex = CodeIndex::NoteOff;
          break;

        case Status::NoteOn:
          codeIndex = CodeIndex::NoteOn;
          break;

        case Status::Aftertouch:
          codeIndex = CodeIndex::Aftertouch;
          break;

        case Status::ControlChange:
          codeIndex = CodeIndex::ControlChange;
          break;

        case Status::ProgramChange:
          codeIndex = CodeIndex::ProgramChange;
          break;

        case Status::AftertouchChannel:
          codeIndex = CodeIndex::AftertouchChannel;
          break;

        case Status::PitchBend:
          codeIndex = CodeIndex::PitchBend;
          break;

        default:
          std::abort();
      }

      data[0] = uint8_t(type) | channel;
      data[1] = data1;
      data[2] = data2;
      return this;
    }

    auto setSystem(Status type, uint8_t data1 = 0, uint8_t data2 = 0) -> Packet* {
      switch (type) {
        case Status::SystemSongSelect:
        case Status::SystemTimeCodeQuarterFrame:
          codeIndex = CodeIndex::SystemCommon2;
          break;

        case Status::SystemSongPosition:
          codeIndex = CodeIndex::SystemCommon3;
          break;

        case Status::SystemTuneRequest:
        case Status::SystemClock:
        case Status::SystemStart:
        case Status::SystemContinue:
        case Status::SystemStop:
        case Status::SystemActiveSensing:
        case Status::SystemReset:
          codeIndex = CodeIndex::SingleByte;
          break;

        default:
          std::abort();
      }

      data[0] = uint8_t(type);
      data[1] = data1;
      data[2] = data2;
      return this;
    }

    auto setNote(uint8_t channel, uint8_t note, uint8_t velocity) -> Packet* {
      if (velocity == 0)
        return set(Status::NoteOff, channel, note, 64);

      return set(Status::NoteOn, channel, note, velocity);
    }

    auto setNoteOff(uint8_t channel, uint8_t note, uint8_t velocity) -> Packet* {
      return set(Status::NoteOff, channel, note, velocity);
    }

    auto setAftertouch(uint8_t channel, uint8_t note, uint8_t pressure) -> Packet* {
      return set(Status::Aftertouch, channel, note, pressure);
    }

    auto setControlChange(uint8_t channel, uint8_t controller, uint8_t value = 0) -> Packet* {
      return set(Status::ControlChange, channel, controller, value);
    }

    auto setAftertouchChannel(uint8_t channel, uint8_t pressure) -> Packet* {
      return set(Status::AftertouchChannel, channel, pressure, 0);
    }

    auto setProgram(uint8_t channel, uint8_t value) {
      return set(Status::ProgramChange, channel, value, 0);
    }

    auto setPitchBend(uint8_t channel, int16_t value) {
      // 14 bit – 8192..8191.
      auto bits{uint16_t(value + 8192)};
      return set(Status::PitchBend, channel, bits & 0x7f, (bits >> 7) & 0x7f);
    }
  };

  static_assert(sizeof(Packet) == 4);
};
