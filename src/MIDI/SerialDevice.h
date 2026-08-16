#pragma once
#include "Packet.h"
#include "Port.h"
#include <V2Base.h>

namespace V2MIDI {
  class SerialDevice : public Port {
  public:
    SerialDevice() = delete;
    constexpr SerialDevice(Uart* uart, const std::string name) : Port{name}, _uart(uart) {}

    auto begin() {
      _uart->begin(31250);
      _uart->setTimeout(1);
    }

    auto reset() {
      _state   = {};
      _channel = {};
      _status  = {};
      _data1   = {};
    }

    auto send(Packet& midi) -> bool override {
      if (_uart->availableForWrite() < 3)
        return false;

      switch (midi.type()) {
        case Packet::Status::NoteOn:
        case Packet::Status::NoteOff:
        case Packet::Status::Aftertouch:
        case Packet::Status::ControlChange:
        case Packet::Status::PitchBend:
        case Packet::Status::SystemSongPosition:
          _uart->write(midi.data.data(), 3);
          statistics.output.packet++;
          return true;

        case Packet::Status::ProgramChange:
        case Packet::Status::AftertouchChannel:
        case Packet::Status::SystemTimeCodeQuarterFrame:
        case Packet::Status::SystemSongSelect:
          _uart->write(midi.data.data(), 2);
          statistics.output.packet++;
          return true;

        case Packet::Status::SystemTuneRequest:
        case Packet::Status::SystemClock:
        case Packet::Status::SystemStart:
        case Packet::Status::SystemContinue:
        case Packet::Status::SystemStop:
        case Packet::Status::SystemActiveSensing:
        case Packet::Status::SystemReset:
          _uart->write(midi.data[0]);
          statistics.output.packet++;
          return true;

        // System Exclusive is only handled right now.
        case Packet::Status::SystemExclusive:
          return false;
      }

      return false;
    }

    auto receive(Packet& midi) -> bool override {
      if (_uart->available() == 0)
        return false;

      uint8_t b = _uart->read();

      if (b & 0x80) {
        switch (b) {
          // Real-Time messages do not update the current Running Status. Do not process,
          // forward them immediately.
          case (uint8_t)Packet::Status::SystemClock:
          case (uint8_t)Packet::Status::SystemStart:
          case (uint8_t)Packet::Status::SystemContinue:
          case (uint8_t)Packet::Status::SystemStop:
          case (uint8_t)Packet::Status::SystemActiveSensing:
          case (uint8_t)Packet::Status::SystemReset:
            midi.setSystem(Packet::Status(b), 0, 0);
            statistics.input.packet++;
            return true;
        }

        _state = State::Status;
      }

      switch (_state) {
        case State::Idle:
          return false;

        case State::Status: {
          _status  = Packet::status(b);
          _channel = b & 0x0f;
          switch (_status) {
            // Single byte message, the Real-Time messages are already handled.
            case Packet::Status::SystemTuneRequest:
              midi.set(_status, 0, 0, 0);
              _state = State::Idle;
              statistics.input.packet++;
              return true;

            // Wait for next byte.
            case Packet::Status::ProgramChange:
            case Packet::Status::AftertouchChannel:
            case Packet::Status::SystemTimeCodeQuarterFrame:
            case Packet::Status::SystemSongSelect:
            case Packet::Status::NoteOn:
            case Packet::Status::NoteOff:
            case Packet::Status::ControlChange:
            case Packet::Status::PitchBend:
            case Packet::Status::SystemSongPosition:
              _state = State::Data1;
              return false;

            case Packet::Status::SystemExclusive:
              _state = State::SysEx;
              return false;
          }

        } break;

        case State::Data1:
          switch (_status) {
            // Two bytes message.
            case Packet::Status::ProgramChange:
            case Packet::Status::AftertouchChannel:
            case Packet::Status::SystemTimeCodeQuarterFrame:
            case Packet::Status::SystemSongSelect:
              midi.set(_status, _channel, b, 0);
              _state = State::Idle;
              statistics.input.packet++;
              return true;

            // Wait for next byte.
            case Packet::Status::NoteOn:
            case Packet::Status::NoteOff:
            case Packet::Status::Aftertouch:
            case Packet::Status::ControlChange:
            case Packet::Status::PitchBend:
            case Packet::Status::SystemSongPosition:
              _data1 = b;
              _state = State::Data2;
              return false;
          }
          break;

        case State::Data2:
          midi.set(_status, _channel, _data1, b);
          _state = State::Idle;
          statistics.input.packet++;
          return true;

        case State::SysEx:
          // System Exclusive is not processed right now. Just discard  the bytes
          // until the next status byte arrives.
          return false;
      }

      return false;
    }

  private:
    enum class State {
      Idle,
      Status,
      Data1,
      Data2,
      SysEx,
    } _state{};

    uint8_t        _channel{};
    Packet::Status _status{};
    uint8_t        _data1{};

    Uart* _uart{};
  };
};
