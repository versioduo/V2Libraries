#pragma once
#include "Clock.h"
#include "Packet.h"
#include "Port.h"
#include <algorithm>
#include <cstdlib>

namespace V2MIDI {
  // Transport-independent MIDI functional interface. Supports message parsing/dispatching,
  // system exclusive buffering/streaming, packet statistics.
  class Device : public Port {
  public:
    Device() = delete;
    constexpr Device(uint8_t index) : Port{"device"}, _portIndex{index} {}

    // Replies can be sent back to the originating port.
    auto dispatch(Port* port, Packet* packet) -> void;

    // The buffer to copy the SysEx message into.
    auto systemExclusiveBuffer() -> std::string&;

    // Set the port's number in the outgoing packet and updates the statistics.
    auto send(Packet& p) -> bool override;
    auto send(Packet* p) -> bool {
      return send(*p);
    }

    // Prepare SysEx message to chunk into packets. Send as many packets as possible,
    // the remaining packets will be sent with loopSystemExclusive().
    auto sendSystemExclusive(Port& port) -> void;

    auto resetSystemExclusive() -> void;

    // Send the next packet over the specified port. Returns:
    //  0: nothing to do,
    // -1: sending failed,
    //  1: there are remaining packets.
    int8_t loopSystemExclusive();

  protected:
    virtual auto handleNote(uint8_t channel, uint8_t note, uint8_t velocity) -> void {}
    virtual auto handleNoteOff(uint8_t channel, uint8_t note, uint8_t velocity) -> void {}
    virtual auto handleAftertouch(uint8_t channel, uint8_t note, uint8_t pressure) -> void {}
    virtual auto handleControlChange(uint8_t channel, uint8_t controller, uint8_t value) -> void {}
    virtual auto handleProgramChange(uint8_t channel, uint8_t value) -> void {}
    virtual auto handleAftertouchChannel(uint8_t channel, uint8_t pressure) -> void {}
    virtual auto handlePitchBend(uint8_t channel, int16_t value) -> void {}
    virtual auto handleSongPosition(uint16_t beats) -> void {}
    virtual auto handleSongSelect(uint8_t number) -> void {}
    virtual auto handleClock(Clock::Event clock) -> void {}
    virtual auto handleSystemExclusive(const uint8_t* buffer, uint32_t len) -> void {}
    virtual auto handleSystemReset() -> void {}
    virtual auto handleSwitchChannel(uint8_t channel) -> void {}

    // All messages besides system exclusive.
    virtual auto handlePacket(Packet* packet) -> void {}

    // During dispatch, replies are sent back to the originating port.
    virtual auto handleSystemExclusive(Port* port, const uint8_t* buffer, uint32_t len) -> void {}

    virtual auto handleSend(Packet* packet) -> bool {
      return false;
    }

  private:
    const uint8_t _portIndex;

    struct {
      struct {
        std::string buffer{};
        bool        appending{};

        void reset() {
          buffer.clear();
          appending = false;
        }
      } in;

      struct {
        Port*            port{};
        std::string      buffer{};
        std::string_view remain;

        void reset() {
          port = nullptr;
          buffer.clear();
          remain = {};
        }
      } out;
    } _sysex;

    bool storeSystemExclusive(Packet* packet);
  };
};

inline auto V2MIDI::Device::dispatch(Port* port, Packet* p) -> void {
  statistics.input.packet++;

  if (!storeSystemExclusive(p))
    return;

  if (p->type() != Packet::Status::SystemExclusive)
    handlePacket(p);

  switch (p->type()) {
    case Packet::Status::NoteOn:
      statistics.input.note++;
      handleNote(p->channel(), p->getNote(), p->getNoteVelocity());
      break;

    case Packet::Status::NoteOff:
      statistics.input.noteOff++;
      handleNoteOff(p->channel(), p->getNote(), p->getNoteVelocity());
      break;

    case Packet::Status::Aftertouch:
      statistics.input.aftertouch++;
      handleAftertouch(p->channel(), p->getAftertouchNote(), p->getAftertouch());
      break;

    case Packet::Status::ControlChange:
      statistics.input.control++;
      handleControlChange(p->channel(), p->getController(), p->getControllerValue());
      break;

    case Packet::Status::ProgramChange:
      statistics.input.program++;
      handleProgramChange(p->channel(), p->getProgram());
      break;

    case Packet::Status::AftertouchChannel:
      statistics.input.aftertouchChannel++;
      handleAftertouchChannel(p->channel(), p->getAftertouchChannel());
      break;

    case Packet::Status::PitchBend:
      statistics.input.pitchbend++;
      handlePitchBend(p->channel(), p->getPitchBend());
      break;

    case Packet::Status::SystemSongPosition:
      handleSongPosition(p->getSongPosition());
      break;

    case Packet::Status::SystemSongSelect:
      handleSongSelect(p->getSongSelect());
      break;

    case Packet::Status::SystemClock:
      statistics.input.system.clock.tick++;
      handleClock(Clock::Event::Tick);
      break;

    case Packet::Status::SystemStart:
      handleClock(Clock::Event::Start);
      break;

    case Packet::Status::SystemContinue:
      handleClock(Clock::Event::Continue);
      break;

    case Packet::Status::SystemStop:
      handleClock(Clock::Event::Stop);
      break;

    case Packet::Status::SystemExclusive: {
      statistics.input.system.exclusive++;
      handleSystemExclusive(port, (uint8_t*)_sysex.in.buffer.data(), _sysex.in.buffer.size());
      handleSystemExclusive((uint8_t*)_sysex.in.buffer.data(), _sysex.in.buffer.size());
    } break;

    case Packet::Status::SystemReset:
      statistics.input.system.reset++;
      handleSystemReset();
      break;
  }
}

inline auto V2MIDI::Device::send(Packet& p) -> bool {
  // Do not interrupt a system exclusive transfer.
  if (!_sysex.out.buffer.empty())
    return false;

  p.port = _portIndex;
  if (!handleSend(&p))
    return false;

  statistics.output.packet++;

  switch (p.type()) {
    case Packet::Status::NoteOn:
      statistics.output.note++;
      break;

    case Packet::Status::NoteOff:
      statistics.output.noteOff++;
      break;

    case Packet::Status::Aftertouch:
      statistics.output.aftertouch++;
      break;

    case Packet::Status::ControlChange:
      statistics.output.control++;
      break;

    case Packet::Status::ProgramChange:
      statistics.output.program++;
      break;

    case Packet::Status::AftertouchChannel:
      statistics.output.aftertouchChannel++;
      break;

    case Packet::Status::PitchBend:
      statistics.output.pitchbend++;
      break;

    case Packet::Status::SystemClock:
      statistics.output.system.clock.tick++;
      break;

    case Packet::Status::SystemReset:
      statistics.output.system.reset++;
      break;
  }

  return true;
}

inline auto V2MIDI::Device::systemExclusiveBuffer() -> std::string& {
  return _sysex.out.buffer;
}

inline auto V2MIDI::Device::sendSystemExclusive(Port& port) -> void {
  if (_sysex.out.buffer.size() < 2) {
    statistics.output.error++;
    return;
  }

  if (_sysex.out.buffer.front() != uint8_t(Packet::Status::SystemExclusive)) {
    statistics.output.error++;
    return;
  }

  if (_sysex.out.buffer.back() != uint8_t(Packet::Status::SystemExclusiveEnd)) {
    statistics.output.error++;
    return;
  }

  _sysex.out.port   = &port;
  _sysex.out.remain = _sysex.out.buffer;

  // Send as many packets as possible.
  while (loopSystemExclusive() > 0)
    ;
}

inline auto V2MIDI::Device::resetSystemExclusive() -> void {
  _sysex.in.reset();
  _sysex.out.reset();
}

inline auto V2MIDI::Device::loopSystemExclusive() -> int8_t {
  if (_sysex.out.buffer.empty())
    return 0;

  Packet _p;
  switch (_sysex.out.remain.size()) {
    case 1:
      _p.port      = _portIndex;
      _p.codeIndex = Packet::CodeIndex::SystemExclusiveEnd1;
      _p.data[0]   = _sysex.out.remain.front();
      _p.data[1]   = 0;
      _p.data[2]   = 0;
      break;

    case 2:
      _p.port      = _portIndex;
      _p.codeIndex = Packet::CodeIndex::SystemExclusiveEnd2;
      std::copy_n(_sysex.out.remain.begin(), 2, _p.data.begin());
      _p.data[2] = 0;
      break;

    case 3:
      _p.port      = _portIndex;
      _p.codeIndex = Packet::CodeIndex::SystemExclusiveEnd3;
      std::copy_n(_sysex.out.remain.begin(), 3, _p.data.begin());
      break;

    default:
      _p.port      = _portIndex;
      _p.codeIndex = Packet::CodeIndex::SystemExclusiveStart;
      std::copy_n(_sysex.out.remain.begin(), 3, _p.data.begin());
      break;
  }

  if (!_sysex.out.port) {
    if (!handleSend(&_p))
      return -1;

  } else {
    if (!_sysex.out.port->send(_p))
      return -1;
  }

  statistics.output.packet++;

  if (_sysex.out.remain.size() > 3) {
    _sysex.out.remain.remove_prefix(3);
    return 1;
  }

  statistics.output.system.exclusive++;
  _sysex.out.reset();
  return 0;
}

inline bool V2MIDI::Device::storeSystemExclusive(Packet* p) {
  switch (p->codeIndex) {
    case Packet::CodeIndex::SystemCommon2:
    case Packet::CodeIndex::SystemCommon3:
    case Packet::CodeIndex::NoteOff:
    case Packet::CodeIndex::NoteOn:
    case Packet::CodeIndex::Aftertouch:
    case Packet::CodeIndex::ControlChange:
    case Packet::CodeIndex::ProgramChange:
    case Packet::CodeIndex::AftertouchChannel:
    case Packet::CodeIndex::PitchBend:
      // Return single packet message, discard any possible SysEx stream.
      _sysex.in.reset();
      return true;

    case Packet::CodeIndex::SingleByte:
      // Single byte, like a system message.
      if (!_sysex.in.appending) {
        _sysex.in.reset();
        return true;
      }

      _sysex.in.buffer.push_back(p->data[0]);
      return false;

    // Start of a new SysEx stream, or append data to the current stream.
    case Packet::CodeIndex::SystemExclusiveStart:
      if (!_sysex.in.appending) {
        _sysex.in.buffer.clear();

        // Must be the start of a SysEx.
        if (p->data[0] != uint8_t(Packet::Status::SystemExclusive)) {
          statistics.input.error++;
          return false;
        }

        _sysex.in.appending = true;
      }

      std::copy(p->data.begin(), p->data.end(), std::back_inserter(_sysex.in.buffer));
      return false;

    // End of SysEx stream with various lengths.
    case Packet::CodeIndex::SystemExclusiveEnd1:
      // Invalid 'End' packet
      if (p->data[0] != uint8_t(Packet::Status::SystemExclusiveEnd)) {
        statistics.input.error++;
        _sysex.in.reset();
        return false;
      }

      // 'End' packet without previous data, discarding.
      if (!_sysex.in.appending) {
        statistics.input.error++;
        _sysex.in.reset();
        return false;
      }

      _sysex.in.buffer.push_back(p->data[0]);
      break;

    case Packet::CodeIndex::SystemExclusiveEnd2:
      // Invalid 'End' packet.
      if (p->data[1] != uint8_t(Packet::Status::SystemExclusiveEnd)) {
        statistics.input.error++;
        _sysex.in.reset();
        return false;
      }

      // Single 'End' packet.
      if (!_sysex.in.appending) {
        _sysex.in.buffer.clear();

        // Must be an 'empty' SysEx.
        if (p->data[0] != uint8_t(Packet::Status::SystemExclusive))
          return false;
      }

      std::copy(p->data.begin(), p->data.begin() + 2, std::back_inserter(_sysex.in.buffer));
      break;

    case Packet::CodeIndex::SystemExclusiveEnd3:
      // Invalid 'End' packet.
      if (p->data[2] != uint8_t(Packet::Status::SystemExclusiveEnd)) {
        statistics.input.error++;
        _sysex.in.reset();
        return false;
      }

      // Single 'End' packet.
      if (!_sysex.in.appending) {
        _sysex.in.buffer.clear();

        // Must be a 'one byte' SysEx.
        if (p->data[0] != uint8_t(Packet::Status::SystemExclusive))
          return false;
      }

      std::copy(p->data.begin(), p->data.end(), std::back_inserter(_sysex.in.buffer));
      break;

    default:
      statistics.input.error++;
      _sysex.in.reset();
      return false;
  }

  // Always return 'SystemExclusive' as type.
  _sysex.in.appending = false;
  p->data[0]          = uint8_t(Packet::Status::SystemExclusive);
  return true;
}
