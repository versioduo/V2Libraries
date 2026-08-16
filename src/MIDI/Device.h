#pragma once
#include "Clock.h"
#include "Packet.h"
#include "Port.h"
#include <cstdlib>

namespace V2MIDI {
  // Transport-independent MIDI functional interface. Supports message parsing/dispatching,
  // system exclusive buffering/streaming, packet statistics.
  class Device : public Port {
  public:
    Device() = delete;
    constexpr Device(uint8_t index) : Port{"device"}, _portIndex{index} {}

    void begin() {
      // Buffer to store an incoming and outgoing SysEx messages. The buffer needs
      // to be able to carry a complete message. The message always starts with
      // 0xf0 (SystemExclusive) and ends with 0xf7 (SystemExclusiveEnd), all other
      // bytes carry 7-bit only.
      //
      // If no buffer is provided, incoming SysEx messages are discarded.
      _sysex.in.buffer  = (uint8_t*)malloc(_sysexSize);
      _sysex.out.buffer = (uint8_t*)malloc(_sysexSize);
    }

    // During dispatch(), replies can be sent back to the given 'port'.
    void dispatch(Port* port, Packet* packet) {
      statistics.input.packet++;

      if (!storeSystemExclusive(packet))
        return;

      if (packet->type() != Packet::Status::SystemExclusive)
        handlePacket(packet);

      switch (packet->type()) {
        case Packet::Status::NoteOn:
          statistics.input.note++;
          handleNote(packet->channel(), packet->getNote(), packet->getNoteVelocity());
          break;

        case Packet::Status::NoteOff:
          statistics.input.noteOff++;
          handleNoteOff(packet->channel(), packet->getNote(), packet->getNoteVelocity());
          break;

        case Packet::Status::Aftertouch:
          statistics.input.aftertouch++;
          handleAftertouch(packet->channel(), packet->getAftertouchNote(), packet->getAftertouch());
          break;

        case Packet::Status::ControlChange:
          statistics.input.control++;
          handleControlChange(packet->channel(), packet->getController(), packet->getControllerValue());
          break;

        case Packet::Status::ProgramChange:
          statistics.input.program++;
          handleProgramChange(packet->channel(), packet->getProgram());
          break;

        case Packet::Status::AftertouchChannel:
          statistics.input.aftertouchChannel++;
          handleAftertouchChannel(packet->channel(), packet->getAftertouchChannel());
          break;

        case Packet::Status::PitchBend:
          statistics.input.pitchbend++;
          handlePitchBend(packet->channel(), packet->getPitchBend());
          break;

        case Packet::Status::SystemSongPosition:
          handleSongPosition(packet->getSongPosition());
          break;

        case Packet::Status::SystemSongSelect:
          handleSongSelect(packet->getSongSelect());
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
          handleSystemExclusive(port, _sysex.in.buffer, _sysex.in.length);
          handleSystemExclusive(_sysex.in.buffer, _sysex.in.length);
        } break;

        case Packet::Status::SystemReset:
          statistics.input.system.reset++;
          handleSystemReset();
          break;
      }
    }

    auto send(Packet& p) -> bool override {
      return send(&p);
    }

    // Set the port's number in the outgoing packet and updates the statistics.
    bool send(Packet* p) {
      // Do not interrupt a system exclusive transfer.
      if (_sysex.out.length > 0)
        return false;

      p->port = _portIndex;
      if (!handleSend(p))
        return false;

      statistics.output.packet++;

      switch (p->type()) {
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

    // Get the raw buffer to copy the SysEx message into.
    uint8_t* getSystemExclusiveBuffer() {
      return _sysex.out.buffer;
    }

    // Prepare SysEx message to chunk into packets. Send as many packets as possible,
    // the remaining packets will be sent with loopSystemExclusive().
    void sendSystemExclusive(Port* port, uint32_t length) {
      if (_sysex.out.length > 0) {
        statistics.output.error++;
        return;
      }

      if (length < 2) {
        statistics.output.error++;
        return;
      }

      if (_sysex.out.buffer[0] != static_cast<uint8_t>(Packet::Status::SystemExclusive)) {
        statistics.output.error++;
        return;
      }

      if (_sysex.out.buffer[length - 1] != static_cast<uint8_t>(Packet::Status::SystemExclusiveEnd)) {
        statistics.output.error++;
        return;
      }

      _sysex.out.port     = port;
      _sysex.out.length   = length;
      _sysex.out.position = 0;

      // Send as many packets as possible.
      while (loopSystemExclusive() > 0)
        ;
    }

    void resetSystemExclusive() {
      _sysex.in.reset();
      _sysex.out.reset();
    }

    // Send the next packet over the specified port. Returns:
    //  0: nothing to do,
    // -1: sending failed,
    //  1: there are remaining packets.
    int8_t loopSystemExclusive() {
      if (_sysex.out.length == 0)
        return 0;

      Packet   _packet;
      uint32_t remain{_sysex.out.length - _sysex.out.position};
      switch (remain) {
        case 1:
          _packet.port      = _portIndex;
          _packet.codeIndex = Packet::CodeIndex::SystemExclusiveEnd1;
          _packet.data[0]   = _sysex.out.buffer[_sysex.out.position];
          _packet.data[1]   = 0;
          _packet.data[2]   = 0;
          break;

        case 2:
          _packet.port      = _portIndex;
          _packet.codeIndex = Packet::CodeIndex::SystemExclusiveEnd2;
          _packet.data[0]   = _sysex.out.buffer[_sysex.out.position];
          _packet.data[1]   = _sysex.out.buffer[_sysex.out.position + 1];
          _packet.data[2]   = 0;
          break;

        case 3:
          _packet.port      = _portIndex;
          _packet.codeIndex = Packet::CodeIndex::SystemExclusiveEnd3;
          _packet.data[0]   = _sysex.out.buffer[_sysex.out.position];
          _packet.data[1]   = _sysex.out.buffer[_sysex.out.position + 1];
          _packet.data[2]   = _sysex.out.buffer[_sysex.out.position + 2];
          break;

        default:
          _packet.port      = _portIndex;
          _packet.codeIndex = Packet::CodeIndex::SystemExclusiveStart;
          _packet.data[0]   = _sysex.out.buffer[_sysex.out.position];
          _packet.data[1]   = _sysex.out.buffer[_sysex.out.position + 1];
          _packet.data[2]   = _sysex.out.buffer[_sysex.out.position + 2];
          break;
      }

      if (!_sysex.out.port) {
        if (!handleSend(&_packet))
          return -1;

      } else {
        if (!_sysex.out.port->send(_packet))
          return -1;
      }

      statistics.output.packet++;

      if (remain > 3) {
        _sysex.out.position += 3;
        return 1;
      }

      statistics.output.system.exclusive++;
      _sysex.out.port   = nullptr;
      _sysex.out.length = 0;
      return 0;
    }

  protected:
    const uint8_t             _portIndex;
    static constexpr uint32_t _sysexSize{18 * 1024};

    friend class Packet;
    virtual void handleNote(uint8_t channel, uint8_t note, uint8_t velocity) {}
    virtual void handleNoteOff(uint8_t channel, uint8_t note, uint8_t velocity) {}
    virtual void handleAftertouch(uint8_t channel, uint8_t note, uint8_t pressure) {}
    virtual void handleControlChange(uint8_t channel, uint8_t controller, uint8_t value) {}
    virtual void handleProgramChange(uint8_t channel, uint8_t value) {}
    virtual void handleAftertouchChannel(uint8_t channel, uint8_t pressure) {}
    virtual void handlePitchBend(uint8_t channel, int16_t value) {}
    virtual void handleSongPosition(uint16_t beats) {}
    virtual void handleSongSelect(uint8_t number) {}
    virtual void handleClock(Clock::Event clock) {}
    virtual void handleSystemExclusive(const uint8_t* buffer, uint32_t len) {}
    virtual void handleSystemReset() {}
    virtual void handleSwitchChannel(uint8_t channel) {}

    // All messages besides system exclusive.
    virtual void handlePacket(Packet* packet) {}

    // During dispatch, replies are sent back to the originating port.
    virtual void handleSystemExclusive(Port* port, const uint8_t* buffer, uint32_t len) {}

    virtual bool handleSend(Packet* packet) {
      return false;
    }

  private:
    struct {
      struct {
        uint8_t* buffer{};
        uint32_t length{};
        bool     appending{};

        void reset() {
          length    = 0;
          appending = false;
        }
      } in;

      struct {
        Port*    port{};
        uint8_t* buffer{};
        uint32_t length{};
        uint32_t position{};

        void reset() {
          length   = 0;
          position = 0;
        }
      } out;
    } _sysex;

    bool storeSystemExclusive(Packet* packet) {
      switch (packet->codeIndex) {
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

          // Used in the middle of a SysEx packet stream to transport a single byte instead of three.
          if (_sysex.in.length + 1 > _sysexSize) {
            statistics.input.error++;
            _sysex.in.reset();
            return false;
          }

          _sysex.in.buffer[_sysex.in.length++] = packet->data[0];
          return false;

        // Start of a new SysEx stream, or append data to the current stream.
        case Packet::CodeIndex::SystemExclusiveStart:
          // Not enough space to store the stream.
          if (_sysex.in.length + 3 > _sysexSize) {
            statistics.input.error++;
            _sysex.in.reset();
            return false;
          }

          if (!_sysex.in.appending) {
            _sysex.in.length = 0;

            // Must be the start of a SysEx.
            if (packet->data[0] != static_cast<uint8_t>(Packet::Status::SystemExclusive)) {
              statistics.input.error++;
              return false;
            }

            _sysex.in.appending = true;
          }

          _sysex.in.buffer[_sysex.in.length++] = packet->data[0];
          _sysex.in.buffer[_sysex.in.length++] = packet->data[1];
          _sysex.in.buffer[_sysex.in.length++] = packet->data[2];
          return false;

        // End of SysEx stream with various lengths.
        case Packet::CodeIndex::SystemExclusiveEnd1:
          // Invalid 'End' packet
          if (packet->data[0] != static_cast<uint8_t>(Packet::Status::SystemExclusiveEnd)) {
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

          // Not enough space to store the stream.
          if (_sysex.in.length + 1 > _sysexSize) {
            statistics.input.error++;
            _sysex.in.reset();
            return false;
          }

          _sysex.in.buffer[_sysex.in.length++] = packet->data[0];
          break;

        case Packet::CodeIndex::SystemExclusiveEnd2:
          // Invalid 'End' packet.
          if (packet->data[1] != static_cast<uint8_t>(Packet::Status::SystemExclusiveEnd)) {
            statistics.input.error++;
            _sysex.in.reset();
            return false;
          }

          // Not enough space to store the stream.
          if (_sysex.in.length + 2 > _sysexSize) {
            statistics.input.error++;
            _sysex.in.reset();
            return false;
          }

          // Single 'End' packet.
          if (!_sysex.in.appending) {
            _sysex.in.length = 0;

            // Must be an 'empty' SysEx.
            if (packet->data[0] != static_cast<uint8_t>(Packet::Status::SystemExclusive))
              return false;
          }

          _sysex.in.buffer[_sysex.in.length++] = packet->data[0];
          _sysex.in.buffer[_sysex.in.length++] = packet->data[1];
          break;

        case Packet::CodeIndex::SystemExclusiveEnd3:
          // Invalid 'End' packet.
          if (packet->data[2] != static_cast<uint8_t>(Packet::Status::SystemExclusiveEnd)) {
            statistics.input.error++;
            _sysex.in.reset();
            return false;
          }

          // Not enough space to store the stream.
          if (_sysex.in.length + 3 > _sysexSize) {
            statistics.input.error++;
            _sysex.in.reset();
            return false;
          }

          // Single 'End' packet.
          if (!_sysex.in.appending) {
            _sysex.in.length = 0;

            // Must be a 'one byte' SysEx.
            if (packet->data[0] != static_cast<uint8_t>(Packet::Status::SystemExclusive))
              return false;
          }

          _sysex.in.buffer[_sysex.in.length++] = packet->data[0];
          _sysex.in.buffer[_sysex.in.length++] = packet->data[1];
          _sysex.in.buffer[_sysex.in.length++] = packet->data[2];
          break;

        default:
          statistics.input.error++;
          _sysex.in.reset();
          return false;
      }

      // Always return 'SystemExclusive' as type.
      _sysex.in.appending = false;
      packet->data[0]     = static_cast<uint8_t>(Packet::Status::SystemExclusive);
      return true;
    }
  };
};
