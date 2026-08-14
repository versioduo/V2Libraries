#pragma once
#include "Packet.h"
#include <string>

namespace V2MIDI {
  class Transport {
  public:
    struct Counter {
      uint32_t packet{};
      uint32_t note{};
      uint32_t noteOff{};
      uint32_t aftertouch{};
      uint32_t control{};
      uint32_t program{};
      uint32_t aftertouchChannel{};
      uint32_t pitchbend{};
      struct {
        struct {
          uint32_t tick{};
        } clock;
        uint32_t exclusive{};
        uint32_t reset{};
      } system;
      uint32_t error{};
    };

    const std::string name;

    struct Statistics {
      Counter input;
      Counter output;
    } statistics;

    Transport() = delete;
    Transport(const std::string& name) : name{name} {}

    virtual auto receive(Packet& midi) -> bool {
      abort();
    }

    virtual auto send(Packet& midi) -> bool {
      abort();
    }
  };
}
