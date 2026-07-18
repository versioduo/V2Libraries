#pragma once
#include <string>

namespace V2MIDI {
  class Transport {
  public:
    Transport() = delete;
    Transport(const std::string& name) : name{name} {}

    const std::string name;
    virtual auto      receive(Packet& midi) -> bool    = 0;
    virtual auto      send(const Packet& midi) -> bool = 0;
  };
}
