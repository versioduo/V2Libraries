#pragma once

namespace V2MIDI {
  class Transport {
  public:
    virtual auto receive(Packet& midi) -> bool    = 0;
    virtual auto send(const Packet& midi) -> bool = 0;
  };
}
