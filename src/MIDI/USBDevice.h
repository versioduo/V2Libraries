#pragma once

#include "Packet.h"
#include "Port.h"
#include <V2Base.h>

namespace V2MIDI {
  class USBDevice : public Port, public V2Base::USBDevice {
  public:
    constexpr USBDevice() : Port{"usb"} {}

    auto send(Packet& midi) -> bool override {
      if (V2Base::USBDevice::send((const uint8_t*)&midi)) {
        statistics.output.packet++;
        return true;
      }

      return false;
    }

    auto receive(Packet& midi) -> bool override {
      if (V2Base::USBDevice::receive((uint8_t*)&midi)) {
        statistics.input.packet++;
        return true;
      };

      return false;
    }
  };
}
