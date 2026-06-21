#pragma once

#include "Packet.h"
#include "Transport.h"
#include <V2Base.h>

namespace V2MIDI {
  class USBDevice : public Transport, public V2Base::USBDevice {
  public:
    auto send(const Packet* midi) -> bool override {
      return V2Base::USBDevice::send(midi->_data);
    }

    auto receive(Packet* midi) -> bool override {
      return V2Base::USBDevice::receive(midi->_data);
    }
  };
}
