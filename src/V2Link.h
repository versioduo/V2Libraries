// V2 Link devices are full-duplex RS422 serial lines, the baud rate is 3 Mhz.
//
// The Plug connects the device to the parent device, the socket connects
// the children devices. Up to 16 devices can be daisy-chained.

#pragma once
#include <Arduino.h>
#include <V2MIDI.h>

class V2Link {
public:
  class Packet {
  public:
    enum class Type : uint8_t {
      MIDI,
      Pulse,
      Number,
    };

    // Solenoid pulse:
    //    4 bit: port
    //   12 bit: watts
    //   12 bit: seconds
    //    1 bit: fade in
    //    1 bit: fade out
    struct Pulse {
      uint8_t port{};
      float   watts{};
      float   seconds{};
      bool    fadeIn{};
      bool    fadeOut{};
    };

    // LE bit order: [address | type]
    Type    type : 4 {};
    uint8_t address : 4 {};
    union {
      V2MIDI::Packet         midi;
      std::array<uint8_t, 4> data{};
    };

    constexpr Packet() = default;
    constexpr Packet(uint8_t address, const V2MIDI::Packet& midi) : address(address), type(Packet::Type::MIDI), midi(midi) {}

    auto getPulse(Pulse& pulse) const {
      pulse.port    = data[0] & 0x0f;
      pulse.fadeIn  = data[0] & (1 << 4);
      pulse.fadeOut = data[0] & (1 << 5);

      {
        auto map{uint16_t((data[1] >> 4) << 8)};
        map |= data[2];
        auto fraction{float(map) / 4095.f};
        pulse.watts = 100.f * powf(fraction, 3);
      }
      {
        auto map{uint16_t((data[1] & 0x0f) << 8)};
        map |= data[3];
        auto fraction{float(map) / 4095.f};
        pulse.seconds = 100.f * powf(fraction, 8);
      }
    }

    auto setPulse(const Packet::Pulse& pulse) {
      type    = Packet::Type::Pulse;
      data[0] = pulse.port & 0x0f;
      if (pulse.fadeIn)
        data[0] |= 1 << 4;
      if (pulse.fadeOut)
        data[0] |= 1 << 5;

      {
        auto watts{pulse.watts};
        if (watts > 100.f)
          watts = 100;

        auto fraction{watts / 100.f};
        auto map{uint16_t(powf(fraction, 1.f / 3.f) * 4095.f)};
        data[1] = (map >> 8) << 4;
        data[2] = map & 0xff;
      }
      {
        float seconds{pulse.seconds};
        if (seconds > 100.f)
          seconds = 100;

        auto fraction{seconds / 100.f};
        auto map{uint16_t(powf(fraction, 1.f / 8.f) * 4095.f)};
        data[1] |= map >> 8;
        data[3] = map & 0xff;
      }
    }

    auto number() const -> uint32_t {
      uint32_t number{};
      auto     bytes{(uint8_t*)&number};
      std::copy(data.begin(), data.end(), bytes);
      return number;
    }

    auto number(uint32_t number) -> void {
      type = Packet::Type::Number;
      auto bytes{(uint8_t*)&number};
      std::copy(bytes, bytes + 4, data.begin());
    }
  };
  static_assert(sizeof(Packet) == 5);

  class Port : public V2MIDI::Port {
  public:
    struct Counter {
      uint32_t pulse{};
      uint32_t number{};
      uint32_t packet{};
      uint32_t error{};
    };

    struct Counters {
      Counter input;
      Counter output;
    } counter;

    Port() = delete;
    constexpr Port(Uart* uart, uint8_t pinTx, const std::string name) : V2MIDI::Port{name}, _uart(uart), _pinTx(pinTx) {}

    auto begin() {
      _uart->begin(3000000);
      _uart->setTimeout(1);

      if (_pinTx > 0) {
        pinMode(_pinTx, OUTPUT);
        digitalWrite(_pinTx, HIGH);
      }
    }

    auto idle() const {
      return !_active;
    }

    auto receive(Packet& p) -> bool {
      if (_uart->available() == 0)
        return false;

      _usec = micros();

      // Drop partial messages which don't complete in time.
      if (_uart->available() < 5) {
        if (_timeoutUsec == 0)
          _timeoutUsec = micros();

        if ((unsigned long)(micros() - _timeoutUsec) > 100) {
          while (_uart->available())
            _uart->read();

          _timeoutUsec = 0;
          counter.input.error++;
        }

        return false;
      }

      _timeoutUsec = 0;
      _uart->readBytes((uint8_t*)&p, 5);
      counter.input.packet++;
      switch (p.type) {
        case Packet::Type::MIDI:
          statistics.input.packet++;
          break;

        case Packet::Type::Pulse:
          counter.input.pulse++;
          break;

        case Packet::Type::Number:
          counter.input.number++;
          break;
      }
      return true;
    }

    auto send(const Packet& p) -> bool {
      if (!_active) {
        if (_pinTx > 0)
          digitalWrite(_pinTx, HIGH);

        _active = true;
      }

      _usec = micros();

      if (_uart->availableForWrite() < 5)
        return false;

      _uart->write((const uint8_t*)&p, 5);
      counter.output.packet++;
      switch (p.type) {
        case Packet::Type::MIDI:
          statistics.output.packet++;
          break;

        case Packet::Type::Pulse:
          counter.output.pulse++;
          break;

        case Packet::Type::Number:
          counter.output.number++;
          break;
      }

      return true;
    }

    // Used for replies during V2Device dispatch, it can only send to address 0.
    auto send(V2MIDI::Packet& midi) -> bool override {
      return send(Packet(0, midi));
    }

    auto powerDown() {
      if (!_active)
        return;

      // Wait for at least 100ms to flush a large outgoing buffer.
      if ((unsigned long)(micros() - _usec) < 100 * 1000)
        return;

      if (_pinTx > 0)
        digitalWrite(_pinTx, LOW);

      _active = false;
    }

  private:
    Uart*         _uart{};
    const uint8_t _pinTx;
    bool          _active{};
    unsigned long _timeoutUsec{};
    unsigned long _usec{};
  };

  constexpr V2Link(Port* port, Port* socket, Port* socketNode = nullptr) : plug(port), socket(socket), socketNode(socketNode) {}

  auto begin() {
    if (plug)
      plug->begin();

    if (socket)
      socket->begin();

    if (socketNode)
      socketNode->begin();
  }

  auto loop() {
    if (plug) {
      if (plug->receive(_link)) {
        if (_link.address > 0) {
          // Forward message from a parent device to a child device.
          if (socket) {
            _link.address--;
            socket->send(_link);
          }
        } else {
          receivePlug(_link);
        }
      }

      plug->powerDown();
    }

    if (socket) {
      if (socket->receive(_link)) {
        // Forward message from a child device towards the parent device, stop after too many hops.
        if (_link.address < 0x0f) {
          _link.address++;
          if (plug)
            plug->send(_link);
          receiveSocket(_link);
        }
      }

      socket->powerDown();
    }

    if (socketNode) {
      if (socketNode->receive(_link))
        receiveSocketNode(_link);

      socketNode->powerDown();
    }
  }

  auto idle() const -> bool {
    if (plug && !plug->idle())
      return false;

    if (socket && !socket->idle())
      return false;

    if (socketNode && !socketNode->idle())
      return false;

    return true;
  }

  Port* plug{};
  Port* socket{};
  Port* socketNode{};

private:
  Packet _link;

  virtual auto receivePlug(Packet& packet) -> void {}
  virtual auto receiveSocket(Packet& packet) -> void {}
  virtual auto receiveSocketNode(Packet& packet) -> void {}
};
