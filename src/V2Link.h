// V2 Link devices are full-duplex RS485 serial lines, the baud rate is 3 Mhz.
//
// The Plug connects the device to the parent device, the socket connects
// the children devices. Up to 16 devices can be daisy-chained.

#pragma once
#include <Arduino.h>
#include <V2MIDI.h>

class V2Link {
public:
  // Header:
  //   4 bit: target/child address,
  //   4 bit: message type
  class Packet : public V2MIDI::Transport {
  public:
    enum class Type : uint8_t { MIDI, Pulse };

    // Solenoid pulse:
    //   12 bit: watts
    //   12 bit: seconds
    //    1 bit: fade in
    //    1 bit: fade out
    struct Pulse {
      uint8_t port;
      float   watts;
      float   seconds;
      bool    fadeIn;
      bool    fadeOut;
    };

    Type getType() const {
      return static_cast<Type>(_data[0] & 0x0f);
    }

    uint8_t getAddress() const {
      return _data[0] >> 4;
    }

    bool receive(V2MIDI::Packet* midi) {
      if (getType() != Packet::Type::MIDI)
        return false;

      std::copy(_data + 1, _data + 5, midi->data());
      return true;
    }

    bool send(V2MIDI::Packet* midi) {
      _data[0] = uint8_t(Packet::Type::MIDI);
      std::copy(midi->data(), midi->data() + 4, _data + 1);
      return true;
    }

    void getPulse(Pulse* pulse) {
      pulse->port    = _data[1] & 0x0f;
      pulse->fadeIn  = _data[1] & (1 << 4);
      pulse->fadeOut = _data[1] & (1 << 5);

      {
        auto map{uint16_t((_data[2] >> 4) << 8)};
        map |= _data[3];
        auto fraction{float(map) / 4095.f};
        pulse->watts = 100.f * powf(fraction, 3);
      }
      {
        auto map{uint16_t((_data[2] & 0x0f) << 8)};
        map |= _data[4];
        const float fraction = float(map) / 4095.f;
        pulse->seconds       = 100.f * powf(fraction, 8);
      }
    }

    void setPulse(const Packet::Pulse* pulse) {
      _data[0] = (uint8_t)Packet::Type::Pulse;
      _data[1] = pulse->port & 0x0f;
      if (pulse->fadeIn)
        _data[1] |= 1 << 4;
      if (pulse->fadeOut)
        _data[1] |= 1 << 5;

      {
        auto watts{pulse->watts};
        if (watts > 100.f)
          watts = 100;

        auto fraction{watts / 100.f};
        auto map{uint16_t(powf(fraction, 1.f / 3.f) * 4095.f)};
        _data[2] = (map >> 8) << 4;
        _data[3] = map & 0xff;
      }
      {
        float seconds = pulse->seconds;
        if (seconds > 100.f)
          seconds = 100;

        auto fraction{seconds / 100.f};
        auto map{uint16_t(powf(fraction, 1.f / 8.f) * 4095.f)};
        _data[2] |= map >> 8;
        _data[4] = map & 0xff;
      }
    }

  private:
    friend class V2Link;
    uint8_t _data[5];
  };

  class Port : public V2MIDI::Transport {
  public:
    struct {
      uint32_t input{};
      uint32_t output{};
    } statistics;

    constexpr Port(Uart* uart, uint8_t pinTx = 0) : _uart(uart), _pinTx(pinTx) {}

    void begin() {
      _uart->begin(3000000);
      _uart->setTimeout(1);

      if (_pinTx > 0) {
        pinMode(_pinTx, OUTPUT);
        digitalWrite(_pinTx, HIGH);
      }
    }

    bool idle() const {
      return !_active;
    }

    bool receive(Packet* packet) {
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
        }

        return false;
      }

      _timeoutUsec = 0;
      _uart->readBytes(packet->_data, 5);
      statistics.input++;

      return true;
    }

    bool send(uint8_t address, Packet* packet) {
      if (!_active) {
        if (_pinTx > 0)
          digitalWrite(_pinTx, HIGH);

        _active = true;
      }

      _usec = micros();

      if (_uart->availableForWrite() < 5)
        return false;

      uint8_t header = address << 4;
      header |= packet->_data[0] & 0x0f;
      _uart->write(header);
      _uart->write(packet->_data + 1, 4);
      statistics.output++;

      return true;
    }

    bool receive(V2MIDI::Packet* midi) {
      return false;
    }

    bool send(V2MIDI::Packet* midi) {
      Packet packet;
      packet._data[0] = (uint8_t)Packet::Type::MIDI;
      memcpy(packet._data + 1, midi->data(), 4);
      return send(midi->getPort(), &packet);
    }

  private:
    friend class V2Link;
    Uart*         _uart;
    const uint8_t _pinTx;
    bool          _active{};
    unsigned long _timeoutUsec{};
    unsigned long _usec{};

    void powerDown() {
      if (!_active)
        return;

      // Wait for at least 100ms to flush a large outgoing buffer.
      if ((unsigned long)(micros() - _usec) < 100 * 1000)
        return;

      if (_pinTx > 0)
        digitalWrite(_pinTx, LOW);

      _active = false;
    }
  };

  constexpr V2Link(Port* port_, Port* socket_) : plug(port_), socket(socket_) {}

  void begin() {
    if (plug)
      plug->begin();

    if (socket)
      socket->begin();
  }

  void loop() {
    Packet packet;

    if (plug) {
      if (plug->receive(&packet)) {
        if (packet.getAddress() > 0) {
          // Forward message from a parent device to a child device.
          if (socket)
            socket->send(packet.getAddress() - 1, &packet);

        } else {
          receivePlug(&packet);
        }
      }

      plug->powerDown();
    }

    if (socket) {
      if (socket->receive(&packet)) {
        // Forward message from a child device towards the parent device, stop after
        // too many hops.
        if (plug && packet.getAddress() < 0x0f)
          plug->send(packet.getAddress() + 1, &packet);

        receiveSocket(&packet);
      }

      socket->powerDown();
    }
  }

  bool idle() const {
    if (plug && !plug->idle())
      return false;

    if (socket && !socket->idle())
      return false;

    return true;
  }

  Port* plug{};
  Port* socket{};

protected:
  virtual void receivePlug(Packet* packet) {}
  virtual void receiveSocket(Packet* packet) {}
};
