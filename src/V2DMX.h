#pragma once
#include <SPI.h>
#include <wiring_private.h>

class V2DMX {
public:
  constexpr V2DMX(uint8_t pin, SERCOM& sercom, SercomSpiTXPad padTX, EPioType pinFunc) :
    _sercom{.pin{pin}, .sercom{sercom}, .padTX{padTX}, .pinFunc{pinFunc}},
    _spi{&_sercom.sercom, _sercom.pin, _sercom.pin, _sercom.pin, _sercom.padTX, SERCOM_RX_PAD_3} {}

  auto begin() -> void;
  auto reset() -> void;

  // Encodes the DMA bit stream and fires a DMA transaction. If there
  // is a pending update and no current DMA transfer active, a new
  // transaction is started immediately.
  auto loop() -> void;

  // Set channel values and request an update.
  auto channels(uint16_t i, const uint8_t* data, uint16_t size) -> void;
  auto channel(uint16_t i, uint8_t value) -> void;
  auto channel(uint16_t i) const -> uint8_t;

private:
  // Break:            26 bits,         104 usec (minimum 92 usec)
  // Mark after Break:  3 bits,          12 usec (minimum 12 usec)
  // Start Code:        1 + 8 + 2 bits,  44 usec (value '0' @ 250kHz)
  // Slots:            Up to 512 slots 8 bit channel data
  //
  // The duration between breaks should be at least 1200 microseconds, the
  // equivalent of approximately 24 slots large message.
  static constexpr uint8_t dmaHeader[5]{
    0b00000000,
    0b00000000,
    0b00000000,
    0b00011100,
    0b11000000,
  };

  // 8 slots with value '0', stop bits, LSB first.
  static constexpr uint8_t dmaBlockInit[11]{
    0b00000000,
    0b00000110,
    0b00110000,
    0b10000000,
    0b00000001,
    0b00001100,
    0b01100000,
    0b00000000,
    0b00000011,
    0b00011000,
    0b11000000,
  };

  union DMABuffer {
    struct {
      uint8_t header[5]{};
      uint8_t blocks[64][11]{};
    };
    uint8_t bytes[5 + (11 * 64)];
  };

  union Channels {
    uint8_t values[512]{};
    uint8_t blocks[64][8];
  };

  struct {
    const uint8_t        pin{};
    SERCOM&              sercom;
    const SercomSpiTXPad padTX{};
    const EPioType       pinFunc{};
  } _sercom;

  SPIClass _spi;

  DMABuffer _dmaBuffer;
  Channels  _channels;
  uint16_t  _nChannels{};
  bool      _updateDMA{};
  bool      _transfer{};
  uint32_t  _transferUsec{};

  static void updateDMABlock(uint8_t block[11], uint8_t channel[8]);
};

inline auto V2DMX::begin() -> void {
  _spi.begin();

  // SPIClass.begin() applies the board config to all given pins. We just passed
  // one and the same pin to all of them, to make sure we do not touch unrelated
  // ones.
  pinPeripheral(_sercom.pin, _sercom.pinFunc);

  // The transaction will never stop.
  _spi.beginTransaction(SPISettings(250000, LSBFIRST, SPI_MODE0));
  reset();
}

inline auto V2DMX::reset() -> void {
  while (_spi.isBusy())
    yield();

  // Break + Mark + Start Code '0', LSB first.
  memcpy(&_dmaBuffer, dmaHeader, sizeof(dmaHeader));

  // 64 blocks of 11 bytes, each block containing 8 channel values:
  // one start bit, value '0', two stop bits.
  for (uint8_t i = 0; i < 64; i++)
    memcpy(_dmaBuffer.blocks[i], dmaBlockInit, sizeof(dmaBlockInit));

  memset(_channels.values, 0, sizeof(_channels.values));
  _nChannels = 0;

  _transferUsec = 0;
  _updateDMA    = true;
}

inline void V2DMX::updateDMABlock(uint8_t block[11], uint8_t channel[8]) {
  block[0]  = dmaBlockInit[0] | channel[0] << 1;
  block[1]  = dmaBlockInit[1] | channel[0] >> 7 | channel[1] << 4;
  block[2]  = dmaBlockInit[2] | channel[1] >> 4 | channel[2] << 7;
  block[3]  = dmaBlockInit[3] | channel[2] >> 1;
  block[4]  = dmaBlockInit[4] | channel[3] << 2;
  block[5]  = dmaBlockInit[5] | channel[3] >> 6 | channel[4] << 5;
  block[6]  = dmaBlockInit[6] | channel[4] >> 3;
  block[7]  = dmaBlockInit[7] | channel[5];
  block[8]  = dmaBlockInit[8] | channel[6] << 3;
  block[9]  = dmaBlockInit[9] | channel[6] >> 5 | channel[7] << 6;
  block[10] = dmaBlockInit[10] | channel[7] >> 2;
}

inline auto V2DMX::loop() -> void {
  if (_updateDMA) {
    // Refresh the DMA data.
    for (uint8_t i = 0; i < 64; i++) {
      // Do not needlessly update the untouched higher channels.
      if (i * 8 > _nChannels)
        break;

      updateDMABlock(_dmaBuffer.blocks[i], _channels.blocks[i]);
    }

    _updateDMA = false;
    _transfer  = true;
  }

  // Regularly send the DMX data regardless if something has changed; some
  // devices switch themselves off after a timeout.
  //
  // Do not transfer unchanged data in a loop though; we want to be able to
  // send new incoming updates as fast as possible (sync an incoming update with
  // the start of a new DMX data frame), and not needlessly wait for an unchanged
  // frame to finish transmitting.
  if (!_transfer && (uint32_t)(micros() - _transferUsec) < 400 * 1000)
    return;

  if (_spi.isBusy())
    return;

  _spi.transfer(&_dmaBuffer, nullptr, sizeof(DMABuffer), false);
  _transfer     = false;
  _transferUsec = micros();
}

inline auto V2DMX::channels(uint16_t i, const uint8_t* data, uint16_t size) -> void {
  if (i + size >= 512)
    return;

  // Remember the largest channel number in use to limit the DMA data update.
  if (_nChannels < i + size)
    _nChannels = i + size;

  memcpy(_channels.values + i, data, size);
  _updateDMA = true;
}

inline auto V2DMX::channel(uint16_t i, uint8_t value) -> void {
  channels(i, &value, 1);
}

inline auto V2DMX::channel(uint16_t i) const -> uint8_t {
  return _channels.values[i];
}
