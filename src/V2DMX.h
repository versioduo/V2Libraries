#pragma once
#include <SPI.h>

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
  auto setChannels(uint16_t i, const uint8_t* data, uint16_t size) -> void;
  auto setChannel(uint16_t i, const uint8_t value) -> void;
  auto getChannel(uint16_t i) const -> uint8_t {
    return _channels.values[i];
  }

private:
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
};
