// Generic I2S Audio Codec, two channels, 48 kHz. The samples sources are
// registered as V2Audio::Channel, and called from interrupt context.

#pragma once
#include <Adafruit_ZeroDMA.h>
#include <V2Base.h>
#include <V2Music.h>

namespace V2Audio {
  class Codec : public V2Base::I2SInterface {
  public:
    static constexpr uint8_t nChannels = 2;

    // Single stream of 32-bit floating point samples. Registered registerChannel() to
    // supply the sample data. It is called from the DMA interrupt to fill the buffer.
    class Channel {
    public:
      virtual float getSample() = 0;
    };

    Codec(uint8_t pinSCK, uint8_t pinFS, uint8_t pinSD, uint8_t pinMCK) : V2Base::I2SInterface(pinSCK, pinFS, pinSD, pinMCK) {}

    // The instance needs to be registered with the DMA engine to update the sample data:
    //   Codec.begin([](Adafruit_ZeroDMA *dma) { Codec.fillNextBuffer(); });
    void begin(void (*dmaCallback)(Adafruit_ZeroDMA* dma)) {
      V2Base::I2SInterface::begin();

      _dma.setTrigger(I2S_DMAC_ID_TX_0);
      _dma.setAction(DMA_TRIGGER_ACTON_BEAT);
      _dma.allocate();
      _dma.loop(true);
      _dma.setCallback(dmaCallback);

      // Allocate a DMA descriptor for every one of the two buffers. The DMA engine
      // will cycle through the buffers until the request is aborted.
      for (uint8_t i = 0; i < V2Base::countof(_buffers); i++) {
        DmacDescriptor* desc = _dma.addDescriptor((void*)_buffers[i].samples,
                                                  (void*)(&I2S->TXDATA.reg),
                                                  _nSamples * nChannels,
                                                  DMA_BEAT_SIZE_WORD,
                                                  true,
                                                  false);

        // Enable the interrupt at the end of the descriptor / block transfer.
        desc->BTCTRL.bit.BLOCKACT = DMA_BLOCK_ACTION_INT;
      }

      adjustSamplerate(0);
    }

    // Enable / disable the power supply.
    virtual bool handlePower(bool on = true) = 0;

    // Enable / disable the codec hardware.
    virtual bool handleEnable(bool on = true) = 0;

    void registerChannel(uint8_t ch, Channel* channel) {
      _channels[ch] = channel;
    }

    void reset() {
      _dma.abort();
      V2Base::I2SInterface::reset();

      handleEnable(false);
      handlePower(false);

      _running = false;
      _index   = 0;
      adjustSamplerate(0);

      for (uint8_t ch = 0; ch < nChannels; ch++)
        _enabled[ch] = false;

      for (uint8_t i = 0; i < V2Base::countof(_buffers); i++)
        _buffers[i] = {};
    }

    void adjustSamplerate(float cents) {
      _frequency = V2Music::Frequency::adjustFrequency(getSamplerate(), cents);
    }

    float getFrequency() const {
      return _frequency;
    }

    bool isChannelEnabled(uint8_t channel) const {
      return _enabled[channel];
    }

    bool enableChannel(uint8_t channel) {

      if (!_channels[channel])
        return false;

      if (!handlePower()) {
        reset();
        return false;
      }

      if (!_running) {
        if (!handleEnable())
          return false;

        // Fill and stream the first buffer.
        _index = 0;
        fillBuffer();
        _dma.startJob();

        // Fill the second buffer. It will be transmitted by the DMA engine cycling through
        // the descriptors. The interrupt for the completed first buffer will fill the first
        // buffer again, and so on.
        fillNextBuffer();
        _running = true;
      }

      _enabled[channel] = true;
      return true;
    }

    void disableChannel(uint8_t channel) {
      _enabled[channel] = false;

      // Play silence.
      for (uint8_t i = 0; i < V2Base::countof(_buffers); i++)
        for (uint32_t sample = 0; sample < _nSamples; sample++)
          _buffers[i].samples[sample].channels[channel] = 0;
    }

    // Fill the current buffer.
    void fillBuffer() {
      const uint32_t usec = V2Base::getUsec();

      for (uint8_t ch = 0; ch < nChannels; ch++) {
        if (!_channels[ch])
          continue;

        if (!isChannelEnabled(ch))
          continue;

        for (uint32_t i = 0; i < _nSamples; i++)
          _buffers[_index].samples[i].channels[ch] = _channels[ch]->getSample() * (float)INT32_MAX;
      }

      _runUsec = V2Base::getUsecSince(usec);
    }

    // Switch to the next buffer and fill it.
    void fillNextBuffer() {
      _index++;
      if (_index == V2Base::countof(_buffers))
        _index = 0;

      // We are calculating samples in interrupt context for longer periods of
      // time. Allow other interrupts to do their tasks.
      __enable_irq();

      fillBuffer();
    }

    // Return the fraction of the time used to calculate the samples. Multiplied by
    // 100 it is an estimate in percent of the CPU usage.
    float getLoad() {
      const float runSec    = _runUsec / (1000.f * 1000.f);
      const float bufferSec = _nSamples / getSamplerate();
      return runSec / bufferSec;
    }

  protected:
    static constexpr uint16_t _nSamples = 64;

  private:
    bool _running{};

    // Cent-adjusted samplerate of the codec.
    float _frequency{};

    // Double-buffer of sample data, streamed by the DMA engine.
    struct {
      struct {
        int32_t channels[nChannels];
      } samples[_nSamples];
    } _buffers[2]{};

    // Currently used buffer.
    uint8_t _index{};

    // Registered callbacks to provide samples.
    Channel* _channels[nChannels]{};
    bool     _enabled[nChannels]{};

    Adafruit_ZeroDMA _dma;

    // Runtime of sample interrupt.
    uint32_t _runUsec{};
  };
};
