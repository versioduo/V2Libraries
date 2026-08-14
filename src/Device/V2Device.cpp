#include "V2Device.h"
#include <V2Base.h>

// This is only initialized after a cold startup when the memory is undefined.
// A reset/reboot will not overwrite the data; it is retained across reset/reboot
// cycles.
// The "noinit" section needs to be outside of the "bss" section; it requires
// explicit support from the linker script to be effective.
static class BootData {
public:
  BootData() {
    if (_magic == 0x8f734e41)
      return;

    clear();
    _magic = 0x8f734e41;
  }

  void clear() {
    usb.ports.enableAccess = false;
  }

  // The number of MIDI ports to export to the host
  struct {
    struct {
      bool enableAccess;
    } ports;
  } usb;

private:
  uint32_t _magic;
} bootData __attribute__((section(".noinit")));

bool V2Device::readEEPROM(bool dryrun) {
  struct EEPROM* eeprom = (struct EEPROM*)V2Base::Memory::EEPROM::getStart();
  // Check our magic, all bytes are 0xff after chip erase.
  if (eeprom->header.magic != _eeprom.header.magic)
    return false;

  if (eeprom->header.size <= sizeof(struct EEPROM::Header))
    return false;

  if (!dryrun)
    memcpy(&_eeprom, eeprom, min(eeprom->header.size, sizeof(_eeprom)));

  if (_eeprom.usb.name[0] != '\0')
    usb.name = _eeprom.usb.name;

  // Device-specific section.
  if (eeprom->local.magic != usb.pid || !configuration.data || eeprom->local.size == 0)
    return true;

  const void* data = (const uint8_t*)V2Base::Memory::EEPROM::getStart() + eeprom->header.size;

  // Try to import an older version of the configuration.
  if (eeprom->local.version != configuration.version) {
    if (!dryrun)
      handleEEPROM(eeprom->local.version, data, eeprom->local.size);

    return true;
  }

  if (!dryrun)
    memcpy(configuration.data, data, min(eeprom->local.size, configuration.size));

  return true;
}

void V2Device::begin() {
  V2MIDI::Device::begin();
  usb.midi.begin();

  // The priority needs to be lower than the SERCOM priorities.
  _ledTimer.begin(std::bind(&V2LED::Basic::tick, &led));
  _ledTimer.setPriority(3);

  if (V2Base::Memory::Flash::UserPage::update()) {
    // Reboot to enable the new settings.
    delay(100);
    V2Base::Memory::Firmware::reboot();
  }

  _boot.id = V2Base::Cryptography::Random::read();

  // Do not block in GetAll(), it takes ~80ms.
  V2Base::Memory::Firmware::calculateHash(V2Base::Memory::Firmware::getStart(), V2Base::Memory::Firmware::getSize(), _firmware.hash);

  // Read a possible config from the previous boot cycle.
  if (bootData.usb.ports.enableAccess)
    usb.ports.enableAccess = true;

  bootData.clear();

#ifdef PIN_REVISION_BITS
  // The revision number is composed of pins which are either floating or
  // connected to ground. A ground connection represents a logical high.
  {
    for (uint8_t i = 0; i < PIN_REVISION_BITS; i++)
      pinMode(PIN_REVISION + i, INPUT_PULLUP);

    for (uint8_t i = 0; i < PIN_REVISION_BITS; i++) {
      if (digitalRead(PIN_REVISION + i))
        continue;

      system.revision |= 1 << i;
    }

    for (uint8_t i = 0; i < PIN_REVISION_BITS; i++)
      pinMode(PIN_REVISION + i, INPUT);
  }
#endif

  readEEPROM();
  handleInit();

  usb.midi.setVendor(metadata.vendor);

  // Set USB device name, the default is provided by the board package, the metadata
  // provides a product name, a custom name might be stored in the EEPROM.
  const char* name = usb.name ? usb.name : metadata.product;
  usb.midi.setName(name);

  if (system.configure && memcmp(system.configure, "https://", 8) == 0)
    usb.midi.setConfigureURL(system.configure, name);

  // Set USB MIDI ports.
  if (usb.ports.enableAccess)
    usb.ports.current = (usb.ports.access > 0) ? usb.ports.access : 16;
  else if (_eeprom.usb.ports > 0)
    usb.ports.current = _eeprom.usb.ports;
  else if (usb.ports.standard > 0)
    usb.ports.current = usb.ports.standard;

  if (usb.ports.current > 1)
    usb.midi.setPorts(usb.ports.current);

  // Operating systems/services/apps get confused if the number
  // of ports changes between device connections; some hang, some
  // don't probe the device again and ignore the new number of ports.
  //
  // To work around it, let the USB ID depend on the number of ports.
  const uint16_t vid = _eeprom.usb.vid > 0 ? _eeprom.usb.vid : usb.vid;
  const uint16_t pid = _eeprom.usb.pid > 0 ? _eeprom.usb.pid : usb.pid;
  usb.midi.setVIDPID(vid, pid + usb.ports.current - 1);

  usb.midi.setVersion(V2DeviceMetadata.version);
  usb.midi.attach();

  // Sleep mode IDLE, wait for interrupts.
  V2Base::Power::setSleepMode(V2Base::Power::Mode::Idle);
}

void V2Device::reset() {
  led.reset();
  resetSystemExclusive();
  handleReset();
}

void V2Device::loop() {
  led.loop();
  loopSystemExclusive();
  handleLoop();
}

// Reply with message to indicate that we are ready for the next packet.
void V2Device::sendFirmwareStatus(V2MIDI::Transport* transport, const char* status) {
  uint8_t* reply = getSystemExclusiveBuffer();
  uint32_t len   = 0;

  // 0x7d == SysEx research/private ID
  reply[len++] = (uint8_t)V2MIDI::Packet::Status::SystemExclusive;
  reply[len++] = 0x7d;

  JsonDocument json;
  JsonObject   jsonDevice = json["com.versioduo.device"].to<JsonObject>();
  jsonDevice["token"]     = _boot.id;
  JsonObject jsonFirmware = jsonDevice["firmware"].to<JsonObject>();
  jsonFirmware["status"]  = status;
  len += serializeJson(json, (char*)reply + len, 1024);

  reply[len++] = (uint8_t)V2MIDI::Packet::Status::SystemExclusiveEnd;
  sendSystemExclusive(transport, len);
}

static int8_t utf8Codepoint(const uint8_t* utf8, uint32_t* codepointp) {
  uint32_t codepoint;
  int8_t   len;

  if (utf8[0] < 0x80)
    len = 1;
  else if ((utf8[0] & 0xe0) == 0xc0)
    len = 2;
  else if ((utf8[0] & 0xf0) == 0xe0)
    len = 3;
  else if ((utf8[0] & 0xf8) == 0xf0)
    len = 4;
  else if ((utf8[0] & 0xfc) == 0xf8)
    len = 5;
  else if ((utf8[0] & 0xfe) == 0xfc)
    len = 6;
  else
    return -1;

  switch (len) {
    case 1:
      codepoint = utf8[0];
      break;

    case 2:
      codepoint = utf8[0] & 0x1f;
      break;

    case 3:
      codepoint = utf8[0] & 0x0f;
      break;

    case 4:
      codepoint = utf8[0] & 0x07;
      break;

    case 5:
      codepoint = utf8[0] & 0x03;
      break;

    case 6:
      codepoint = utf8[0] & 0x01;
      break;
  }

  for (int8_t i = 1; i < len; i++) {
    if ((utf8[i] & 0xc0) != 0x80)
      return -1;

    codepoint <<= 6;
    codepoint |= utf8[i] & 0x3f;
  }

  *codepointp = codepoint;
  return len;
}

// Escape unicode to fit into a 7 bit byte stream.
static uint32_t escapeJSON(const uint8_t* jsonBuffer, uint32_t jsonLen, uint8_t* buffer, uint32_t size) {
  uint32_t bufferLen = 0;

  for (uint32_t i = 0; i < jsonLen; i++) {
    if (jsonBuffer[i] > 0x7f) {
      uint32_t codepoint;
      uint8_t  len = utf8Codepoint(jsonBuffer + i, &codepoint);
      if (len < 0)
        continue;

      // Advance the additional UTF8 characters for this codepoint.
      i += len - 1;

      if (codepoint < 0xffff) {
        if (bufferLen + 7 > size)
          return 0;

        bufferLen += sprintf((char*)buffer + bufferLen, "\\u%04x", codepoint);

      } else {
        if (bufferLen + 13 > size)
          return 0;

        codepoint -= 0x10000;
        uint16_t surrogate1 = (codepoint >> 10) + 0xd800;
        uint16_t surrogate2 = (codepoint & 0x3ff) + 0xdc00;
        bufferLen += sprintf((char*)buffer + bufferLen, "\\u%04x\\u%04x", surrogate1, surrogate2);
      }

    } else {
      if (bufferLen >= size)
        return 0;

      buffer[bufferLen++] = jsonBuffer[i];
    }
  }

  return bufferLen;
}

void addStatistics(JsonObject json, const V2MIDI::Transport::Statistics& s) {
  auto counter{[](JsonObject j, const V2MIDI::Transport::Counter& c) {
    j["packet"] = c.packet;

    if (c.note > 0)
      j["note"] = c.note;

    if (c.noteOff > 0)
      j["noteOff"] = c.noteOff;

    if (c.aftertouch > 0)
      j["aftertouch"] = c.aftertouch;

    if (c.control > 0)
      j["control"] = c.control;

    if (c.program > 0)
      j["program"] = c.program;

    if (c.aftertouchChannel > 0)
      j["aftertouchChannel"] = c.aftertouchChannel;

    if (c.pitchbend > 0)
      j["pitchbend"] = c.pitchbend;

    if (c.system.exclusive > 0 || c.system.reset > 0 || c.system.clock.tick > 0) {
      auto system{j["system"].to<JsonObject>()};
      if (c.system.exclusive > 0)
        system["exclusive"] = c.system.exclusive;

      if (c.system.reset > 0)
        system["reset"] = c.system.reset;

      if (c.system.clock.tick > 0) {
        auto clock{system["clock"].to<JsonObject>()};
        clock["tick"] = c.system.clock.tick;
      }
    }

    if (c.error > 0)
      j["error"] = c.error;
  }};

  auto m{json["midi"].to<JsonObject>()};
  auto in{m["input"].to<JsonObject>()};
  counter(in, s.input);

  auto out{m["output"].to<JsonObject>()};
  counter(out, s.output);
}

// Send the current data as a SystemExclusive, JSON message.
void V2Device::sendReply(V2MIDI::Transport* transport) {
  uint8_t* reply = getSystemExclusiveBuffer();
  uint32_t len   = 0;

  // 0x7d == SysEx research/private ID
  reply[len++] = (uint8_t)V2MIDI::Packet::Status::SystemExclusive;
  reply[len++] = 0x7d;

  JsonDocument json;
  JsonObject   jsonDevice = json["com.versioduo.device"].to<JsonObject>();

  // Requests and replies contain the device's current bootID.
  jsonDevice["token"] = _boot.id;

  {
    JsonObject jsonMeta = jsonDevice["metadata"].to<JsonObject>();
    if (metadata.product)
      jsonMeta["product"] = metadata.product;

    if (metadata.description)
      jsonMeta["description"] = metadata.description;

    if (metadata.vendor)
      jsonMeta["vendor"] = metadata.vendor;

    if (metadata.home)
      jsonMeta["home"] = metadata.home;

    {
      char serial[33];
      usb.midi.readSerial(serial);
      jsonMeta["serial"] = serial;
    }

    jsonMeta["version"] = V2DeviceMetadata.version;
    exportMetadata(jsonMeta);
  }

  {
    JsonArray jsonLinks = jsonDevice["links"].to<JsonArray>();
    exportLinks(jsonLinks);
  }

  {
    JsonObject jsonHelp = jsonDevice["help"].to<JsonObject>();
    if (help.device)
      jsonHelp["device"] = help.device;

    if (help.configuration)
      jsonHelp["configuration"] = help.configuration;
  }

  {
    auto jsonSystem{jsonDevice["system"].to<JsonObject>()};
    if (usb.name)
      jsonSystem["name"] = usb.name;

    {
      auto j{jsonSystem["boot"].to<JsonObject>()};
      j["uptime"] = float(millis()) / 1000.f;
      j["id"]     = _boot.id;
    }

    {
      auto j{jsonSystem["connection"].to<JsonObject>()};
      j["transport"] = transport->name;
      addStatistics(j, statistics);
      exportSystem(jsonSystem);
    }

    {
      auto j{jsonSystem["firmware"].to<JsonObject>()};
      if (system.download)
        j["download"] = system.download;

      if (system.configure)
        j["configure"] = system.configure;

      j["id"]    = V2DeviceMetadata.id;
      j["board"] = V2DeviceMetadata.board;
      j["hash"]  = _firmware.hash;
      j["start"] = V2Base::Memory::Firmware::getStart();
      j["size"]  = V2Base::Memory::Firmware::getSize();
    }

    {
      auto jsonHardware{jsonSystem["hardware"].to<JsonObject>()};
      {
        // The end of the bootloader contains an array of four offsets/pointers.
        const uint32_t* info = (uint32_t*)V2Base::Memory::Firmware::getStart() - 4;

        // The first entry is the location of our metadata.
        const char*  metadata = (const char*)info[0];
        JsonDocument jsonMetadata;
        if (deserializeJson(jsonMetadata, metadata))
          return;

        JsonObject jsonBootloader = jsonMetadata["com.versioduo.bootloader"];
        if (!jsonBootloader)
          return;

        if (!jsonBootloader["board"])
          return;

        jsonHardware["board"] = jsonBootloader["board"];
      }

      if (system.revision > 0)
        jsonHardware["revision"] = system.revision;

      {
        auto j{jsonHardware["ram"].to<JsonObject>()};
        j["size"] = V2Base::Memory::RAM::getSize();
        j["free"] = V2Base::Memory::RAM::getFree();
      }

      {
        auto j{jsonHardware["flash"].to<JsonObject>()};
        j["size"] = V2Base::Memory::Flash::getSize();
      }

      {
        auto j{jsonHardware["eeprom"].to<JsonObject>()};
        j["size"] = V2Base::Memory::EEPROM::getSize();
        j["used"] = readEEPROM(true);
      }

      {
        auto jsonUsb{jsonHardware["usb"].to<JsonObject>()};
        {
          JsonObject j  = jsonUsb["connection"].to<JsonObject>();
          j["active"]   = usb.midi.connected();
          j["sequence"] = usb.midi.getConnectionSequence();
        }

        jsonUsb["vid"] = _eeprom.usb.vid > 0 ? _eeprom.usb.vid : usb.vid;
        jsonUsb["pid"] = _eeprom.usb.pid > 0 ? _eeprom.usb.pid : usb.pid;

        if (usb.ports.standard > 0) {
          auto j{jsonUsb["ports"].to<JsonObject>()};
          j["standard"] = usb.ports.standard;
          if (usb.ports.fixed)
            j["fixed"] = usb.ports.fixed;
          if (usb.ports.access > 0)
            j["access"] = usb.ports.access;
          j["current"] = usb.ports.current;
        }

        addStatistics(jsonUsb, usb.midi.statistics);
      }

      if (link) {
        auto statistics{[](JsonObject port, const V2Link::Port::Counters& c) {
          if (c.input.packet > 0 || c.output.packet > 0) {
            auto j{port["packet"].to<JsonObject>()};
            if (c.input.packet > 0)
              j["input"] = c.input.packet;

            if (c.output.packet > 0)
              j["output"] = c.output.packet;
          }

          if (c.input.pulse > 0 || c.output.pulse > 0) {
            auto j{port["pulse"].to<JsonObject>()};
            if (c.input.pulse > 0)
              j["input"] = c.input.pulse;

            if (c.output.pulse > 0)
              j["output"] = c.output.pulse;
          }

          if (c.input.number > 0 || c.output.number > 0) {
            auto j{port["number"].to<JsonObject>()};
            if (c.input.number > 0)
              j["input"] = c.input.number;

            if (c.output.number > 0)
              j["output"] = c.output.number;
          }

          if (c.input.error > 0 || c.output.error > 0) {
            auto j{port["error"].to<JsonObject>()};
            if (c.input.error > 0)
              j["input"] = c.input.error;

            if (c.output.error > 0)
              j["output"] = c.output.error;
          }
        }};

        if (link->plug) {
          auto j{jsonHardware["plug"].to<JsonObject>()};
          statistics(j, link->plug->counter);
          addStatistics(j, link->plug->statistics);
        }

        if (link->socket) {
          auto j{jsonHardware["socket"].to<JsonObject>()};
          statistics(j, link->socket->counter);
          addStatistics(j, link->socket->statistics);
        }

        if (link->socketNode) {
          auto j{jsonHardware["socketNode"].to<JsonObject>()};
          statistics(j, link->socketNode->counter);
          addStatistics(j, link->socketNode->statistics);
        }
      }

      if (serial) {
        auto j{jsonHardware["serial"].to<JsonObject>()};
        addStatistics(j, serial->statistics);
      }
    }
  }

  JsonArray settings = jsonDevice["settings"].to<JsonArray>();
  exportSettings(settings);

  {
    auto config{jsonDevice["configuration"].to<JsonObject>()};
    config["#usb"] = "USB Settings";

    auto jsonUsb{config["usb"].to<JsonObject>()};
    jsonUsb["#name"] = "Device Name";
    jsonUsb["name"]  = _eeprom.usb.name;
    jsonUsb["#vid"]  = "USB Vendor ID";
    jsonUsb["vid"]   = _eeprom.usb.vid;
    jsonUsb["#pid"]  = "USB Product ID";
    jsonUsb["pid"]   = _eeprom.usb.pid;

    if (usb.ports.standard > 0) {
      jsonUsb["#ports"] = "Number of MIDI ports";
      jsonUsb["ports"]  = _eeprom.usb.ports;
    }

    exportConfiguration(config);
  }

  JsonObject input = jsonDevice["input"].to<JsonObject>();
  exportInput(input);
  if (input.begin() == input.end())
    jsonDevice.remove("input");

  JsonObject output = jsonDevice["output"].to<JsonObject>();
  exportOutput(output);
  if (output.begin() == output.end())
    jsonDevice.remove("output");

  {
    uint8_t  jsonBuffer[_sysexSize];
    uint32_t jsonLen = serializeJson(json, (char*)jsonBuffer, _sysexSize);
    len += escapeJSON(jsonBuffer, jsonLen, reply + len, _sysexSize - len);
  }

  reply[len++] = (uint8_t)V2MIDI::Packet::Status::SystemExclusiveEnd;
  sendSystemExclusive(transport, len);
}

// Handle a SystemExclusive, JSON request from the host.
void V2Device::handleSystemExclusive(V2MIDI::Transport* transport, const uint8_t* buffer, uint32_t len) {
  if (len < 24)
    return;

  // 0x7d == SysEx prototype/research/private ID
  if (buffer[1] != 0x7d)
    return;

  // Handle only JSON messages.
  if (buffer[2] != '{' || buffer[len - 2] != '}')
    return;

  // Read incoming message.
  JsonDocument json;
  if (deserializeJson(json, buffer + 2, len - 1))
    return;

  // Only handle requests for our interface.
  JsonObject jsonDevice = json["com.versioduo.device"];
  if (!jsonDevice)
    return;

  if (jsonDevice["method"] == "getAll") {
    json.clear();
    sendReply(transport);
    return;
  }

  // Requests and replies contain the device's current bootID. The token prevents
  // devices from accepting messages intended for a different device, or messages
  // addressed to the same device but a different boot cycle.
  if (jsonDevice["token"] != _boot.id) {
    return;

  } else if (jsonDevice["method"] == "eraseConfiguration") {
    // Wipe the entire EEPROM area.
    V2Base::Memory::EEPROM::erase();
    V2Base::Memory::Firmware::reboot();
    return;

  } else if (jsonDevice["method"] == "switchChannel") {
    if (!jsonDevice["channel"].isNull())
      handleSwitchChannel(jsonDevice["channel"]);
    json.clear();
    sendReply(transport);
    return;

  } else if (jsonDevice["method"] == "reboot") {
    V2Base::Memory::Firmware::reboot();
    return;

  } else if (jsonDevice["method"] == "bootloader") {
    V2Base::Memory::Firmware::bootloader();
    return;

  } else if (jsonDevice["method"] == "rebootWithPorts") {
    bootData.usb.ports.enableAccess = true;
    V2Base::Memory::Firmware::reboot();
    return;

  } else if (jsonDevice["method"] == "writeConfiguration") {
    // Write the configuration the the EEPROM.

    // The data is enclosed in an object to prevent name clashes with the
    // calling convention.

    if (auto config{jsonDevice["configuration"]}; config) {
      if (auto jsonUsb{config["usb"]}; jsonUsb) {
        if (const char* n = jsonUsb["name"]; n) {
          if (strlen(n) > 1 && strlen(n) < 32) {
            usb.name = n;
            strcpy(_eeprom.usb.name, n);

          } else {
            usb.name = NULL;
            memset(_eeprom.usb.name, 0, sizeof(_eeprom.usb.name));
          }
        }

        if (!jsonUsb["vid"].isNull()) {
          uint16_t vid    = jsonUsb["vid"];
          _eeprom.usb.vid = vid;
        }

        if (!jsonUsb["pid"].isNull()) {
          uint16_t pid    = jsonUsb["pid"];
          _eeprom.usb.pid = pid;
        }

        if (!jsonUsb["ports"].isNull()) {
          uint8_t p = jsonUsb["ports"];
          if (p <= 16)
            _eeprom.usb.ports = p;
        }
      }

      // Device-specific section.
      if (configuration.size > 0)
        importConfiguration(config);

      writeConfiguration();
    }

    // Reply with the updated configuration.
    json.clear();
    sendReply(transport);
    return;

  } else if (jsonDevice["method"] == "writeFirmware") {
    // The data in enclosed in an object to prevent name clashes with the
    // calling convention.
    JsonObject firmware = jsonDevice["firmware"];
    if (firmware) {
      uint32_t offset = firmware["offset"];
      if (offset % V2Base::Memory::Flash::getBlockSize() != 0) {
        sendFirmwareStatus(transport, "invalidOffset");
        return;
      }

      const char* data = firmware["data"];
      union {
        uint32_t block[V2Base::Memory::Flash::getBlockSize() / sizeof(uint32_t)];
        uint8_t  bytes[V2Base::Memory::Flash::getBlockSize()];
      };
      uint32_t blockLen = V2Base::Text::Base64::decode((const uint8_t*)data, bytes);

      memset(bytes + blockLen, 0xff, V2Base::Memory::Flash::getBlockSize() - blockLen);
      led.brightness(0.3);
      V2Base::Memory::Firmware::Secondary::writeBlock(offset, block);
      led.brightness(0.1);

      // The final message contains our hash over the entire image.
      const char* hash = firmware["hash"];
      if (hash) {
        V2Base::Memory::Firmware::Secondary::copyBootloader();

        if (V2Base::Memory::Firmware::Secondary::verify(offset + blockLen, hash)) {
          sendFirmwareStatus(transport, "success");

          // Flush system exclusive message, loop() is no longer called.
          uint32_t usec = V2Base::getUsec();
          for (;;) {
            if (loopSystemExclusive() == 0)
              break;

            if ((uint32_t)(V2Base::getUsec() - usec) > 100 * 1000)
              break;

            yield();
          }

          // Give the host time to process the message before the USB device disconnects.
          led.brightness(1);
          delay(100);

          // System reset with the new firmware image.
          V2Base::Memory::Firmware::Secondary::activate();
        }

        sendFirmwareStatus(transport, "hashMismatch");

      } else {
        sendFirmwareStatus(transport, "success");
      }
    }

    return;
  }
}

void V2Device::writeConfiguration() {
  // Common section.
  _eeprom.local.magic   = usb.pid;
  _eeprom.local.version = configuration.version;
  _eeprom.local.size    = configuration.size;
  V2Base::Memory::EEPROM::write(0, (const uint8_t*)&_eeprom, sizeof(_eeprom));

  // Device-specific section.
  if (configuration.size > 0)
    V2Base::Memory::EEPROM::write(sizeof(_eeprom), (const uint8_t*)configuration.data, configuration.size);
}

bool V2Device::idle() {
  if (!usb.midi.idle())
    return false;

  return true;
}
