#pragma once
#define ARDUINOJSON_USE_DOUBLE 0
#include <ArduinoJson.h>
#include <V2Base.h>
#include <V2LED.h>
#include <V2Link.h>
#include <V2MIDI.h>

class V2Device : public V2MIDI::Device {
public:
  // Device metadata stored in a global variable.
  struct MetadataFirmware {
    // Reverse-domain, unique device identifier (e.g. com.example.frobnicator).
    const std::string_view id;

    // The version will always be presented to the user as a simple decimal number.
    const uint16_t version;

    // The fully-qualified board name.
    const std::string_view board;

    // JSON object, it can be read from the offline firmware image. It needs to
    // be an embedded array not a pointer, to be able to retrieve its location
    // and export the offset to the end of the file.
    const char json[];
  };

  struct {
    // Few words, title line describing the function.
    const char* description{};

    // Human readable text, also used as USB strings.
    const char* vendor{};
    const char* product{};

    // Link to a website, including protocol prefix.
    const char* home{};

    const MetadataFirmware* firmware{};
  } metadata;

  // Help texts, paragraphs are separated by newline.
  struct {
    const char* device{};
    const char* configuration{};
  } help;

  struct {
    // Link to firmware the image updates, including the protocol prefix. It expects an
    // 'index.json' file at the location.
    const char* download{};

    // Link to the WebMIDI configuration site. The link is advertized over WebUSB.
    const char* configure{};

    // A specific hardware revision number encoded with GPIO pins connected to ground.
    uint8_t revision{};
  } system;

  // Custom USB IDs, initialized with the board specified values.
  struct {
    // Custom USB device name.
    std::string name;

    uint16_t vid{USB_VID};
    uint16_t pid{USB_PID};

    // Number of MIDI ports / virtual cables to access children devices.
    struct {
      // The default number of ports
      uint8_t standard{1};
      bool    fixed{};

      // The number of ports to enable when 'rebootWithPorts' is called
      // to gain access to the children devices.
      uint8_t access{};
      bool    enableAccess{};

      // The currently active number of ports.
      uint8_t current{1};
    } ports;

    V2MIDI::USBDevice midi{};
  } usb;

  // Export packet statistics for the MIDI ports.
  V2Link*                    link{};
  std::vector<V2MIDI::Port*> ports;

  // Built-in LED.
  V2LED::Basic led;

  // Local device-specific configuration which will be read and written to the EEPROM.
  struct {
    uint16_t version; // A different configuration version calls handleEEPROM() to possibly convert from.
    uint16_t size;
    void*    data;
  } configuration{};

  // The maximum system exclusive message size. It needs to carry at least the firmware
  // update packet of 8k bytes -> base64 encoded -> wrapped in a JSON object -> ~12kb.
  //
  // The default USB MIDI port 0.
  constexpr V2Device() : Device(0), led(PIN_LED_ONBOARD, &_ledTimer), _ledTimer(3, 1000) {}

  // Read the configuration from the EEPROM, initialize the bootup data which
  // might be carried over to the next reboot.
  auto begin() -> void;

  auto reset() -> void;
  auto loop() -> void;

  // Return if there is pending work, e.g. queued messages.
  auto idle() -> bool;

  // Wait for interrupts, goes into sleep mode IDLE. The system tick will wake it
  // up at least once every millisecond.
  auto sleep() -> void {
    V2Base::Power::sleep();
  }

  // Write the configuration to the EEPROM.
  auto writeConfiguration() -> void;

protected:
  // Called after reading the configuration from the EEPROM, before USB is initialized.
  virtual auto handleInit() -> void {}

  virtual auto handleReset() -> void {}

  virtual auto handleLoop() -> void {}

  // Called when updateConfiguration() is called. Parses the config and writes it to
  // the EEPROM.
  virtual auto importConfiguration(JsonObject json) -> void {}

  // The human readable device properties, e.g. name, vendor, product, description.
  virtual auto exportMetadata(JsonObject json) -> void {}

  // A list of external links to web applications using the device.
  virtual auto exportLinks(JsonArray json) -> void {}

  // The machine-readable device properties, e.g. state, statistics, firmware update URL.
  virtual auto exportSystem(JsonObject json) -> void {}

  // List of configuration objects pointing to data in the configuration and providing
  // metadata to create sections in the configuration editor.
  virtual auto exportSettings(JsonArray json) -> void {}

  // The device configuration. A single JSON record to edit/backup/restore.
  virtual auto exportConfiguration(JsonObject json) -> void {}

  // The Notes and controllers the device listens to.
  virtual auto exportInput(JsonObject json) -> void {}

  // The notes and controllers the device sends out.
  virtual auto exportOutput(JsonObject json) -> void {}

  // Read the binary configuration from an different/older version.
  virtual auto handleEEPROM(uint16_t version, const void* data, uint32_t size) -> void {}

private:
  // This is only initialized after a cold startup when the memory is undefined.
  // A reset/reboot will not overwrite the data; it is retained across reset/reboot
  // cycles.
  //
  // The linker needs to place the data outside the bss section. Explicit support
  // from the linker script provides a ".noinit" section.
  static class Bootdata {
  public:
    Bootdata() {
      if (_magic == 0x8f734e41)
        return;

      clear();
      _magic = 0x8f734e41;
    }

    void clear() {
      usb.ports.enableAccess = false;
    }

    // The additional MIDI ports to export to the host.
    struct {
      struct {
        bool enableAccess;
      } ports;
    } usb;

  private:
    uint32_t _magic; // [[indeterminate]];
  } _bootdata;

  struct EEPROM {
    const struct Header {
      uint32_t magic{0x7ed63a8b};
      uint32_t size{sizeof(EEPROM)};
    } header;

    // The device-specific part.
    struct {
      uint16_t magic;
      uint16_t version;
      uint32_t size;
    } local{};

    struct {
      // The custom name of the USB device.
      char name[32]{};

      // Custom USB VID/PID.
      uint16_t vid{};
      uint16_t pid{};

      // The number of MIDI ports to create.
      uint8_t ports{};
      uint8_t padding{};
    } usb;
  } _eeprom;

  struct {
    uint32_t id;
  } _boot{};

  struct {
    char hash[41];
  } _firmware{};

  V2Base::Timer::Periodic _ledTimer;

  auto sendReply(V2MIDI::Port& port) -> void;
  auto sendFirmwareStatus(V2MIDI::Port& port, const char* status) -> void;
  auto handleSystemExclusive(V2MIDI::Port* port, const uint8_t* buffer, uint32_t len) -> void override;
  auto readEEPROM(bool dryrun = false) -> bool;
};

extern V2Device::Bootdata __attribute__((section(".noinit"))) V2Device::_bootdata;

// Store the image metadata in a JSON record which is located at the very end
// of the firmware image, with a leading and trailing NUL character. The updater
// can read it and verify that the update file matches the board information.
// The "metadata" section requires explicit support from the linker script to
// be effective.
#define V2DeviceFirmware(_name, _id, _version, _board)                                                                                     \
  const V2Device::MetadataFirmware _name __attribute__((section(".metadata"))) {                                                           \
    .id{_id}, .version{_version}, .board{_board},                                                                                          \
      .json{"\0{\"com.versioduo.firmware\":{"                                                                                              \
            "\"id\":\"" _id "\","                                                                                                          \
            "\"version\":" #_version ","                                                                                                   \
            "\"board\":\"" _board "\"}}"}                                                                                                  \
  }

inline auto V2Device::readEEPROM(bool dryrun) -> bool {
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

inline auto V2Device::begin() -> void {
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
  if (_bootdata.usb.ports.enableAccess)
    usb.ports.enableAccess = true;

  _bootdata.clear();

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
  const char* name = !usb.name.empty() ? usb.name.c_str() : metadata.product;
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

  usb.midi.setVersion(metadata.firmware->version);
  usb.midi.attach();

  // Sleep mode IDLE, wait for interrupts.
  V2Base::Power::setSleepMode(V2Base::Power::Mode::Idle);
}

inline auto V2Device::reset() -> void {
  led.reset();
  resetSystemExclusive();
  handleReset();
}

inline auto V2Device::loop() -> void {
  led.loop();
  loopSystemExclusive();
  handleLoop();
}

// Reply with message to indicate that we are ready for the next packet.
inline auto V2Device::sendFirmwareStatus(V2MIDI::Port& port, const char* status) -> void {
  JsonDocument json;
  auto         jsonDevice{json["com.versioduo.device"].to<JsonObject>()};
  jsonDevice["token"] = _boot.id;
  {
    auto j{jsonDevice["firmware"].to<JsonObject>()};
    j["status"] = status;
  }
  {
    std::string s;
    serializeJson(json, s);

    auto& reply{systemExclusiveBuffer()};
    reply.push_back(uint8_t(V2MIDI::Packet::Status::SystemExclusive));
    reply.push_back(0x7d); // 0x7d == SysEx research/private ID
    reply.append(s);
    reply.push_back(uint8_t(V2MIDI::Packet::Status::SystemExclusiveEnd));
  }
  sendSystemExclusive(port);
}

static inline auto utf8Codepoint(const uint8_t* utf8, uint32_t* codepointp) -> int8_t {
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
static inline auto appendEscapedJSON(const std::string& in, std::string& out) {
  for (uint32_t i{}; i < in.size(); i++) {
    if (in[i] > 0x7f) {
      uint32_t codepoint{};
      uint8_t  len = utf8Codepoint((const uint8_t*)in.data() + i, &codepoint);
      if (len < 0)
        continue;

      // Advance the additional UTF8 characters for this codepoint.
      i += len - 1;

      if (codepoint < 0xffff) {
        char hex[8];
        sprintf(hex, "\\u%04x", codepoint);
        out.append(hex);

      } else {
        codepoint -= 0x10000;
        auto surrogate1{uint16_t((codepoint >> 10) + 0xd800)};
        auto surrogate2{uint16_t((codepoint & 0x3ff) + 0xdc00)};
        char hex[16];
        sprintf(hex, "\\u%04x\\u%04x", surrogate1, surrogate2);
        out.append(hex);
      }

    } else {
      out.push_back(in[i]);
    }
  }
}

inline auto addStatistics(JsonObject json, const V2MIDI::Port::Statistics& s) -> void {
  auto counter{[](JsonObject j, const V2MIDI::Port::Counter& c) {
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
inline auto V2Device::sendReply(V2MIDI::Port& port) -> void {
  JsonDocument json;
  auto         jsonDevice{json["com.versioduo.device"].to<JsonObject>()};
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

    jsonMeta["version"] = metadata.firmware->version;
    exportMetadata(jsonMeta);
  }
  {
    auto jsonLinks{jsonDevice["links"].to<JsonArray>()};
    exportLinks(jsonLinks);
  }
  {
    auto jsonHelp{jsonDevice["help"].to<JsonObject>()};
    if (help.device)
      jsonHelp["device"] = help.device;

    if (help.configuration)
      jsonHelp["configuration"] = help.configuration;
  }
  {
    auto jsonSystem{jsonDevice["system"].to<JsonObject>()};
    if (!usb.name.empty())
      jsonSystem["name"] = usb.name;

    {
      auto j{jsonSystem["boot"].to<JsonObject>()};
      j["uptime"] = float(millis()) / 1000.f;
      j["id"]     = _boot.id;
    }

    {
      auto j{jsonSystem["connection"].to<JsonObject>()};
      j["port"] = port.name;
      addStatistics(j, statistics);
      exportSystem(jsonSystem);
    }

    {
      auto j{jsonSystem["firmware"].to<JsonObject>()};
      if (system.download)
        j["download"] = system.download;

      if (system.configure)
        j["configure"] = system.configure;

      j["id"]    = metadata.firmware->id;
      j["board"] = metadata.firmware->board;
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
        auto ram{jsonHardware["ram"].to<JsonObject>()};
        ram["size"] = V2Base::Memory::RAM::size();
        ram["free"] = V2Base::Memory::RAM::free();
        {
          auto j{ram["data"].to<JsonObject>()};
          j["size"]        = V2Base::Memory::RAM::Data::size();
          j["initialized"] = V2Base::Memory::RAM::Data::Initialized::size();
        }
        {
          auto j{ram["heap"].to<JsonObject>()};
          j["size"]      = V2Base::Memory::RAM::Heap::size();
          j["allocated"] = V2Base::Memory::RAM::Heap::allocated();
        }
        {
          auto j{ram["stack"].to<JsonObject>()};
          j["size"] = V2Base::Memory::RAM::Stack::size();
        }
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
        auto statistics{[](JsonObject p, const V2Link::Port::Counters& c) {
          if (c.input.packet > 0 || c.output.packet > 0) {
            auto j{p["packet"].to<JsonObject>()};
            if (c.input.packet > 0)
              j["input"] = c.input.packet;

            if (c.output.packet > 0)
              j["output"] = c.output.packet;
          }

          if (c.input.pulse > 0 || c.output.pulse > 0) {
            auto j{p["pulse"].to<JsonObject>()};
            if (c.input.pulse > 0)
              j["input"] = c.input.pulse;

            if (c.output.pulse > 0)
              j["output"] = c.output.pulse;
          }

          if (c.input.number > 0 || c.output.number > 0) {
            auto j{p["number"].to<JsonObject>()};
            if (c.input.number > 0)
              j["input"] = c.input.number;

            if (c.output.number > 0)
              j["output"] = c.output.number;
          }

          if (c.input.error > 0 || c.output.error > 0) {
            auto j{p["error"].to<JsonObject>()};
            if (c.input.error > 0)
              j["input"] = c.input.error;

            if (c.output.error > 0)
              j["output"] = c.output.error;
          }
        }};

        if (link->plug) {
          auto j{jsonHardware[link->plug->name].to<JsonObject>()};
          statistics(j, link->plug->counter);
          addStatistics(j, link->plug->statistics);
        }

        if (link->socket) {
          auto j{jsonHardware[link->socket->name].to<JsonObject>()};
          statistics(j, link->socket->counter);
          addStatistics(j, link->socket->statistics);
        }

        if (link->socketNode) {
          auto j{jsonHardware[link->socketNode->name].to<JsonObject>()};
          statistics(j, link->socketNode->counter);
          addStatistics(j, link->socketNode->statistics);
        }
      }

      for (const auto& p : ports) {
        auto j{jsonHardware[p->name].to<JsonObject>()};
        addStatistics(j, p->statistics);
      }
    }
  }
  {
    auto settings{jsonDevice["settings"].to<JsonArray>()};
    exportSettings(settings);
  }
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
  {
    auto input{jsonDevice["input"].to<JsonObject>()};
    exportInput(input);
    if (input.begin() == input.end())
      jsonDevice.remove("input");
  }
  {
    auto output{jsonDevice["output"].to<JsonObject>()};
    exportOutput(output);
    if (output.begin() == output.end())
      jsonDevice.remove("output");
  }
  {
    std::string s;
    serializeJson(json, s);

    auto& reply{systemExclusiveBuffer()};
    reply.push_back(uint8_t(V2MIDI::Packet::Status::SystemExclusive));
    reply.push_back(0x7d); // 0x7d == SysEx research/private ID
    appendEscapedJSON(s, reply);
    reply.push_back(uint8_t(V2MIDI::Packet::Status::SystemExclusiveEnd));
  }
  sendSystemExclusive(port);
}

// Handle a SystemExclusive, JSON request from the host.
inline auto V2Device::handleSystemExclusive(V2MIDI::Port* port, const uint8_t* buffer, uint32_t len) -> void {
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
  JsonObject jsonDevice{json["com.versioduo.device"]};
  if (!jsonDevice)
    return;

  if (jsonDevice["method"] == "getAll") {
    json.clear();
    sendReply(*port);
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
    sendReply(*port);
    return;

  } else if (jsonDevice["method"] == "reboot") {
    V2Base::Memory::Firmware::reboot();
    return;

  } else if (jsonDevice["method"] == "bootloader") {
    V2Base::Memory::Firmware::bootloader();
    return;

  } else if (jsonDevice["method"] == "rebootWithPorts") {
    _bootdata.usb.ports.enableAccess = true;
    V2Base::Memory::Firmware::reboot();
    return;

  } else if (jsonDevice["method"] == "writeConfiguration") {
    // Write the configuration the the EEPROM.

    // The data is enclosed in an object to prevent possible name clashes with the calling convention.
    if (auto config{jsonDevice["configuration"]}; config) {
      if (auto jsonUsb{config["usb"]}; jsonUsb) {
        if (!jsonUsb["name"].isNull()) {
          std::fill(_eeprom.usb.name, _eeprom.usb.name + sizeof(_eeprom.usb.name), 0);
          usb.name.clear();

          if (!name.size() < sizeof(_eeprom.usb.name) - 1) {
            usb.name = jsonUsb["name"].as<std::string_view>();
            std::copy(usb.name.begin(), usb.name.end(), _eeprom.usb.name);
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
    sendReply(*port);
    return;

  } else if (jsonDevice["method"] == "writeFirmware") {
    // The data in enclosed in an object to prevent name clashes with the
    // calling convention.
    JsonObject firmware = jsonDevice["firmware"];
    if (firmware) {
      uint32_t offset = firmware["offset"];
      if (offset % V2Base::Memory::Flash::getBlockSize() != 0) {
        sendFirmwareStatus(*port, "invalidOffset");
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
          sendFirmwareStatus(*port, "success");

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

        sendFirmwareStatus(*port, "hashMismatch");

      } else {
        sendFirmwareStatus(*port, "success");
      }
    }

    return;
  }
}

inline auto V2Device::writeConfiguration() -> void {
  // Common section.
  _eeprom.local.magic   = usb.pid;
  _eeprom.local.version = configuration.version;
  _eeprom.local.size    = configuration.size;
  V2Base::Memory::EEPROM::write(0, (const uint8_t*)&_eeprom, sizeof(_eeprom));

  // Device-specific section.
  if (configuration.size > 0)
    V2Base::Memory::EEPROM::write(sizeof(_eeprom), (const uint8_t*)configuration.data, configuration.size);
}

inline auto V2Device::idle() -> bool {
  if (!usb.midi.idle())
    return false;

  return true;
}
