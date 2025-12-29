#include "V2BHY1.h"

extern "C" {
#include "bhy/bhy_firmware.h"
#include "bhy/bhy_support.h"
#include "bhy/bhy_uc_driver.h"
}

extern TwoWire* i2c;

namespace {
  class Sensor {
  public:
    enum class State { Init, WaitForInit, Setup, Running, _count } state;
    bhy_data_quaternion_t rotation;
    bhy_data_quaternion_t game;
    bhy_data_vector_t     gravity;
    bhy_data_vector_t     gyroscope;

    auto reset() {
      state     = {};
      rotation  = {.w{std::numeric_limits<decltype(bhy_data_quaternion_t::w)>::max() / 2}};
      game      = {.w{std::numeric_limits<decltype(bhy_data_quaternion_t::w)>::max() / 2}};
      gravity   = {};
      gyroscope = {};
    }
  } Sensor;

  auto fifoDataHandler(bhy_data_generic_t* data, bhy_virtual_sensor_t sensor_id) {
    switch (sensor_id) {
      case VS_ID_ROTATION_VECTOR_WAKEUP:
        Sensor.rotation = data->data_quaternion;
        break;

      case VS_ID_GAME_ROTATION_VECTOR_WAKEUP:
        Sensor.game = data->data_quaternion;
        break;

      case VS_ID_GRAVITY_WAKEUP:
        Sensor.gravity = data->data_vector;
        break;

      case VS_ID_GYROSCOPE_WAKEUP:
        Sensor.gyroscope = data->data_vector;
        break;
    }
  }

  class {
  public:
    volatile bool pending;

    auto reset() {
      pending          = false;
      _bytes_left      = 0;
      _bytes_remaining = 0;
    }

    auto hasData() const -> bool {
      return _bytes_remaining > 0;
    }

    auto processEvents() {
      pending = false;

      bhy_read_fifo(_data + _bytes_left, sizeof(_data) - _bytes_left, &_bytes_read, &_bytes_remaining);
      _bytes_read += _bytes_left;
      _pos         = _data;
      _packet_type = BHY_DATA_TYPE_PADDING;

      for (;;) {
        if (bhy_parse_next_fifo_packet(&_pos, &_bytes_read, &_packet, &_packet_type) != BHY_SUCCESS)
          break;

        if (_bytes_read <= (_bytes_remaining > 0 ? sizeof(bhy_data_generic_t) : 0))
          break;
      }

      _bytes_left = 0;
      if (_bytes_remaining == 0)
        return;

      while (_bytes_left < _bytes_read)
        _data[_bytes_left++] = *(_pos++);
    }

  private:
    uint8_t            _data[300]{};
    uint8_t*           _pos{};
    uint8_t            _bytes_left{};
    uint16_t           _bytes_remaining{};
    uint16_t           _bytes_read{};
    bhy_data_generic_t _packet{};
    bhy_data_type_t    _packet_type{};
  } FIFO;

  static auto fifoInterruptHandler(void) {
    FIFO.pending = true;
  }

  // Scale factors:
  //  - Rotation Vector, Game Rotation Vector, Geomagnetic Rotation Vector 2^14 -> 2
  //  - Accel, Gravity, Linear Acceleration allows for a 4g range (39.24 m/s2) -> 4
  constexpr auto i16scale(int16_t v, float range) -> float {
    return float(v) / (float(std::numeric_limits<decltype(v)>::max()) / range);
  }
}

auto V2BHY1::begin() -> void {
  i2c = _i2c;
  attachInterrupt(_pin_interrupt, fifoInterruptHandler, RISING);
  reset();
}

auto V2BHY1::reset() -> void {
  Sensor.reset();
}

auto V2BHY1::loop() -> void {
  switch (Sensor.state) {
    case Sensor::State::Init:
      bhy_driver_init(bhy_firmware_bmm150);
      FIFO.reset();
      Sensor.state = Sensor::State::WaitForInit;
      break;

    case Sensor::State::WaitForInit:
      if (!FIFO.pending)
        return;

      Sensor.state = Sensor::State::Setup;
      break;

    case Sensor::State::Setup:
      bhy_install_sensor_callback(VS_TYPE_ROTATION_VECTOR, VS_WAKEUP, fifoDataHandler);
      bhy_enable_virtual_sensor(VS_TYPE_ROTATION_VECTOR, VS_WAKEUP, 100, 0, VS_FLUSH_NONE, 0, 0);

      bhy_install_sensor_callback(VS_TYPE_GAME_ROTATION_VECTOR, VS_WAKEUP, fifoDataHandler);
      bhy_enable_virtual_sensor(VS_TYPE_GAME_ROTATION_VECTOR, VS_WAKEUP, 100, 0, VS_FLUSH_NONE, 0, 0);

      bhy_install_sensor_callback(VS_TYPE_GRAVITY, VS_WAKEUP, fifoDataHandler);
      bhy_enable_virtual_sensor(VS_TYPE_GRAVITY, VS_WAKEUP, 100, 0, VS_FLUSH_NONE, 0, 0);

      bhy_install_sensor_callback(VS_TYPE_GYROSCOPE, VS_WAKEUP, fifoDataHandler);
      bhy_enable_virtual_sensor(VS_TYPE_GYROSCOPE, VS_WAKEUP, 100, 0, VS_FLUSH_NONE, 0, 0);

      Sensor.state = Sensor::State::Running;
      break;

    case Sensor::State::Running:
      if (!FIFO.pending && !FIFO.hasData())
        return;

      FIFO.processEvents();
      break;
  }
}

auto V2BHY1::getRAMVersion() -> uint16_t {
  uint16_t version{};
  bhy_get_ram_version(&version);
  return version;
}

auto V2BHY1::getProductID() -> uint8_t {
  uint8_t id{};
  bhy_get_product_id(&id);
  return id;
}

auto V2BHY1::getRevisionID() -> uint8_t {
  uint8_t id{};
  bhy_get_revision_id(&id);
  return id;
}

auto V2BHY1::getGeoOrientation() -> V23D::Quaternion {
  return V23D::Quaternion(i16scale(Sensor.rotation.w, 2),
                          i16scale(Sensor.rotation.x, 2),
                          i16scale(Sensor.rotation.y, 2),
                          i16scale(Sensor.rotation.z, 2));
}

auto V2BHY1::getOrientation() -> V23D::Quaternion {
  return V23D::Quaternion(i16scale(Sensor.game.w, 2), i16scale(Sensor.game.x, 2), i16scale(Sensor.game.y, 2), i16scale(Sensor.game.z, 2));
}

auto V2BHY1::getGravity() -> V23D::Vector3 {
  return V23D::Vector3(i16scale(Sensor.gravity.x, 4), i16scale(Sensor.gravity.y, 4), i16scale(Sensor.gravity.z, 4));
}

auto V2BHY1::getGyroscope() -> V23D::Vector3 {
  return V23D::Vector3(i16scale(Sensor.gyroscope.x, 4), i16scale(Sensor.gyroscope.y, 4), i16scale(Sensor.gyroscope.z, 4));
}
