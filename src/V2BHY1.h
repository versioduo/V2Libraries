#pragma once
#include <Arduino.h>
#include <V23D.h>
#include <Wire.h>

// XYZ – ENU (East-North-Up), right handed.
class V2BHY1 {
public:
  constexpr V2BHY1(TwoWire* i2c, uint8_t pin_interrupt) : _pin_interrupt(pin_interrupt), _i2c(i2c) {}

  auto begin() -> void;
  auto reset() -> void;
  auto loop() -> void;
  auto getRAMVersion() -> uint16_t;
  auto getProductID() -> uint8_t;
  auto getRevisionID() -> uint8_t;

  // Use the magnetometer and orient towards magnetic north, or use the relative orientation only.
  auto getGeoOrientation() -> V23D::Quaternion;
  auto getOrientation() -> V23D::Quaternion;

  auto getGravity() -> V23D::Vector3;
  auto getGyroscope() -> V23D::Vector3;

private:
  uint8_t  _pin_interrupt;
  TwoWire* _i2c;
};
