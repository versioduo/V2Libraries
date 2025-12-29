extern "C" {
#include "bhy_support.h"
#include "bhy_uc_driver_config.h"
}

#include <Arduino.h>
#include <Wire.h>

static struct bhy_t bhy;
TwoWire*            i2c{};

static int8_t sensor_i2c_read(uint8_t addr, uint8_t reg, uint8_t* p_buf, uint16_t size) {
  i2c->beginTransmission(addr);
  i2c->write(reg);
  i2c->endTransmission();

  i2c->requestFrom(addr, size);
  while (i2c->available()) {
    *p_buf = i2c->read();
    p_buf++;
  }

  return BHY_SUCCESS;
}

static int8_t sensor_i2c_write(uint8_t addr, uint8_t reg, uint8_t* p_buf, uint16_t size) {
  i2c->beginTransmission(addr);
  i2c->write(reg);
  for (uint8_t i = 0; i < size; i++) {
    i2c->write(*p_buf);
    p_buf++;
  }
  i2c->endTransmission();

  yield();
  return BHY_SUCCESS;
}

int8_t bhy_initialize_support(void) {
  int8_t retry{3};

  bhy.bus_write   = &sensor_i2c_write;
  bhy.bus_read    = &sensor_i2c_read;
  bhy.delay_msec  = &bhy_delay_msec;
  bhy.device_addr = BHY_I2C_ADDR1;

  bhy_init(&bhy);
  bhy_set_reset_request(BHY_RESET_ENABLE);

  while (retry-- > 0) {
    bhy_get_product_id(&bhy.product_id);

    if (bhy.product_id == PRODUCT_ID_7183)
      return BHY_SUCCESS;

    bhy_delay_msec(BHY_PARAMETER_ACK_DELAY);
  }

  return BHY_PRODUCT_ID_ERROR;
}

void bhy_delay_msec(unsigned int msec) {
  delay(msec);
}
