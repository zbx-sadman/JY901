#pragma once
#include <stdint.h>
#include "witmotion_base.h"
#include <Wire.h>

#define JY901_ADDRESS_I2C_DEFAULT 0x50

class JY901_I2C : public JY901_BASE {
  public:
    JY901_I2C(TwoWire& _twoWire)                      { configure(_twoWire, JY901_ADDRESS_I2C_DEFAULT); }
    JY901_I2C(TwoWire& _twoWire, uint8_t _i2cAddress) { configure(_twoWire, _i2cAddress); }

    inline void configure(TwoWire& _twoWire) { configure(_twoWire, JY901_ADDRESS_I2C_DEFAULT); }
    void configure(TwoWire&, uint8_t);

    uint8_t  begin();

    // Bulk raw data readers / updaters
    uint8_t  fetchDateTime();
    uint8_t  fetchAccelerations();
    uint8_t  fetchAngularVelocities();
    uint8_t  fetchAngles();
    uint8_t  fetchMagnetics();
    uint8_t  fetchEpioses();
    uint8_t  fetchTemperature();
    uint8_t  fetchBarometer();
    uint8_t  fetchGpsPosition();
    uint8_t  fetchGpsSpeed();
    uint8_t  fetchQuaternions();
    // ???
    uint8_t  fetchGpsAccuracy();

  private:
    uint8_t  isI2CDeviceReady();
    uint8_t  sendData(uint8_t, uint16_t);
    uint8_t  readData(uint8_t, uint8_t*, uint8_t);
             
    uint16_t fetchRaw16(uint8_t);
    uint32_t fetchRaw32(uint8_t);

    TwoWire* ptrTwoWire;
    uint8_t  i2cAddress;
};
