#include "witmotion_i2c.h"
#include <Wire.h>

#define TWI_NO_ERROR                                             (0x00)
#define TWI_BUFFER_FULL                                          (0x01)
#define TWI_ADDRESS_NACK                                         (0x02)
#define TWI_DATA_NACK                                            (0x03)
#define TWI_OTHER                                                (0x04)


void JY901_I2C::configure(TwoWire& _twoWire, uint8_t _i2cAddress) { 
  ptrTwoWire = &_twoWire; 
  i2cAddress = _i2cAddress; 
}

uint8_t JY901_I2C::begin() {
  ptrTwoWire->begin();
  return isI2CDeviceReady();
}

uint8_t JY901_I2C::readData(uint8_t _registerAddress, uint8_t* _dst, uint8_t _size) {
  uint8_t rc = false, recievedBytes;

  ptrTwoWire->beginTransmission(i2cAddress);
  ptrTwoWire->write((uint8_t) _registerAddress);
  if (TWI_NO_ERROR != ptrTwoWire->endTransmission(false)) {
    goto finish;
  }

  recievedBytes = ptrTwoWire->requestFrom((uint8_t)i2cAddress, _size, (uint8_t)true);
  if (_size != recievedBytes) {
    goto finish;
  }

  ptrTwoWire->readBytes(_dst, _size);
  rc = true;

finish:
  return rc;
}

uint8_t JY901_I2C::sendData(uint8_t _registerAddress, uint16_t _data) {
  const uint8_t packet[] = {_registerAddress, (uint8_t)(_data & 0xFF), (uint8_t)(_data >> 0x08)};

  uint8_t rc = false;
  ptrTwoWire->beginTransmission(i2cAddress);
  ptrTwoWire->write(packet, sizeof(packet));
  rc = ptrTwoWire->endTransmission();
  if (TWI_NO_ERROR == rc) {
    rc = true;
  }
  return rc;
}

uint8_t JY901_I2C::isI2CDeviceReady() {
  ptrTwoWire->beginTransmission(i2cAddress);
  return (TWI_NO_ERROR == ptrTwoWire->endTransmission(true));
}

uint8_t JY901_I2C::fetchDateTime() {
  return readData(JY901_ADDRESS_REGISTER_YEAR_MONTH, (uint8_t*)&metrics.dateTime, sizeof(metrics.dateTime));
}

uint8_t JY901_I2C::fetchAccelerations() {
  // read register range from 'X axis Acceleration' to 'Z axis Acceleration'
  return readData(JY901_ADDRESS_REGISTER_ACCELERATION_X, (uint8_t*)&metrics.accelerations, sizeof(metrics.accelerations));
}

uint8_t JY901_I2C::fetchAngularVelocities() {
  return readData(JY901_ADDRESS_REGISTER_ANGULAR_VELOCITY_X, (uint8_t*)&metrics.angularVelocities, sizeof(metrics.angularVelocities));
}

uint8_t JY901_I2C::fetchAngles() {
  return readData(JY901_ADDRESS_REGISTER_ANGLE_X, (uint8_t*)&metrics.angles, sizeof(metrics.angles));
}

uint8_t JY901_I2C::fetchEpioses() {
  return readData(JY901_ADDRESS_REGISTER_STATUS_D0, (uint8_t*)&metrics.extendedPorts, sizeof(metrics.extendedPorts));
}

uint8_t JY901_I2C::fetchBarometer() {
  return readData(JY901_ADDRESS_REGISTER_BAROMETER_PRESSURE, (uint8_t*)&metrics.barometer, sizeof(metrics.barometer));
}

uint8_t JY901_I2C::fetchTemperature() {
  return readData(JY901_ADDRESS_REGISTER_TEMPERATURE, (uint8_t*)&metrics.temperature, sizeof(metrics.temperature));
}

uint8_t JY901_I2C::fetchGpsPosition() {
  return readData(JY901_ADDRESS_REGISTER_GPS_LONGTITUDE, (uint8_t*)&metrics.gpsPosition, sizeof(metrics.gpsPosition));
}

uint8_t JY901_I2C::fetchGpsSpeed() {
  return readData(JY901_ADDRESS_REGISTER_GPS_YAW, (uint8_t*)&metrics.gpsSpeed, sizeof(metrics.gpsSpeed));
}

uint8_t JY901_I2C::fetchQuaternions() {
  return readData(JY901_ADDRESS_REGISTER_QUATERNION_0, (uint8_t*)&metrics.quaternions, sizeof(metrics.quaternions));
}

uint8_t JY901_I2C::fetchGpsAccuracy() {
  return readData(JY901_ADDRESS_REGISTER_GPS_ACCURACY, (uint8_t*)&metrics.gpsAccuracy, sizeof(metrics.gpsAccuracy));
}

uint16_t JY901_I2C::fetchRaw16(uint8_t _registerAddress) {
  uint16_t result;
  readData(_registerAddress, (uint8_t*)&result, sizeof(result));
  return result;
}

uint32_t JY901_I2C::fetchRaw32(uint8_t _registerAddress) {
  uint32_t result;
  readData(_registerAddress, (uint8_t*)&result, sizeof(result));
  return result;
}

