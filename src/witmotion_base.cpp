#include "witmotion_base.h"

// Return date & time in the wmRaw standart
wmRawDateTime_t JY901_BASE::getRawDateTime() {
  return metrics.dateTime;
}

wmRawAccelerations_t JY901_BASE::getRawAccelerations() {
  return metrics.accelerations;
}

wmRawAngularVelocities_t JY901_BASE::getRawAngularVelocities() {
  return metrics.angularVelocities;
}

wmRawAngles_t JY901_BASE::getRawAngles() {
  return metrics.angles;
}

wmRawMagnetics_t JY901_BASE::getRawMagnetics() {
  return metrics.magnetics;
}

wmRawExtendedPorts_t JY901_BASE::getRawEpioses() {
  return metrics.extendedPorts;
}

wmRawBarometer_t JY901_BASE::getRawBarometer() {
  return metrics.barometer;
}
int16_t JY901_BASE::getRawTemperature() {
  return metrics.temperature;
}

wmRawGpsPosition_t JY901_BASE::getRawGpsPosition() {
  return metrics.gpsPosition;
}

wmRawGpsSpeed_t JY901_BASE::getRawGpsSpeed() {
  return metrics.gpsSpeed;
}

wmRawQuaternions_t JY901_BASE::getRawQuaternions() {
  return metrics.quaternions;
}

wmRawGpsAccuracy_t JY901_BASE::getRawGpsAccuracy() {
  return metrics.gpsAccuracy;
}

// *********************************************************

wmDateTime_t JY901_BASE::getDateTime() {
  return static_cast<wmDateTime_t>(metrics.dateTime);
}

wmAccelerations_t JY901_BASE::getAccelerations() {
  wmAccelerations_t result;
  float k  = (float) 16 / 32768;
  result.x = (float) metrics.accelerations.x * k;
  result.y = (float) metrics.accelerations.y * k;
  result.z = (float) metrics.accelerations.z * k;
  return result;
}

wmAngularVelocities_t JY901_BASE::getAngularVelocities() {
  wmAngularVelocities_t result;
  float k  = (float) 2000 / 32768;
  result.x = (float) metrics.angularVelocities.x * k;
  result.y = (float) metrics.angularVelocities.y * k;
  result.z = (float) metrics.angularVelocities.z * k;
  return result;
}

wmAngles_t JY901_BASE::getAngles() {
  wmAngles_t result;
  float k  = (float) 180 / 32768;
  result.x = (float) metrics.angles.x * k;
  result.y = (float) metrics.angles.y * k;
  result.z = (float) metrics.angles.z * k;
  return result;
}

wmMagnetics_t JY901_BASE::getMagnetics() {
  return static_cast<wmMagnetics_t>(metrics.magnetics);
}

wmExtendedPorts_t JY901_BASE::getEpioses() {
  return static_cast<wmExtendedPorts_t>(metrics.extendedPorts);
}

uint16_t JY901_BASE::getEpioStatus(wmExtendedPort_t _extPort) {
  return getEpioStatus(static_cast<uint8_t>(_extPort));
}

uint16_t JY901_BASE::getEpioStatus(uint8_t _extPortNo) {
  return metrics.extendedPort[_extPortNo];
}

wmBarometer_t JY901_BASE::getBarometer() {
  wmBarometer_t result;
  result.pressure = metrics.barometer.pressure;
  result.altitude = (float) metrics.barometer.altitude / 100;
  return result;
}

float JY901_BASE::getTemperature() {
  return (float) metrics.temperature / 100;
}

uint16_t JY901_BASE::getVersion() {
  return metrics.version;
}

wmGpsPosition_t JY901_BASE::getGpsPosition() {
  const float k1 = 10000000.0;
  const float k2 = 100000.0;
  uint32_t temporary;
  wmGpsPosition_t result;
  // ???: May be need to use chars 'w' / 'e'
  result.longitude.direction = (0x00 < metrics.gpsPosition.longitude) ? east : west;
  temporary = abs(metrics.gpsPosition.longitude);
  result.longitude.degree = (temporary / k1);
  result.longitude.minute = (temporary - (result.longitude.degree * k1)) / k2;

  // ???: May be need to use chars 's' / 'n'
  result.latitude.direction = (0x00 < metrics.gpsPosition.latitude) ? north : south;
  temporary = abs(metrics.gpsPosition.latitude);
  result.latitude.degree = (temporary / k1);
  result.latitude.minute = (temporary - (result.latitude.degree * k1)) / k2;

  return result;
}

wmGpsSpeed_t JY901_BASE::getGpsSpeed() {
  wmGpsSpeed_t result;
  result.altitude = (float) metrics.gpsSpeed.altitude / 10;
  result.yaw      = (float) metrics.gpsSpeed.yaw / 10;
  result.speed    = (float) metrics.gpsSpeed.speed / 1000;
  return result;
}

wmQuaternions_t JY901_BASE::getQuaternions() {
  wmQuaternions_t result;
  result.q0 = (float) metrics.quaternions.q0 / 32768;
  result.q1 = (float) metrics.quaternions.q1 / 32768;
  result.q2 = (float) metrics.quaternions.q2 / 32768;
  result.q3 = (float) metrics.quaternions.q3 / 32768;
  return result;
}

wmGpsAccuracy_t JY901_BASE::getGpsAccuracy() {
  wmGpsAccuracy_t result;
  result.satellitesQuantity = metrics.gpsAccuracy.satellitesQuantity;
  // 32768 from manual is wrong divider?
  result.pdop = (float) metrics.gpsAccuracy.pdop / 100;
  result.hdop = (float) metrics.gpsAccuracy.hdop / 100;
  result.vdop = (float) metrics.gpsAccuracy.vdop / 100;
  return result;
}

uint8_t JY901_BASE::begin() {
  return true;
}
