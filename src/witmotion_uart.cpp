#include <stdint.h>
#include "witmotion_uart.h"

void JY901_UART::configure(Stream& _stream) { 
  ptrStream = &_stream; 
}

uint8_t JY901_UART::readData() {

  uint8_t rc = false;
  uint8_t checksum = 0x00;

  if (0x00 >= ptrStream->available()) { goto finish; }

  buffer[writePos] = ptrStream->read();

#if defined(JY901_DEBUG_ON)
  Serial.print("buffer["); Serial.print(writePos); Serial.print("]: 0x"); Serial.print(buffer[writePos], HEX); Serial.println();
#endif

  writePos = (0x55 == buffer[0x00]) ? (writePos + 0x01) : 0x00;

  if (sizeof(buffer) > writePos) { goto finish; }

  for (uint8_t i = 0x00; (sizeof(buffer) - 0x01) > i; i++) {
    checksum += buffer[i];
  }

#if defined(JY901_DEBUG_ON)
   Serial.print("checksum 0x"); Serial.print(checksum, HEX); Serial.print(",  recv 0x"); Serial.print(buffer[sizeof(buffer)-0x01], HEX); Serial.println();
#endif

  if (buffer[sizeof(buffer) - 0x01] == checksum) 
   { 
     switch(buffer[0x01]) {
       case 0x50: {
         memcpy((uint8_t*)&metrics.dateTime, ptrData, sizeof(metrics.dateTime));
         break; 
       }
    
       case 0x51: {
         memcpy((uint8_t*)&metrics.accelerations, ptrData, sizeof(metrics.accelerations));
         memcpy((uint8_t*)&metrics.temperature, ptrData + sizeof(metrics.accelerations), sizeof(metrics.temperature));
         break; 
       }
    
       case 0x52: {
         memcpy((uint8_t*)&metrics.angularVelocities, ptrData, sizeof(metrics.angularVelocities));
         memcpy((uint8_t*)&metrics.temperature, ptrData + sizeof(metrics.angularVelocities), sizeof(metrics.temperature));
         break; 
       }
    
       case 0x53: {
         memcpy((uint8_t*)&metrics.angles, ptrData, sizeof(metrics.angles));
         // ??? perhaps this is an erroneous assumption. Always return the same in this two bytes
         memcpy((uint8_t*)&metrics.version, ptrData + sizeof(metrics.angles), sizeof(metrics.version));
         break; 
       }
    
       case 0x54: {
         memcpy((uint8_t*)&metrics.magnetics, ptrData, sizeof(metrics.magnetics));
         memcpy((uint8_t*)&metrics.temperature, ptrData + sizeof(metrics.magnetics), sizeof(metrics.temperature));
         break; 
       }
    
       case 0x55: {
         memcpy((uint8_t*)&metrics.extendedPorts, ptrData, sizeof(metrics.extendedPorts));
         break; 
       }
    
       case 0x56: {
         memcpy((uint8_t*)&metrics.barometer, ptrData, sizeof(metrics.barometer));
         break; 
       }
    
       case 0x57: {
         memcpy((uint8_t*)&metrics.gpsPosition, ptrData, sizeof(metrics.gpsPosition));
         break; 
       }
    
       case 0x58: {
         memcpy((uint8_t*)&metrics.gpsSpeed, ptrData, sizeof(metrics.gpsSpeed));
         break; 
       }
    
       case 0x59: {
         memcpy((uint8_t*)&metrics.quaternions, ptrData, sizeof(metrics.quaternions));
         break; 
       }
    
       case 0x5A: {
         memcpy((uint8_t*)&metrics.gpsAccuracy, ptrData, sizeof(metrics.gpsAccuracy));
         break; 
       }
     }
     rc = true;
  }
  writePos = 0x00;
  buffer[0x00] = 0x00;

finish:
  return rc;
}

uint8_t JY901_UART::sendData(uint8_t _registerAddress, uint16_t _data) {
  const uint8_t packet[] = {0xFF, 0xAA, _registerAddress, (uint8_t)(_data & 0xFF), (uint8_t)(_data >> 0x08)};

#if defined(JY901_DEBUG_ON)
  Serial.print("packet:\n"); 
  for (uint8_t i = 0x00; i < sizeof(packet); i++) {
    Serial.print(" 0x"); Serial.print(packet[i], HEX); 
  }
  Serial.println();  
#endif

  ptrStream->write(packet, sizeof(packet));
  return true;
}
