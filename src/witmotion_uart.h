#pragma once
#include <stdint.h>
#include "witmotion_base.h"
#include <Stream.h>


#define JY901_STREAM_SPEED_DEFAULT 9600

class JY901_UART : public JY901_BASE {
  public:
    JY901_UART(Stream& _stream) { configure(_stream); }
    void configure(Stream& _stream);

    // Bulk raw data "reader"
    inline uint8_t readReport() { return readData(); }

  private:

    uint8_t sendData(uint8_t, uint16_t);

    uint8_t readData();

    Stream* ptrStream;

    uint8_t buffer[0x0B];
    uint8_t* ptrData = &buffer[0x02];

    uint8_t writePos = 0x00;
};
