#pragma once


#include "Arduino.h"
#include "Wire.h"
#include <time.h>

enum RTC_Type {
    DS1338,
    MCP79411,
    PCF8563
};

class Artron_RTC {
    private:
        TwoWire *wire = NULL;
        RTC_Type type;
        int devAddr = 0x00;

        uint8_t BCDtoDEC(uint8_t n) ;
        uint8_t DECtoBCD(uint8_t n) ;

        bool CheckI2CDevice(int addr) ;

    public:
        Artron_RTC(TwoWire *bus = &Wire);
        
        bool begin() ;
        bool read(struct tm *timeinfo) ;
        bool write(struct tm *timeinfo) ;

};
