#include "Artron_RTC.h"

#define DS1338_ADDR 0x68
#define MCP79411_ADDR 0x6F
#define PCF8563_ADDR 0x51
#define RTC_ADDR MCP79411_ADDR

Artron_RTC::Artron_RTC(TwoWire *bus) {
    this->wire = bus;
}

bool Artron_RTC::begin() {
    if (CheckI2CDevice(DS1338_ADDR)) {
        this->type = DS1338;
        Serial.println("RTC chip found DS1338");
    } else if (CheckI2CDevice(MCP79411_ADDR)) {
        this->type = MCP79411;
        Serial.println("RTC chip found MCP79411");
    } else if (CheckI2CDevice(PCF8563_ADDR)) {
        this->type = PCF8563;
        Serial.println("RTC chip found PCF8563");
    } else {
        Serial.println("Error, Not found RTC device");
    }

    if (this->type == DS1338) {
        this->devAddr = DS1338_ADDR;

        this->wire->beginTransmission(this->devAddr);
        this->wire->write(0x07); // Start at address 0x07
        this->wire->write(0); // Write 0 to Oscillator Stop Flag for start
        if (this->wire->endTransmission() != 0) {
            return false;
        }
    }

    if (this->type == MCP79411) {
        this->devAddr = MCP79411_ADDR;

        this->wire->beginTransmission(this->devAddr);
        this->wire->write(0x00); // Start at address 0
        if (this->wire->endTransmission() != 0) {
            return false;
        }

        size_t len = this->wire->requestFrom(this->devAddr, 1);
        if (len != 1) {
            return false;
        }

        uint8_t rtcsec = this->wire->read();
        Serial.println((byte) rtcsec, HEX);
        if ((rtcsec & 0x80) == 0) { // ST flag is 0
            Serial.printf("Set start flag 0x%02x\n", rtcsec);
            this->wire->beginTransmission(this->devAddr);
            this->wire->write(0x00); // Start at address 0
            this->wire->write(rtcsec | 0x80); // Set ST flag
            if (this->wire->endTransmission() != 0) {
                return false;
            }
        }

        this->wire->beginTransmission(this->devAddr);
        this->wire->write(0x07); // Start at address 0
        this->wire->write(0); // Write 0 to EXTOSC flag, Disable external 32.768 kHz input
        if (this->wire->endTransmission() != 0) {
            return false;
        }
    }

    if (this->type == PCF8563) {
        this->devAddr = PCF8563_ADDR;

        this->wire->beginTransmission(this->devAddr);
        this->wire->write(0); // Start at address 0
        this->wire->write(0); // Write 0 to Oscillator Stop Flag for start
        if (this->wire->endTransmission() != 0) {
            return false;
        }
    }

    return true;
}

bool Artron_RTC::read(struct tm* timeinfo) {
    if (!timeinfo) {
        return false;
    }

    this->wire->beginTransmission(this->devAddr);
    if (this->type == PCF8563) {
        this->wire->write(0x02); // Start at address 0
    } else {
        this->wire->write(0); // Start at address 0
    }
    if (this->wire->endTransmission() != 0) {
        return false;
    }

    size_t len = this->wire->requestFrom(this->devAddr, 7);
    if (len != 7) {
        return false;
    }

    uint8_t buff[7];
    this->wire->readBytes(buff, 7);

    timeinfo->tm_sec = BCDtoDEC(buff[0] & 0x7F);
    timeinfo->tm_min = BCDtoDEC(buff[1] & 0x7F);
    timeinfo->tm_hour = BCDtoDEC(buff[2] & 0x3F);
    if (this->type == PCF8563) {
        timeinfo->tm_mday = BCDtoDEC(buff[3] & 0x3F);
        timeinfo->tm_wday = BCDtoDEC(buff[4] & 0x07);
    } else {
        timeinfo->tm_wday = BCDtoDEC(buff[3] & 0x07);
        timeinfo->tm_mday = BCDtoDEC(buff[4] & 0x3F);
    }
    timeinfo->tm_mon = BCDtoDEC(buff[5] & 0x1F);
    timeinfo->tm_year = BCDtoDEC(buff[6]) + 2000 - 1900;

    return true;
}

bool Artron_RTC::write(struct tm* timeinfo) {
    uint8_t buff[7];
    buff[0] = DECtoBCD(timeinfo->tm_sec) & 0x7F;
    if (this->type == MCP79411) {
        buff[0] |= 0x80;
    }
    buff[1] = DECtoBCD(timeinfo->tm_min) & 0x7F;
    buff[2] = DECtoBCD(timeinfo->tm_hour) & 0x3F;
    if (this->type == PCF8563) {
        buff[3] = DECtoBCD(timeinfo->tm_mday) & 0x3F;
        buff[4] = DECtoBCD(timeinfo->tm_wday) & 0x07;
    } else {
        buff[3] = DECtoBCD(timeinfo->tm_wday) & 0x07;
        if (this->type == MCP79411) {
            buff[3] |= (1 << 3); // Set VBATEN flag
        }
        buff[4] = DECtoBCD(timeinfo->tm_mday) & 0x3F;
    }
    buff[5] = DECtoBCD(timeinfo->tm_mon) & 0x1F;
    buff[6] = DECtoBCD((timeinfo->tm_year + 1900) % 100);

    this->wire->beginTransmission(this->devAddr);
    if (this->type == PCF8563) {
        this->wire->write(0x02); // Start at address 2
    } else {
        this->wire->write(0); // Start at address 0
    }
    this->wire->write(buff, 7);
    if (this->wire->endTransmission() != 0) {
        return false;
    }

    return true;
}

bool Artron_RTC::CheckI2CDevice(int addr) {
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
}

uint8_t Artron_RTC::BCDtoDEC(uint8_t n) {
    return ((n >> 4)  * 10) + (n & 0x0F);
}

uint8_t Artron_RTC::DECtoBCD(uint8_t n) {
    return (((n / 10) << 4) & 0xF0) | ((uint8_t)(n % 10) & 0x0F);
}