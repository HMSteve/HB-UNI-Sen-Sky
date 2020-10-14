//- -----------------------------------------------------------------------------------------------------------------------
// AskSin++
// 2018-04-03 papa Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
// 2018-08-25 jp112sdl Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
//- -----------------------------------------------------------------------------------------------------------------------

/***************************************************
 This is a library for the MLX90614 Temp Sensor

 Designed specifically to work with the MLX90614 sensors in the
 adafruit shop
 ----> https://www.adafruit.com/products/1748
 ----> https://www.adafruit.com/products/1749

 These sensors use I2C to communicate, 2 pins are required to
 interface
 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 Written by Limor Fried/Ladyada for Adafruit Industries.
 BSD license, all text above must be included in any redistribution
 ****************************************************/

#ifndef __SENSORS_MLX90614_h__
#define __SENSORS_MLX90614_h__

#include <Sensors.h>
#include <Wire.h>

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

// RAM
#define MLX90614_RAWIR1 0x04
#define MLX90614_RAWIR2 0x05
#define MLX90614_TA 0x06
#define MLX90614_TOBJ1 0x07
#define MLX90614_TOBJ2 0x08
// EEPROM
#define MLX90614_TOMAX 0x20
#define MLX90614_TOMIN 0x21
#define MLX90614_PWMCTRL 0x22
#define MLX90614_TARANGE 0x23
#define MLX90614_EMISS 0x24
#define MLX90614_CONFIG 0x25
#define MLX90614_ADDR 0x0E
#define MLX90614_ID1 0x3C
#define MLX90614_ID2 0x3D
#define MLX90614_ID3 0x3E
#define MLX90614_ID4 0x3F

namespace as {

template <byte ADDRESS>
class Sens_MLX90614 : public Sensor {
public:
    int16_t _temperatureAmb = -999;
    int16_t _temperatureObj = -999;

  void init () {
    Wire.begin();
    _present = read16(MLX90614_CONFIG) != 0xffff;
    DPRINT("MLX90614 ");
    if (_present) {
      DPRINTLN("OK");
    } else {
      DPRINTLN("ERROR");
    }
  }
  void measure (__attribute__((unused)) bool async=false) {
    if( present() == true ) {
      _temperatureAmb = readAmbientTempC();
      DPRINT("MLX90614 Ambient T = ");DDECLN(_temperatureAmb);
      _temperatureObj = readObjectTempC();
      DPRINT("MLX90614 Object  T = ");DDECLN(_temperatureObj);
    }
  }

  uint8_t sleep(void)
  {
  	// Calculate a crc8 value.
  	// Bits sent: _deviceAddress (shifted left 1) + 0xFF
  	uint8_t crc = crc8(0, (ADDRESS << 1));
  	crc = crc8(crc, 0xFF);
  	// Manually send the sleep command:
  	Wire.beginTransmission(ADDRESS);
  	Wire.write(0xFF);
  	Wire.write(crc);
  	Wire.endTransmission(true);
  }

  uint8_t wake(void)
  {
    TWCR &= ~_BV(TWEN); //Disable Two Wire
    pinMode(SDA,OUTPUT);
    digitalWrite(SDA, LOW);
    delay(36);
    digitalWrite(SDA, HIGH);
    Wire.begin();
    delay(250);
  }

private:
  int16_t readObjectTempC(void) {
    return readTemp(MLX90614_TOBJ1);
  }

  int16_t readAmbientTempC(void) {
    return readTemp(MLX90614_TA);
  }

  float readTemp(uint8_t reg) {
    float temp;

    temp = read16(reg);
    temp = read16(reg); // have to do this twice
    temp *= .02;
    temp  -= 273.15;
    return temp * 10;
  }

  uint16_t read16(uint8_t a) {
    uint16_t ret;

    Wire.beginTransmission(ADDRESS); // start transmission to device
    Wire.write(a); // sends register address to read from
    Wire.endTransmission(false); // end transmission

    Wire.requestFrom(ADDRESS, (uint8_t)3);// send data n-bytes read
    ret = Wire.read(); // receive DATA
    ret |= Wire.read() << 8; // receive DATA

    uint8_t pec = Wire.read();

    return ret;
  }

  uint8_t crc8 (uint8_t inCrc, uint8_t inData)
  {
  	uint8_t i;
  	uint8_t data;
  	data = inCrc ^ inData;
  	for ( i = 0; i < 8; i++ )
  	{
  		if (( data & 0x80 ) != 0 )
  		{
  			data <<= 1;
  			data ^= 0x07;
  		}
  		else
  		{
  			data <<= 1;
  		}
  	}
  	return data;
  }


public:
    int16_t  temperatureAmb ()    { return _temperatureAmb; }
    int16_t  temperatureObj ()    { return _temperatureObj; }
};

}
#endif
