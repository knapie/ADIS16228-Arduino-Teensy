////////////////////////////////////////////////////////////////////////////////////////////////////////
//  January 2017
//  Author: Juan Jose Chong <juan.chong@analog.com>
////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ADIS16228.h
////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
//  This library provides all the functions necessary to interface the ADIS16228 IMU with a 
//  PJRC 32-Bit Teensy 3.2 Development Board. Functions for SPI configuration, reads and writes,
//  and scaling are included. This library may be used for the entire ADIS1646X family of devices 
//  with some modification.
//
//  Permission is hereby granted, free of charge, to any person obtaining
//  a copy of this software and associated documentation files (the
//  "Software"), to deal in the Software without restriction, including
//  without limitation the rights to use, copy, modify, merge, publish,
//  distribute, sublicense, and/or sell copies of the Software, and to
//  permit persons to whom the Software is furnished to do so, subject to
//  the following conditions:
//
//  The above copyright notice and this permission notice shall be
//  included in all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
//  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
//  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
//  LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
//  WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef ADIS16228_h
#define ADIS16228_h
#include "Arduino.h"
#include <SPI.h>

// User Register Memory Map from Table 8
#define FLASH_CNT 0x00
#define X_SENS 0x02
#define Y_SENS 0x04
#define Z_SENS 0x06
#define TEMP_OUT 0x08 
#define SUPPLY_OUT 0x0A
#define FFT_AVG1 0x0C
#define FFT_AVG2 0x0E
#define BUF_PNTR 0x10
#define REC_PNTR 0x12
#define X_BUF 0x14
#define Y_BUF 0x16
#define Z_BUF 0x18
#define REC_CTRL1 0x1A
#define REC_CTRL2 0x1C
#define REC_PRD 0x1E
#define ALM_F_LOW 0x20
#define ALM_F_HIGH 0x22
#define ALM_X_MAG1 0x24
#define ALM_Y_MAG1 0x26
#define ALM_Z_MAG1 0x28
#define ALM_X_MAG2 0x2A
#define ALM_Y_MAG2 0x2C
#define ALM_Z_MAG2 0x2E
#define ALM_PNTR 0x30
#define ALM_S_MAG 0x32
#define ALM_CTRL 0x34
#define DIO_CTRL 0x36
#define GPIO_CTRL 0x38
#define AVG_CNT 0x3A
#define DIAG_STAT 0x3C
#define GLOB_CMD 0x3E
#define ALM_X_STAT 0x40
#define ALM_Y_STAT 0x42
#define ALM_Z_STAT 0x44
#define ALM_X_PEAK 0x46
#define ALM_Y_PEAK 0x48
#define ALM_Z_PEAK 0x4A
#define TIME_STAMP_L 0x4C
#define TIME_STAML_H 0x4E
#define LOT_ID1 0x52
#define LOT_ID2 0x54
#define PROD_ID 0x56
#define SERIAL_NUM 0x58
#define USER_ID 0x5C
#define REC_FLSH_CNT 0x5E
#define REC_INFO1 0x6E
#define ALM_X_FREQ 0x70
#define ALM_Y_FREQ 0x72
#define ALM_Z_FREQ 0x74
#define REC_INFO2 0x76
#define REC_CNTR 0x78
#define CAL_ENABLE 0x7A

// ADIS16228 class definition
class ADIS16228 {

public:
  // Constructor with configurable CS, data ready, and HW reset pins

  // ADIS16228(int CS, int DR, int RST, int MOSI, int MISO, int CLK);
  ADIS16228(int CS, int DR, int RST);

  // Destructor
  ~ADIS16228();

  // Performs hardware reset by sending Arduino pin 8 low for n ms
  int resetDUT(uint8_t ms);

  // Sets SPI bit order, clock divider, and data mode
  int configSPI();

  // Read single 16-bit register from sensor
  uint16_t regRead(uint8_t regAddr);

  // Write data to specified register
  int regWrite(uint8_t regAddr, uint16_t regData);

  // Read one FFT buffer axis data 
  uint16_t *readFFT(uint8_t regAddr);

  // Scale accelerator data
  float accelScale(int16_t sensorData);

  // Scale temperature data
  float tempScale(int16_t sensorData);

private:
  // Variables to store hardware pin assignments
  int _CS;
  int _DR;
  int _RST;
  int _stall = 20; //Minimum SPI stall time

};

#endif
