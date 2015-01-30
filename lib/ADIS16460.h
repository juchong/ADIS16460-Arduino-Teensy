////////////////////////////////////////////////////////////////////////////////////////////////////////
//  January 2015
//  By: Juan Jose Chong
////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ADIS16460.h
////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
//  This file interfaces the ADIS16460 IMU with an Arduino-based MCU.
//
//  This example is free software. You can redistribute it and/or modify it
//  under the terms of the GNU Lesser Public License as published by the Free Software
//  Foundation, either version 3 of the License, or any later version.
//
//  This example is distributed in the hope that it will be useful, but WITHOUT ANY
//  WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS 
//  FOR A PARTICULAR PURPOSE.  See the GNU Lesser Public License for more details.
//
//  You should have received a copy of the GNU Lesser Public License along with 
//  this example.  If not, see <http://www.gnu.org/licenses/>.
//
//  This library is based on the ADIS16460 library written by Daniel Tatum.
//
////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef ADIS16460_h
#define ADIS16460_h
#include "Arduino.h"
#include <SPI.h>

//#define DEBUG //Uncomment for DEBUG mode

// User Register Memory Map from Table 9
#define FLASH_CNT 0x00 //Flash memory write count
#define XGYRO_OUT 0x04 //X-axis gyroscope output
#define YGYRO_OUT 0x06 //Y-axis gyroscope output
#define ZGYRO_OUT 0x08 //Z-axis gyroscope output
#define XACCL_OUT 0x0A //X-axis accelerometer output
#define YACCL_OUT 0x0C //Y-axis accelerometer output
#define ZACCL_OUT 0x0E //Z-axis accelerometer output
#define TEMP_OUT 0x10 //Temperature output
#define X_DELT_ANG 0x12 //X-axis delta angle output
#define Y_DELT_ANG 0x14 //Y-axis delta angle output
#define Z_DELT_ANG 0x16 //Z-axis delta angle output
#define X_DELT_VEL 0x18 //X-axis delta velocity output
#define Y_DELT_VEL 0x1A //Y-axis delta velocity output
#define Z_DELT_VEL 0x1C //Z-axis delta velocity output
#define XGYRO_OFF 0x24 //X-axis gyroscope bias offset factor
#define YGYRO_OFF 0x26 //Y-axis gyroscope bias offset factor
#define ZGYRO_OFF 0x28 //Z-axis gyroscope bias offset factor
#define XACCL_OFF 0x2A //X-axis acceleration bias offset factor
#define YACCL_OFF 0x2C //Y-axis acceleration bias offset factor
#define ZACCL_OFF 0x2E //Z-axis acceleration bias offset factor
#define GPIO_CTRL 0x32 //GPIO control
#define MSC_CTRL 0x34 //Misc. control
#define SMPL_PRD 0x36 //Sample clock/Decimation filter control
#define SENS_AVG 0x38 //Digital filter control
#define DIAG_STAT 0x3C //System status
#define GLOB_CMD 0x3E //System command
//#define LOT_ID1 0x52 //Lot identification number
//#define LOT_ID2 0x54 //Lot identification number
//#define PROD_ID 0x56 //Product identifier
//#define SERIAL_NUM 0x58 //Lot-specific serial number


// ADIS16460 class definition
class ADIS16460{

public:
  // Constructor with configurable CS, data ready, and HW reset pins

  //ADIS16460(int CS, int DR, int RST, int MOSI, int MISO, int CLK);
  ADIS16460(int CS, int DR, int RST);

  //Destructor
  ~ADIS16460();

  //Performs hardware reset by sending pin 8 low on the DUT for 2 seconds
  void resetDUT();

  //Sets SPI bit order, clock divider, and data mode
  void configSPI();

  //Read sensor
  int32_t regRead(uint8_t regAddr);

  //Write register
  void regWrite(uint8_t regAddr, uint16_t regData);

  //Scale accelerator data
  float accelScale(int32_t sensorData);

  //Scale gyro data
  float gyroScale(int32_t sensorData);

  //Scale temperature data
  float tempScale(int32_t sensorData);

private:
  //Variables to store hardware pin assignments
  int _CS;
  int _DR;
  int _RST;

};

#endif
