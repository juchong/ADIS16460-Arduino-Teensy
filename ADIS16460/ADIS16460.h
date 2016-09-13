////////////////////////////////////////////////////////////////////////////////////////////////////////
//  November 2015
//  Author: Juan Jose Chong <juan.chong@analog.com>
////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ADIS16460.h
////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
//  This library provides all the functions necessary to interface the ADIS16460 IMU with an 
//  8-Bit Atmel-based Arduino development board. Functions for SPI configuration, reads and writes,
//  and scaling are included. This library may be used for the entire ADIS1646X family of devices 
//  with some modification.
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
////////////////////////////////////////////////////////////////////////////////////////////////////////

#define ADIS16460_h
#include "Arduino.h"
#include <SPI.h>

// User Register Memory Map from Table 6
#define FLASH_CNT   0x00  //Flash memory write count
#define DIAG_STAT   0x02  //Diagnostic and operational status
#define X_GYRO_LOW  0x04  //X-axis gyroscope output, lower word
#define X_GYRO_OUT  0x06  //X-axis gyroscope output, upper word
#define Y_GYRO_LOW  0x08  //Y-axis gyroscope output, lower word
#define Y_GYRO_OUT  0x0A  //Y-axis gyroscope output, upper word
#define Z_GYRO_LOW  0x0C  //Z-axis gyroscope output, lower word
#define Z_GYRO_OUT  0x0E  //Z-axis gyroscope output, upper word
#define X_ACCL_LOW  0x10  //X-axis accelerometer output, lower word
#define X_ACCL_OUT  0x12  //X-axis accelerometer output, upper word
#define Y_ACCL_LOW  0x14  //Y-axis accelerometer output, lower word
#define Y_ACCL_OUT  0x16  //Y-axis accelerometer output, upper word
#define Z_ACCL_LOW  0x18  //Z-axis accelerometer output, lower word
#define Z_ACCL_OUT  0x1A  //Z-axis accelerometer output, upper word
#define SMPL_CNTR   0x1C  //Sample Counter, MSC_CTRL[3:2]=11
#define TEMP_OUT    0x1E  //Temperature output (internal, not calibrated)
#define X_DELT_ANG  0x24  //X-axis delta angle output
#define Y_DELT_ANG  0x26  //Y-axis delta angle output
#define Z_DELT_ANG  0x28  //Z-axis delta angle output
#define X_DELT_VEL  0x2A  //X-axis delta velocity output
#define Y_DELT_VEL  0x2C  //Y-axis delta velocity output
#define Z_DELT_VEL  0x2E  //Z-axis delta velocity output
#define MSC_CTRL    0x32  //Miscellaneous control
#define SYNC_SCAL   0x34  //Sync input scale control
#define DEC_RATE    0x36  //Decimation rate control
#define FLTR_CTRL   0x38  //Filter control, auto-null record time
#define GLOB_CMD    0x3E  //Global commands
#define XGYRO_OFF   0x40  //X-axis gyroscope bias offset error
#define YGYRO_OFF   0x42  //Y-axis gyroscope bias offset error
#define ZGYRO_OFF   0x44  //Z-axis gyroscope bias offset factor
#define XACCL_OFF   0x46  //X-axis acceleration bias offset factor
#define YACCL_OFF   0x48  //Y-axis acceleration bias offset factor
#define ZACCL_OFF   0x4A  //Z-axis acceleration bias offset factor
#define LOT_ID1     0x52  //Lot identification number
#define LOT_ID2     0x54  //Lot identification number
#define PROD_ID     0x56  //Product identifier
#define SERIAL_NUM  0x58  //Lot-specific serial number
#define CAL_SGNTR   0x60  //Calibration memory signature value
#define CAL_CRC     0x62  //Calibration memory CRC values
#define CODE_SGNTR  0x64  //Code memory signature value
#define CODE_CRC    0x66  //Code memory CRC values

// ADIS16460 class definition
class ADIS16460{

public:
  // Constructor with configurable CS, data ready, and HW reset pins

  //ADIS16460(int CS, int DR, int RST, int MOSI, int MISO, int CLK);
  ADIS16460(int CS, int DR, int RST);

  //Destructor
  ~ADIS16460();

  //Performs hardware reset by sending pin 8 low on the DUT for 2 seconds
  int resetDUT(uint8_t ms);

  //Sets SPI bit order, clock divider, and data mode
  int configSPI();

  //Read sensor
  int16_t regRead(uint8_t regAddr);

  //Write register
  int regWrite(uint8_t regAddr, int16_t regData);

  int16_t * burstRead(uint8_t regAddr);

  int checksum(int16_t * burstArray);

  //Scale accelerator data
  float accelScale(int16_t sensorData);

  //Scale gyro data
  float gyroScale(int16_t sensorData);

  //Scale temperature data
  float tempScale(int16_t sensorData);

  //Scale delta angle data
  float deltaAngleScale(int16_t sensorData);

  //Scale delta velocity
  float deltaVelocityScale(int16_t sensorData);

private:
  //Variables to store hardware pin assignments
  int _CS;
  int _DR;
  int _RST;

};

