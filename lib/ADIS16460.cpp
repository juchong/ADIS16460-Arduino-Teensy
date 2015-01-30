////////////////////////////////////////////////////////////////////////////////////////////////////////
//  January 2015
//  By: Juan Jose Chong
////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ADIS16460.cpp
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

#include "ADIS16460.h"

////////////////////////////////////////////////////////////////////////////
// Constructor with configurable CS, DR, and RST
////////////////////////////////////////////////////////////////////////////
// CS - Chip select pin
// DR - DR output pin for data ready
// RST - Hardware reset pin
////////////////////////////////////////////////////////////////////////////
ADIS16460::ADIS16460(int CS, int DR, int RST) {
  _CS = CS;
  _DR = DR;
  _RST = RST;
  #ifdef __SAM3X8E__
    SPI.begin();
    SPI.setBitOrder(MSBFIRST);
    SPI.setClockDivider(10, 42); //For 2MHz
    SPI.setDataMode(SPI_MODE3);
  #else
    SPI.begin(); //Initialize SPI bus
    SPI.setBitOrder(MSBFIRST); //As per the ADIS16460 datasheet
    SPI.setClockDivider(SPI_CLOCK_DIV8); //For 1MHz (max ~2MHz)
    SPI.setDataMode(SPI_MODE3); //Clock base at one, sampled on falling edge
  #endif

//Set default pin states
  pinMode(_CS, OUTPUT); // Set CS pin to be an output
  pinMode(_DR, INPUT); // Set DR pin to be an input
  pinMode(_RST, OUTPUT); // Set RST pin to be an output
  digitalWrite(_CS, HIGH); // Initialize CS pin to be high
  digitalWrite(_RST, HIGH); // Initialize RST pin to be high
}

////////////////////////////////////////////////////////////////////////////
// Destructor
////////////////////////////////////////////////////////////////////////////
ADIS16460::~ADIS16460() {
  // Close SPI bus
  SPI.end();
}

////////////////////////////////////////////////////////////////////////////
// Performs hardware reset by setting _RST pin low for 2 seconds.
////////////////////////////////////////////////////////////////////////////
void ADIS16460::resetDUT() {
  digitalWrite(_RST, LOW);
  delay(100);
  digitalWrite(_RST, HIGH);
  delay(2000);
}

////////////////////////////////////////////////////////////////////////////
// Sets SPI bit order, clock divider, and data mode. This function is useful
// when there are multiple SPI devices using different settings.
////////////////////////////////////////////////////////////////////////////
void ADIS16460::configSPI() {
  #ifdef __SAM3X8E__
    SPI.setBitOrder(MSBFIRST);
    SPI.setClockDivider(10, 42); //For 2MHz
    SPI.setDataMode(SPI_MODE3);
  #else
    SPI.setBitOrder(MSBFIRST); // for ADIS16460
    SPI.setClockDivider(SPI_CLOCK_DIV8); // for 1MHz (ADIS16460 max 2MHz)
    SPI.setDataMode(SPI_MODE3); // Clock base at one, sampled on falling edge
  #endif
}

////////////////////////////////////////////////////////////////////////////////////////////
// Reads two bytes (one word) in two sequential registers over SPI
////////////////////////////////////////////////////////////////////////////////////////////
// regAddr - register address from the lookup table in ADIS16460.h
// return - (int) signed 16 bit 2's complement number
////////////////////////////////////////////////////////////////////////////////////////////
int32_t ADIS16460::regRead(uint8_t regAddr) {
//Read registers using SPI
#ifdef __SAM3X8E__
  //SPI transfers for Arduino Due
  configSPI();

  // Write register address to be read
  SPI.transfer(10, regAddr, SPI_CONTINUE); //write data over SPI bus
  SPI.transfer(10, 0x00); //write 0x00 to the SPI bus fill the 16 bit requirement

  delayMicroseconds(15); //delay to not violate readrate (40us)

  // Read data from register requested
  uint8_t _msbData = SPI.transfer(10, 0x00, SPI_CONTINUE); // send (0x00) and read SPI bus
  uint8_t _lsbData = SPI.transfer(10, 0x00); // send dummy data and read

  delayMicroseconds(15); //delay to not violate readrate (40us)
  
  int32_t _dataOut = (_msbData << 8) | (_lsbData & 0xFF); //concatenate upper and lower bytes
#else
//SPI transfers for all other Arduinos
  configSPI();

  // Write register address to be read
  digitalWrite(_CS, LOW); //enable device
  SPI.transfer(regAddr); //write data over SPI bus
  SPI.transfer(0x00); //write 0x00 to the SPI bus fill the 16 bit requirement
  digitalWrite(_CS, HIGH); //disable device

  delayMicroseconds(15); //delay to not violate readrate (40us)

  // Read data from register requested
  digitalWrite(_CS, LOW); //enable device
  uint8_t _msbData = SPI.transfer(0x00); // send (0x00) and read SPI bus
  uint8_t _lsbData = SPI.transfer(0x00); // send dummy data and read
  digitalWrite(_CS, HIGH); //disable device

  delayMicroseconds(15); //delay to not violate readrate (40us)
  
  int32_t _dataOut = (_msbData << 8) | (_lsbData & 0xFF); //concatenate upper and lower bytes
#endif

#ifdef DEBUG 
  Serial.print("Register 0x");
  Serial.print((unsigned char)regAddr, HEX);
  Serial.print(" reads: ");
  Serial.println(_dataOut);
#endif

  return(_dataOut);
}

////////////////////////////////////////////////////////////////////////////
// Writes one byte of data to the specified register over SPI
////////////////////////////////////////////////////////////////////////////
// regAddr - register address from the lookup table
// regData - data to be written to the register
////////////////////////////////////////////////////////////////////////////
void ADIS16460::regWrite(uint8_t regAddr,uint16_t regData) {
#ifdef __SAM3X8E__
  configSPI();
  // Write register address and data
  uint16_t addr = (((regAddr & 0x7F) | 0x80) << 8); //Check that the address is 7 bits, flip the sign bit
  uint16_t lowWord = (addr | (regData & 0xFF));
  uint16_t highWord = ((addr | 0x100) | ((regData >> 8) & 0xFF));
  SPI.transfer(10, ((uint8_t)(lowWord >> 8)), SPI_CONTINUE);
  SPI.transfer(10, ((uint8_t)lowWord));

  delayMicroseconds(25);

  SPI.transfer(10, ((uint8_t)(highWord >> 8)), SPI_CONTINUE);
  SPI.transfer(10, ((uint8_t)highWord));
  
  delayMicroseconds(40); //delay to not violate readrate (40us)
#else
  configSPI();
  // Write register address and data
  uint16_t addr = (((regAddr & 0x7F) | 0x80) << 8); //Check that the address is 7 bits, flip the sign bit
  uint16_t lowWord = (addr | (regData & 0xFF));
  uint16_t highWord = ((addr | 0x100) | ((regData >> 8) & 0xFF));
  digitalWrite(_CS, LOW); 
  SPI.transfer((uint8_t)(lowWord >> 8));
  SPI.transfer((uint8_t)lowWord);
  digitalWrite(_CS, HIGH);

  delayMicroseconds(25);

  digitalWrite(_CS, LOW); 
  SPI.transfer((uint8_t)(highWord >> 8));
  SPI.transfer((uint8_t)highWord);
  digitalWrite(_CS, HIGH);
  
  delayMicroseconds(40); //delay to not violate readrate (40us)
#endif

  #ifdef DEBUG
    Serial.print("Wrote 0x");
    Serial.println(regData);
    Serial.print(" to register: 0x");
    Serial.print((unsigned char)regAddr, HEX);
  #endif

}

/////////////////////////////////////////////////////////////////////////////////////////
// Converts accelerometer data output from the sensorRead() function and outputs 
// acceleration in G's
/////////////////////////////////////////////////////////////////////////////////////////
// sensorData - data output from sensorRead()
// return - (float) signed/scaled accelerometer in G's
/////////////////////////////////////////////////////////////////////////////////////////
float ADIS16460::accelScale(int32_t sensorData)
{
  int signedData = 0;
  int isNeg = sensorData & 0x8000;
  if (isNeg == 0x8000) //if the number is negative, scale and sign the output
    signedData = sensorData - 0xFFFF;
  else
    signedData = sensorData; // else return the raw number
  float finalData = signedData * 0.00025; // multiply by accel sensitivity (250uG/LSB)
  return finalData;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Converts gyro data output from the sensorRead() function and outputs gyro rate in deg/sec
/////////////////////////////////////////////////////////////////////////////////////////////
// sensorData - data output from sensorRead()
// return - (float) signed/scaled gyro in degrees/sec
/////////////////////////////////////////////////////////////////////////////////////////
float ADIS16460::gyroScale(int32_t sensorData)
{
  int signedData = 0;
  int isNeg = sensorData & 0x8000;
  if (isNeg == 0x8000) //if the number is negative, scale and sign the output
    signedData = sensorData - 0xFFFF;
  else
    signedData = sensorData;
  float finalData = signedData * 0.005; //multiply by gyro sensitivity (0.005 LSB/dps)
  return finalData;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Converts temperature data output from the sensorRead() function and outputs temperature 
// in degrees Celcius
/////////////////////////////////////////////////////////////////////////////////////////////
// sensorData - data output from sensorRead()
// return - (float) signed/scaled temperature in degrees Celcius
/////////////////////////////////////////////////////////////////////////////////////////
float ADIS16460::tempScale(int32_t sensorData)
{
  int signedData = 0;
  int isNeg = sensorData & 0x8000;
  if (isNeg == 0x8000) //if the number is negative, scale and sign the output
    signedData = sensorData - 0xFFFF;
  else
    signedData = sensorData;
  float finalData = (signedData * 0.05) + 25; //multiply by gyro sensitivity (0.005 LSB/dps)
  return finalData;
}