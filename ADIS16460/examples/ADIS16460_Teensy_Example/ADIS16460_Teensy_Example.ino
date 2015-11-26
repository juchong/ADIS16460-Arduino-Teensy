////////////////////////////////////////////////////////////////////////////////////////////////////////
//  May 2015
//  Author: Juan Jose Chong <juan.chong@analog.com>
////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ADIS16448.ino
////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
//  This Arduino project interfaces with an ADIS16448 using SPI and the accompanying C++ libraries, 
//  reads IMU data in LSBs, scales the data, and outputs measurements to a serial debug terminal (putty) via
//  the onboard USB serial port.
//
//  This project has been tested on an Arduino Duemilanove and Uno, but should be compatible with any other
//  8-Bit Arduino embedded platform. 
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
//  Pinout for Arduino Uno/Diecimila/Duemilanove
//  RST = DIO6
//  SCLK = DIO13
//  CS = DIO10
//  DOUT(MISO) = DIO12
//  DIN(MOSI) = DIO11
//  DR = DIO2
//
////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <ADIS16460.h>
#include <SPI.h>

// Initialize Variables
// Accelerometer
int AX = 0;
int AY = 0;
int AZ = 0;
float AXS = 0;
float AYS = 0;
float AZS = 0;
// Gyro
int GX = 0;
int GY = 0;
int GZ = 0;
float GXS = 0;
float GYS = 0;
float GZS = 0;
// Control Registers
int MSC = 0;
int FLTR = 0;
int DECR = 0;
// Temperature
float TEMPS = 0;
int TEMP = 0;

// Data Ready Flag
boolean validData = false;

// Call ADIS16460 Class
ADIS16460 IMU(10,2,6); //ChipSelect,DataReady,Reset Pin Assignments

void setup()
{
  Serial.begin(115200); // Initialize serial output via USB
  IMU.configSPI(); // Configure SPI communication
  
  #ifdef DEBUG
    Serial.println("**********DEBUG MODE**********");
  #endif
  
  delay(500); // Give the part time to start up
  IMU.regWrite(MSC_CTRL, 0xC1);  // Enable Data Ready on IMU
  delay(20); 
  IMU.regWrite(FLTR_CTRL, 0x00); // Set Digital Filter on IMU
  delay(20);
  IMU.regWrite(DEC_RATE, 0xFF), // Set Decimation on IMU
  delay(20);
  
  // Read the control registers once to print to screen
  MSC = IMU.regRead(MSC_CTRL);
  FLTR = IMU.regRead(FLTR_CTRL);
  DECR = IMU.regRead(DEC_RATE);
  
  attachInterrupt(2, setDRFlag, RISING); // Attach interrupt to pin 2. Trigger on the rising edge
  
}

// Function used to read register values via SPI and load them into variables in LSBs
void grabData()
{
    // Put all the Data Registers you want to read here
    TEMP = 0;
    IMU.configSPI(); // Configure SPI before the read
    GX = IMU.regRead(X_GYRO_OUT);
    GY = IMU.regRead(Y_GYRO_OUT);
    GZ = IMU.regRead(Z_GYRO_OUT);
    AX = IMU.regRead(X_ACCL_OUT);
    AY = IMU.regRead(Y_ACCL_OUT);
    AZ = IMU.regRead(Z_ACCL_OUT);
    TEMP = IMU.regRead(TEMP_OUT);
}

// Function used to scale all acquired data (scaling functions are included in ADIS16460.cpp)
void scaleData()
{
    GXS = IMU.gyroScale(GX); //Scale X Gyro
    GYS = IMU.gyroScale(GY); //Scale Y Gyro
    GZS = IMU.gyroScale(GZ); //Scale Z Gyro
    AXS = IMU.accelScale(AX); //Scale X Accel
    AYS = IMU.accelScale(AY); //Scale Y Accel
    AZS = IMU.accelScale(AZ); //Scale Z Accel
    TEMPS = IMU.tempScale(TEMP); //Scale Temp Sensor
}

// Data Ready Interrupt Routine
void setDRFlag()
{
  validData = !validData;
}

// Main loop. Scale and display registers read using the interrupt
void loop()
{
  if (validData) // If data present in the ADIS16460 registers is valid...
  {
    grabData(); // Grab data from the IMU
    
    scaleData(); // Scale data acquired from the IMU
    
    //Print control registers to the serial port
    Serial.println("Control Registers");
    Serial.print("MSC_CTRL: ");
    Serial.println((unsigned char)MSC,HEX);
    Serial.print("FLTR_CTRL: ");
    Serial.println((unsigned char)FLTR,HEX);
    Serial.print("DEC_RATE: ");
    Serial.println((unsigned char)DECR,HEX);
    Serial.println(" ");
    Serial.println("Data Registers");
    
    //Print scaled gyro data
    Serial.print("XGYRO: ");
    Serial.println(GXS);
    Serial.print("YGYRO: ");
    Serial.println(GYS);
    Serial.print("ZGYRO: ");
    Serial.println(GZS);
  
    //Print scaled accel data
    Serial.print("XACCL: ");
    Serial.println(AXS);
    Serial.print("YACCL: ");
    Serial.println(AYS);
    Serial.print("ZACCL: ");
    Serial.println(AZS);
    
    //Print scaled temp data
    Serial.print("TEMP: ");
    Serial.println(TEMPS);
   
    delay(150); // Give the user time to read the data
    
    //Clear the serial terminal and reset cursor
    //Only works on supported serial terminal programs (Putty)
    Serial.print("\033[2J");
    Serial.print("\033[H");
  }
}
