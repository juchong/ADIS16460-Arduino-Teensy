////////////////////////////////////////////////////////////////////////////////////////////////////////
//  January 2015
//  By: Juan Jose Chong
////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ADIS16460.ino
////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
//  This Arduino project interfaces with an ADIS16460 using SPI, reads IMU data, scales, and outputs
/// data to a serial debug terminal (Putty).
//
//  This project has been tested on an Arduino Duemilanove, but should be compatible with any other
//  Arduino embedded platform.
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
#include <ADIS16460.h>
#include <SPI.h>

//Variable Declarations
//Accelerometer
int AX = 0;
int AY = 0;
int AZ = 0;
float AXS = 0;
float AYS = 0;
float AZS = 0;
//Gyro
int GX = 0;
int GY = 0;
int GZ = 0;
float GXS = 0;
float GYS = 0;
float GZS = 0;
//Control Registers
int MSC = 0;
int SENS = 0;
int SMPL = 0;
//Temperature
float TEMPS = 0;
int TEMP = 0;

boolean NewData = false;

//Pinout for Arduino Uno
//Gray = RST = 4
//Purple = SCLK = 13
//Blue = CS = 7
//Green = DOUT(MISO) = 11
//Yellow = DIN(MOSI) = 12
//Brown = GND
//Red = VCC

//Call ADIS16460 Class
ADIS16460 IMU(7,2,4); //CS,DR,RST

void setup()
{
  Serial.begin(115200);
  #ifdef DEBUG
    Serial.println("**********DEBUG MODE**********");
  #endif
  delay(100); //Give the part time to start up
  IMU.regWrite(MSC_CTRL,0x6);  //Enable Data Ready
  delay(100); 
  IMU.regWrite(SENS_AVG,0x402); //Set Digital Filter
  delay(100);
  IMU.regWrite(SMPL_PRD,0x601), //Set Decimation
  delay(100);
  
  //Read the control registers once to print to screen
  MSC = IMU.regRead(MSC_CTRL);
  SENS = IMU.regRead(SENS_AVG);
  SMPL = IMU.regRead(SMPL_PRD);
  
  attachInterrupt(0,grabData,LOW); //Attach interrupt
}

//Interrupt-driven function used to grab register values and load them into variables
void grabData()
{
  //Put all the Data Registers you want to read here
  detachInterrupt(0);
  TEMP = 0;
  GX = IMU.regRead(XGYRO_OUT);
  GY = IMU.regRead(YGYRO_OUT);
  GZ = IMU.regRead(ZGYRO_OUT);
  AX = IMU.regRead(XACCL_OUT);
  AY = IMU.regRead(YACCL_OUT);
  AZ = IMU.regRead(ZACCL_OUT);
  TEMP = IMU.regRead(TEMP_OUT);
  NewData = !NewData;
  attachInterrupt(0,grabData,LOW);
}

//Scale and display registers read using the interrupt
void loop()
{
  //Print Control Registers
  Serial.println("Control Registers");
  Serial.print("MSC_CTRL: ");
  Serial.println((unsigned char)MSC,HEX);
  Serial.print("SENS_AVG: ");
  Serial.println((unsigned char)SENS,HEX);
  Serial.print("SMPL_PRD: ");
  Serial.println((unsigned char)SMPL,HEX);
  Serial.println(" ");
  Serial.println("Data Registers");
  
  //If interrupt has fired and updated the registers with new data, scale the data and set the NewData flag
  if (NewData == true){
    detachInterrupt(0); //Detatch interrupt while writing to the screen
    GXS = IMU.gyroScale(GX); //Scale X Gyro
    GYS = IMU.gyroScale(GY); //Scale Y Gyro
    GZS = IMU.gyroScale(GZ); //Scale Z Gyro
    AXS = IMU.accelScale(AX); //Scale X Accel
    AYS = IMU.accelScale(AY); //Scale Y Accel
    AZS = IMU.accelScale(AZ); //Scale Z Accel
    TEMPS = IMU.tempScale(TEMP); //Scale Temp Sensor
    NewData = !NewData;  //Set New Data Flag
  }
  //Print Gyro Data
  Serial.print("XGYRO: ");
  Serial.println(GXS);
  Serial.print("YGYRO: ");
  Serial.println(GYS);
  Serial.print("ZGYRO: ");
  Serial.println(GZS);

  //Print Accel Data
  Serial.print("XACCL: ");
  Serial.println(AXS);
  Serial.print("YACCL: ");
  Serial.println(AYS);
  Serial.print("ZACCL: ");
  Serial.println(AZS);
 
  //Print Temp Data
  Serial.print("TEMP: ");
  Serial.println(TEMPS);
  
  attachInterrupt(0,grabData,LOW);
 
  delay(100);
  
  //Clear serial out and reset cursor
  //Only works on supported serial terminal programs (Putty)
  Serial.print("\033[2J");
  Serial.print("\033[H");
}
