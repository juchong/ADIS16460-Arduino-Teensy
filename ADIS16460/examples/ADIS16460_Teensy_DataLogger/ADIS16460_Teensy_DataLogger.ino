////////////////////////////////////////////////////////////////////////////////////////////////////////
//  July 2015
//  Author: Juan Jose Chong <juan.chong@analog.com>
////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ADIS16448.ino
////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
//  This Arduino project interfaces with an ADIS16448 using SPI and the accompanying C++ libraries, 
//  reads IMU data in LSBs, scales the data (if enabled), and outputs measurements to a serial debug 
//  terminal (putty) via the onboard USB serial port. 
//
//  The data output is limited to 102.4 Hz (0x301) on 8-bit processors due to hardware limitations.
//
//  This project has been tested on an Arduino Duemilanove and Uno, but should be compatible with any other
//  8-Bit Arduino embedded platform. 
//
//  If the #define SCALE is uncommented, the program will output scaled data to the serial port. Otherwise,
//  it will output data as 16 bit signed hex. The data format is:
//
//  ACCEL_X,ACCEL_Y,ACCEL_Z,GYRO_X,GYRO_Y,GYRO_Z,MAG_X,MAG_Y,MAG_Z
//
//  **NOTE** Printing in HEX requires the use of printf(). Details on modifying the Arduino IDE
//  to include these functions can be found here: http://playground.arduino.cc/Main/Printf
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
//  Gray = RST = 4
//  Purple = SCLK = 13
//  Blue = CS = 7
//  Yellow = DOUT(MISO) = 11
//  Green = DIN(MOSI) = 12
//  Black = GND
//  Red = VCC [3.3V ONLY]
//  White = DR = 2
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <ADIS16448.h>
#include <SPI.h>

// Uncomment to print scaled data
//#define SCALE

// Call ADIS16448 Class
ADIS16448 IMU(7,2,4); // (ChipSelect,DataReady,Reset) Pin Assignments

void setup()
{
  Serial.begin(115200); // Initialize serial output via USB
  IMU.configSPI(); // Configure SPI communication to IMU  
  delay(100);
  IMU.regWrite(MSC_CTRL,0x06);  // Enable Data Ready on IMU
  delay(20); 
  IMU.regWrite(SENS_AVG,0x400); // Set Digital Filter on IMU
  delay(20);
  IMU.regWrite(SMPL_PRD,0x301), // Set Decimation on IMU (MAX = 102.4 Hz = 0x301)
  delay(20);
  
  attachInterrupt(0, grabData, RISING); // Attach interrupt to pin 2. Trigger on the rising edge
}

// Function used to read register values via SPI and load them into variables in LSBs
void grabData()
{
    // Put all the Data Registers you want to read here
    int16_t * data; // Instantiate data array
    //IMU.configSPI(); // Configure SPI before the read - only required if multiple SPI slaves are on bus
    data = IMU.sensorRead(); // Read predefined sensors

#ifndef SCALE

  // Grab data from pointer and place into array
  int16_t intData[8];
  for(int i = 0; i < 9; i++){
    intData[i] = data[i];
  }
  // Print data to serial port **Requires the use of printf()**
  // See: http://playground.arduino.cc/Main/Printf
  Serial.printf("%3X",intData[3]); //XACCEL
  Serial.printf(",");
  Serial.printf("%3X",intData[4]); //YACCEL
  Serial.printf(",");
  Serial.printf("%3X",intData[5]); //ZACCEL
  Serial.printf(",");
  Serial.printf("%3X",intData[0]); //XGYRO
  Serial.printf(",");
  Serial.printf("%3X",intData[1]); //YGYRO
  Serial.printf(",");
  Serial.printf("%3X",intData[2]); //ZGYRO
  Serial.printf(",");
  Serial.printf("%3X",intData[6]); //XMAG
  Serial.printf(",");
  Serial.printf("%3X",intData[7]); //YMAG
  Serial.printf(",");
  Serial.printf("%3X",intData[8]); //ZMAG
  Serial.println(" ");
}

#endif

#ifdef SCALE

  float scaledData[8];
  // Scale sensor data and place data into array of floats
  for(int i = 0; i < 3; i++){
    scaledData[i] = IMU.gyroScale(data[i]);
  }
  for(int i = 0; i < 3; i++){
    scaledData[i + 3] = IMU.accelScale(data[i+3]);
  }
  for(int i = 0; i < 3; i++){
    scaledData[i + 6] = IMU.magnetometerScale(data[i+6]);
  }
  
  // Print data to serial port
  Serial.print(scaledData[3]); //XACCEL
  Serial.print(",");
  Serial.print(scaledData[4]); //YACCEL
  Serial.print(",");
  Serial.print(scaledData[5]); //ZACCEL
  Serial.print(",");
  Serial.print(scaledData[0]); //XGYRO
  Serial.print(",");
  Serial.print(scaledData[1]); //YGYRO
  Serial.print(",");
  Serial.print(scaledData[2]); //ZGYRO
  Serial.print(",");
  Serial.print(scaledData[6]); //XMAG
  Serial.print(",");
  Serial.print(scaledData[7]); //YMAG
  Serial.print(",");
  Serial.print(scaledData[8]); //ZMAG
  Serial.println(" ");
}

#endif
    
// Main loop
void loop()
{
  // Nothing to do here! The program is entirely interrupt driven which ensures each sample is recorded.
}
