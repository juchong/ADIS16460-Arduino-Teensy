////////////////////////////////////////////////////////////////////////////////////////////////////////
//  December 2015
//  Author: Juan Jose Chong <juan.chong@analog.com>
////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ADIS16460.ino
////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
//  This Arduino project interfaces with an ADIS16460 using SPI and the accompanying C++ libraries, 
//  reads IMU data in LSBs, scales the data, and outputs measurements to a serial debug terminal (putty) via
//  the onboard USB serial port.
//
//  This project has been tested on a PJRC 32-Bit Teensy 3.2 Development Board, but should be compatible 
//  with any other embedded platform with some modification.
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
//  Pinout for a Teensy 3.2 Development Board
//  RST = D6
//  SCK = D13/SCK
//  CS = D10/CS
//  DOUT(MISO) = D12/MISO
//  DIN(MOSI) = D11/MOSI
//  DR = D2
//
////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <ADIS16460.h>
#include <SPI.h>

// Initialize Variables
// Accelerometer
int AX, AY, AZ = 0;
float AXS, AYS, AZS = 0;

// Gyro
int GX, GY, GZ = 0;
float GXS, GYS, GZS = 0;

//Delta Angle
int XDANGL, YDANGL, ZDANGL = 0;
float SXDANGL, SYDANGL, SZDANGL = 0;

//Delta Velocity
int XDVEL, YDVEL, ZDVEL = 0;
int SXDVEL, SYDVEL, SZDVEL = 0;

// Control Registers
int MSC = 0;
int FLTR = 0;
int DECR = 0;
// Temperature
int TEMP = 0;
float TEMPS = 0;

// Data Ready Flag
boolean validData = false;

// Call ADIS16460 Class
ADIS16460 IMU(10,2,6); // Chip Select, Data Ready, Reset Pin Assignments

void setup()
{
  Serial.begin(115200); // Initialize serial output via USB
  IMU.configSPI(); // Configure SPI communication
   
  delay(2000); // Give the part time to start up
  IMU.regWrite(MSC_CTRL, 193);  // Enable Data Ready, set polarity
  delay(20); 
  IMU.regWrite(FLTR_CTRL, 1280); // Set Digital Filter (Range = 5, Estimation time = 32 sec)
  delay(20);
  IMU.regWrite(DEC_RATE, 0), // Disable Decimation
  delay(100);
  
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
    XDANGL = IMU.regRead(X_DELT_ANG);
    YDANGL = IMU.regRead(Y_DELT_ANG);
    ZDANGL = IMU.regRead(Z_DELT_ANG);
    XDVEL = IMU.regRead(X_DELT_VEL);
    YDVEL = IMU.regRead(Y_DELT_VEL);
    ZDVEL = IMU.regRead(Z_DELT_VEL);
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
    SXDANGL = IMU.deltaAngleScale(XDANGL);
    SYDANGL = IMU.deltaAngleScale(YDANGL);
    SZDANGL = IMU.deltaAngleScale(ZDANGL);
    SXDVEL = IMU.deltaVelocityScale(XDVEL);
    SYDVEL = IMU.deltaVelocityScale(YDVEL);
    SZDVEL = IMU.deltaVelocityScale(ZDVEL);
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

    // Print header
    Serial.println("ADIS16460 Teensy Example Program");
    Serial.println("Juan Chong - December 2015");
    Serial.println(" ");

    //Print control registers to the serial port
    Serial.println("Control Registers");
    Serial.print("MSC_CTRL: ");
    Serial.println(MSC,HEX);
    Serial.print("FLTR_CTRL: ");
    Serial.println(FLTR,HEX);
    Serial.print("DEC_RATE: ");
    Serial.println(DECR,HEX);
    Serial.println(" ");
    Serial.println("Raw Output Registers");
    
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
    Serial.println(" ");

    Serial.println("Delta Angle/Velocity Registers");

    //Print scaled delta angle data
    Serial.print("X ANGLE: ");
    Serial.println(SXDANGL);
    Serial.print("Y ANGLE: ");
    Serial.println(SYDANGL);
    Serial.print("Z ANGLE: ");
    Serial.println(SZDANGL);

    //Print scaled delta velocity data
    Serial.print("X VEL: ");
    Serial.println(SXDVEL);
    Serial.print("Y VEL: ");
    Serial.println(SYDVEL);
    Serial.print("Z VEL: ");
    Serial.println(SZDVEL);
    
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
