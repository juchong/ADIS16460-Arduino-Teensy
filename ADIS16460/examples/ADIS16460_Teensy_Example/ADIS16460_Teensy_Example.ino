////////////////////////////////////////////////////////////////////////////////////////////////////////
//  September 2016
//  Author: Juan Jose Chong <juan.chong@analog.com>
////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ADIS16460_Teensy_Example.ino
////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
//  This Arduino project interfaces with an ADIS16460 using SPI and the 
//  accompanying C++ libraries, reads IMU data in LSBs, scales the data, and 
//  outputs measurements to a serial debug terminal (PuTTY) via the onboard 
//  USB serial port.
//
//  This project has been tested on a PJRC 32-Bit Teensy 3.2 Development Board, 
//  but should be compatible with any other embedded platform with some modification.
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
float SXDVEL, SYDVEL, SZDVEL = 0;

// Control Registers
int MSC = 0;
int FLTR = 0;
int DECR = 0;

// Temperature
int TEMP = 0;
float TEMPS = 0;

// Delay counter variable
int printCounter = 0;

// Call ADIS16460 Class
ADIS16460 IMU(10,2,6); // Chip Select, Data Ready, Reset Pin Assignments

void setup()
{
    Serial.begin(115200); // Initialize serial output via USB
    IMU.configSPI(); // Configure SPI communication
    delay(1000); // Give the part time to start up
    IMU.regWrite(MSC_CTRL, 0xC1);  // Enable Data Ready, set polarity
    delay(20); 
    IMU.regWrite(FLTR_CTRL, 0x500); // Set digital filter
    delay(20);
    IMU.regWrite(DEC_RATE, 0), // Disable decimation
    delay(20);

    // Read the control registers once to print to screen
    MSC = IMU.regRead(MSC_CTRL);
    FLTR = IMU.regRead(FLTR_CTRL);
    DECR = IMU.regRead(DEC_RATE);

    attachInterrupt(2, grabData, RISING); // Attach interrupt to pin 2. Trigger on the rising edge
}

// Function used to read register values via SPI and load them into variables in LSBs
void grabData()
{
    // Put all the Data Registers you want to read here
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

// Main loop. Scale and display registers read using the interrupt
void loop()
{
    printCounter ++;
    if (printCounter >= 200000) // Delay for writing data to the serial port
    {
        detachInterrupt(2); //Detach interrupt to avoid overwriting data
        scaleData(); // Scale data acquired from the IMU

        //Clear the serial terminal and reset cursor
        //Only works on supported serial terminal programs (Putty)
        Serial.print("\033[2J");
        Serial.print("\033[H");
        Serial.println(" ");

        // Print header
        Serial.println("ADIS16460 Teensy Example Program");
        Serial.println("Juan Chong - September 2016");
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

        printCounter = 0;
        attachInterrupt(2, grabData, RISING);
    }
}
