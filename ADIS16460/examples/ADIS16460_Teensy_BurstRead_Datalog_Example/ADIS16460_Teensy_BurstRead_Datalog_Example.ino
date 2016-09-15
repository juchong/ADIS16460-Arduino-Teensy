////////////////////////////////////////////////////////////////////////////////////////////////////////
//  September 2016
//  Author: Juan Jose Chong <juan.chong@analog.com>
////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ADIS16460_Teensy_BurstRead_Example.ino
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
// Temporary Data Array
int16_t *burstData;

// Checksum variable
int16_t burstChecksum = 0;

// Scaled data array
float scaledData[7];

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
    attachInterrupt(2, grabData, RISING); // Attach interrupt to pin 2. Trigger on the rising edge
}

// Function used to read register values when an ISR is triggered using the IMU's DataReady output
void grabData()
{
    burstData = IMU.burstRead(); // Read data and insert into array
    burstChecksum = IMU.checksum(burstData); // Calculate checksum based on data array
    scaleData(); // Scale sensor output data
    printtoserial(); // Print data to the serial port
}

// Print burst data to serial port. Data output rate is determined by the IMU decimation rate
void printtoserial()
{
    Serial.print((*(burstData + 0))); // DIAG_STAT
    Serial.print(",");
    Serial.print(scaledData[0]); // Scaled XGYRO
    Serial.print(",");
    Serial.print(scaledData[1]); // Scaled YGYRO
    Serial.print(",");
    Serial.print(scaledData[2]); // Scaled ZGYRO
    Serial.print(",");
    Serial.print(scaledData[3]); // Scaled XACCL
    Serial.print(",");
    Serial.print(scaledData[4]); // Scaled YACCL
    Serial.print(",");
    Serial.print(scaledData[5]); // Scaled ZACCL
    Serial.print(",");
    Serial.print(scaledData[6]); // Scaled TEMP
    Serial.print(",");
    Serial.print((*(burstData + 8))); // DIAG_STAT
    Serial.print(",");
    Serial.print((*(burstData + 9))); // CHECKSUM
    Serial.print(",");
    if (burstChecksum == *(burstData + 9)) // 1 = Valid Checksum, 0 = Invalid Checksum
        Serial.println("1");
    else
        Serial.println("0");
}

// Function used to scale all acquired data (scaling functions are included in ADIS16460.cpp)
void scaleData()
{
    scaledData[0] = IMU.gyroScale(*(burstData + 1)); //Scale X Gyro
    scaledData[1] = IMU.gyroScale(*(burstData + 2)); //Scale Y Gyro
    scaledData[2] = IMU.gyroScale(*(burstData + 3)); //Scale Z Gyro
    scaledData[3] = IMU.accelScale(*(burstData + 4)); //Scale X Accel
    scaledData[4] = IMU.accelScale(*(burstData + 5)); //Scale Y Accel
    scaledData[5] = IMU.accelScale(*(burstData + 6)); //Scale Z Accel
    scaledData[6] = IMU.tempScale(*(burstData + 7)); //Scale Temp Sensor
}

// Main loop
void loop()
{
    // Nothing to do here! The program is entirely interrupt-driven!
}
