# ADIS16460 Teensyduino (Arduino for Teensy) Demo
### An example C++ library and Teensyduino project for the ADIS16460 iSensor Six Degrees of Freedom Inertial Sensor

This example library was written to give engineers, students, and makers a starting point for using a high-performance, compact, precision inertial sensor. The code in this repository will provide the user with:
- A header file listing all of the unit's available registers
- Functions for reading output registers and writing control registers using **8-bit** frames
    - Note that the ADIS16460 requires 16 bit SPI transactions. spi.transfer() is called twice for each transfer and CS is manually toggled to overcome the Arduino language's limitation 
- Functions for performing common routines such as resetting the sensor
- Burst-mode data acquisition and checksum verification
- Example Arduino sketches which synchronously read data from the sensor and write it to the serial port

### What do I need to get started?

- In order to compile and execute the Teensyduino sketch, you'll need to download the Arduino package (v1.6.11 as of this writing). You can download the IDE [here](https://www.arduino.cc/en/Main/Software).
- You'll also need to install the Teensyduino [library](https://www.pjrc.com/teensy/td_download.html) provided by PJRC.
- Finally, you'll need a Teensy sold by PJRC [here](https://www.pjrc.com/store/teensy32.html). Version 3.x or LC is supported.
- The main Teensyduino sketch issues a command to clear the terminal window after displaying data. For best results, connect to your Teensy using [PuTTY](http://www.chiark.greenend.org.uk/~sgtatham/putty/download.html), an open source terminal program.

### How do I install the library?

Once you've installed the Arduino IDE and Teensyduino libraries, copy the ADIS16460 folder into `My Documents > Arduino > libraries`

Be sure to restart the Arduino IDE to refresh the library directory!

### How do I connect the IMU to my Arduino?

**If using a Teensy, the onboard regulator should provide enough current for the ADIS16460 to properly operate.**

You'll need to build a cable to interface the sensor with the [ADIS16IMU4/PCBZ](http://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/EVAL-ADIS16IMU4.html#eb-overview). The image below shows a custom Teensy interface board for iSensors products.

![ADIS16460-Teensy Interface Board](https://raw.githubusercontent.com/juchong/ADIS16460-Arduino-Teensy/master/ADIS16460/images/teensy_interface.JPG)

Pin assignments for the Teensy can be found in the example sketch comments.

### How do I know it's working?

Once you have the sensor connected and have opened the **ADIS16460_Teensy_BurstRead_Example.ino** example sketch, use PuTTY to connect to the arduino using the following settings. Note that your COM port may be different:

![ADIS16460 Example PuTTY Config](https://raw.githubusercontent.com/juchong/ADIS16209-Arduino-Demo/master/setup_pictures/PuTTYConfig.PNG)

If everything is working, you should see a screen like this:

![ADIS16460 Example PuTTY Output](https://raw.githubusercontent.com/juchong/ADIS16460-Arduino-Teensy/master/ADIS16460/images/burst_demo.PNG)

The demo software will only update the screen ~2 times/second, but every sample is being captured by the interrupt service routine.

A single burst frame should look like this:
![ADIS16460 Burst Frame](https://raw.githubusercontent.com/juchong/ADIS16460-Arduino-Teensy/master/ADIS16460/images/burst_frame_capture.PNG)
