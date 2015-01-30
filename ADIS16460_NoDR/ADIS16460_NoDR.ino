#include <ADIS16460.h>
#include <SPI.h>

float AX = 0;
float AY = 0;
float AZ = 0;

float GX = 0;
float GY = 0;
float GZ = 0;

int MSC = 0;

float TEMP = 0;

//Pinout for Arduino Uno
//Gray = RST = 4
//Purple = SCLK = 13
//Blue = CS = 7
//Green = DOUT(MISO) = 11
//Yellow = DIN(MOSI) = 12
//Brown = GND
//Red = VCC

ADIS16460 IMU(7,6,4); //CS,DR,RST (Hardware SPI)

void setup()
{
  Serial.begin(115200);
  #ifdef DEBUG
    Serial.println("**********DEBUG MODE**********");
  #endif
  delay(100); //This delay is required to allow the part to fully initialize
  IMU.regWrite(MSC_CTRL,0x06); //Enable Data Ready
}

void loop()
{
  MSC = IMU.regRead(MSC_CTRL);
  Serial.print("MSC_CTRL: ");
  Serial.println((unsigned char)MSC,HEX);
  
  GX = IMU.regRead(XGYRO_OUT);
  GX = IMU.gyroScale(GX);
  GY = IMU.regRead(YGYRO_OUT);
  GY = IMU.gyroScale(GY);
  GZ = IMU.regRead(ZGYRO_OUT);
  GZ = IMU.gyroScale(GZ);
  Serial.print("XGYRO: ");
  Serial.println(GX,BIN);
  Serial.print("YGYRO: ");
  Serial.println(GY,BIN);
  Serial.print("ZGYRO: ");
  Serial.println(GZ,BIN);
  
  
  AX = IMU.regRead(XACCL_OUT);
  AX = IMU.accelScale(AX);
  AY = IMU.regRead(YACCL_OUT);
  AY = IMU.accelScale(AY);
  AZ = IMU.regRead(ZACCL_OUT);
  AZ = IMU.accelScale(AZ);
  Serial.print("XACCL: ");
  Serial.println(AX);
  Serial.print("YACCL: ");
  Serial.println(AY);
  Serial.print("ZACCL: ");
  Serial.println(AZ);
  
  
  TEMP = IMU.regRead(TEMP_OUT);
  TEMP = IMU.tempScale(TEMP);
  Serial.print("TEMP: ");
  Serial.println(TEMP);
  
  delay(250);
  
  //Clear serial out and reset cursor
  Serial.print("\033[2J");
  Serial.print("\033[H");

}
