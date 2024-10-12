/*
 * Подключение HiTechnic Color Sensor v1.5 & v2 к Arduino
 * (c) Serg Kosachenko
 * Подключение к разъему NXT -> Arduino
   white -> 
   black -> 
   red -> battery – (GND) 
   green -> Arduino +3.3V 
   yellow -> Arduino SCL (A5)
   blue -> Arduino SDA (A4)
   
 * ToDo:
 * 50 Hz operating 0x35
 * 60 Hz operating 0x36
 */


#include <Wire.h>

int htColorSensorAddress = 0x01; //we got this from I2C Scanner
int TestValue;
byte ColorRed, ColorGreen, ColorBlue;

/**
 * This define returns the smaller of the two numbers
 */
#define min2(a, b) (a < b ? a : b)

/**
 * This define returns the smallest of the three numbers
 */
#define min3(a, b, c) (a < b) ? ((a < c) ? a : c) : ((b < c) ? b : c)

/**
 * This function returns the bigger of the two numbers
 */
#define max2(a, b) (a > b ? a : b)

/**
 * This function returns the biggest of the three numbers
 */
#define max3(a, b, c) (a > b) ? ((a > c) ? a : c) : ((b > c) ? b : c)

/**
 * Returns x if it is between min and max. If outside the range,
 * it returns min or max.
 */
#define clip(a, b, c) min2(c, max2(b, a))


void setup() {
  Serial.begin(9600); //sets up Serial monitor
  Serial.println("Start...");

  //The code below turns on i2c, opens communication to color
  //sensor, and does a test connection.
  Wire.begin(); //turn on i2c
  Wire.beginTransmission(htColorSensorAddress); //open communication to sensor 
  Wire.write(0x00); //blank packet
  byte error = Wire.endTransmission(); //close communication
  if (error == 0){
    Serial.print("I2C device found at address 0x");
    if (htColorSensorAddress < 16) {
        Serial.print("0");
      }
      Serial.println(htColorSensorAddress, HEX);
    }
    else
    {
    Serial.println("I2C device not found!");
      }
  while(Wire.available() > 0)
     Serial.print(Wire.read());
     ReadVersionSensor();
}

void loop(){
  float H,S,V;
  
  ReadColorSensor();
  //Serial.print("Color number: ");
  //Serial.println(TestValue);
  
  ReadRGBSensor();
/*  Serial.print("ColorRed: ");
  Serial.print(ColorRed);
  Serial.print(" ColorGreen: ");
  Serial.print(ColorGreen);
  Serial.print(" ColorBlue: ");
  Serial.println(ColorBlue);*/

  RGBtoHSV((float) ColorRed, (float) ColorGreen, (float) ColorBlue, H, S, V);
  Serial.print("Hue: ");
  Serial.print(H);
  Serial.print(" Saturation: ");
  Serial.print(S);
  Serial.print(" Value: ");
  Serial.println(V);
  
  delay(500); //delay for reading output
}

void ReadColorSensor(){
  //this code lets the arduino know you want to talk to the HT Color Sensor
  Wire.beginTransmission(htColorSensorAddress);
  Wire.write(0x42); // color number
  Wire.endTransmission();

  //the code below requests 1 bytes of information 
  //to make a number 0-17 that it saves in the variable TestValue
  Wire.requestFrom(htColorSensorAddress, 1); 
   while(Wire.available() < 1);
   TestValue = Wire.read();
}

void ReadVersionSensor(){
  char c;
  Wire.beginTransmission(htColorSensorAddress);
  Wire.write(0x00); // version sensor
  Wire.endTransmission();

  //the code below requests 24 bytes of information 
  Wire.requestFrom(htColorSensorAddress, 24); 
   while(Wire.available() < 24);
   for (int i=0; i<24;i++){
   c = Wire.read();
   Serial.print(c);
   }
   Serial.println("");
}

void ReadRGBSensor(){
  Wire.beginTransmission(htColorSensorAddress);
  Wire.write(0x43); // Red+Green+Blue
  Wire.endTransmission();

  //the code below requests 3 bytes of information RGB
  Wire.requestFrom(htColorSensorAddress, 3); 
   while(Wire.available() < 3);
   ColorRed = Wire.read();
   ColorGreen = Wire.read();
   ColorBlue = Wire.read();
}

/**
 * Convert RGB colors to HSV
 * @param red the red input value
 * @param green the green input value
 * @param blue the blue input value
 * @param hue the hue output value (from 0 to 365, or -1 if n/a)
 * @param sat the saruration output value (from 0 to 100, or -1 if n/a)
 * @param value the value output value (from 0 to 100)
 * @return void
 */
void RGBtoHSV(float red, float green, float blue, float &hue, float &sat, float &value)
{
  hue = 0;
  sat = 0;
  value = 0;

  //   Value
  float rgb_max = max3(red, green, blue);
  float rgb_min;
  value = rgb_max / 2.56;
  if (value == 0){
    hue = -1;
    sat = -1;
    return;
  }

  //   Saturation
  red /= rgb_max;
  green /= rgb_max;
  blue /= rgb_max;

  rgb_max = max3(red, green, blue);
  rgb_min = min3(red, green, blue);
  sat = (rgb_max - rgb_min) * 100;
  if (sat == 0){
    hue = -1;
    return;
  }

  //   Hue

  red = (red - rgb_min) / (rgb_max - rgb_min);
  green = (green - rgb_min) / (rgb_max - rgb_min);
  blue = (blue - rgb_min) / (rgb_max - rgb_min);

  rgb_max = max3(red, green,blue);
  rgb_min = min3(red, green,blue);

  if (rgb_max == red){
    hue = 0.0 + 60.0*(green-blue);
    if (hue < 0.0){
      hue += 360.0;
    }
  } else if (rgb_max == green){
    hue = 120.0 + 60.0 * (blue-red);
  } else {
    hue = 240.0 + 60.0 * (red-green);
  }
}
