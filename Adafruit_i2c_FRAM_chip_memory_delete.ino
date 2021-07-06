//Include Header File Statements ----------------------------------------------------
#include <fstream>
#include <iostream>
#include <cmath>
#include <math.h>
#include <stdio.h>
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "Adafruit_FRAM_I2C.h"
#include <Adafruit_BMP280.h>
Adafruit_BMP280 bmp;// I2C

Adafruit_FRAM_I2C fram     = Adafruit_FRAM_I2C();

void setup(void) {
  Serial.begin(115200);
  
  if (fram.begin()) {
    Serial.println("Found I2C FRAM");
  } else {
    Serial.println("I2C FRAM not identified ... check your connections?\r\n");
    Serial.println("Will continue in case this processor doesn't support repeated start\r\n");
  }

  uint8_t value;
  for (uint16_t n = 0; n < 32761; n++) {

    fram.write8(n, 0);
    Serial.print(F("cleared data pos "));
    Serial.println(n);
    
  }
  Serial.println(F("data reset complete"));

}




void loop() {
}
