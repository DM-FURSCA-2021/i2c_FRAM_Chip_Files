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
//#include <SPI.h>

/*INCLUDE FULL FILE PATHWAY IF THE PROGRAM CANNOT FIND THE ARDUINOPID.h FILE*/
#include "Full_Flight_Program.h"

Adafruit_FRAM_I2C fram     = Adafruit_FRAM_I2C();

short servoPos1 = 0;
short servoPos2 = 0;

void setup(void) {
  Serial.begin(115200);
  
  if (fram.begin()) {
    Serial.println("Found I2C FRAM");
  } else {
    Serial.println("I2C FRAM not identified ... check your connections?\r\n");
    Serial.println("Will continue in case this processor doesn't support repeated start\r\n");
  }
  

  uint8_t value;
  flightStage = 2;
  
  //while (flightStage == 2 or flightStage == 0) {
  for (short n = 0; n <= motor_time+2; n++){
    
    //22bit interpreter
    thetaX = fram.read8(universalIndex*22+1) + .01*fram.read8(universalIndex*22+2);
    if (fram.read8(universalIndex*22+3) == 1) {
      thetaX = thetaX*(-1);
    }
    thetaY[0] = fram.read8(universalIndex*22+4) + .01*fram.read8(universalIndex*22+5);
    if (fram.read8(universalIndex*22+6) == 1) {
      thetaY[0] = thetaY[0]*(-1);
    }
    thetaZ[0] = fram.read8(universalIndex*22+7) + .01*fram.read8(universalIndex*22+8);
    if (fram.read8(universalIndex*22+9) == 1) {
      thetaZ[0] = thetaZ[0]*(-1);
    }

    flightStage = fram.read8(universalIndex*22);

    gimbal_angleDecimalY = fram.read8(universalIndex*22+11);
    gimbal_angleDecimalZ = fram.read8(universalIndex*22+13);

    if (gimbal_angleDecimalY >= 100) {
      gimbal_angleDecimalY = (gimbal_angleDecimalY-100);
      gimbal_angleY[0] = -1*(fram.read8(universalIndex*22+12) + .01*gimbal_angleDecimalY);
    }
    else {
      gimbal_angleY[0] = fram.read8(universalIndex*22+10) + .01*gimbal_angleDecimalY;
    }
    
    if (gimbal_angleDecimalZ >= 100) {
      gimbal_angleDecimalZ = (gimbal_angleDecimalZ-100);
      gimbal_angleZ[0] = -1*(fram.read8(universalIndex*22+12) + .01*gimbal_angleDecimalZ);
    }
    else {
      gimbal_angleZ[0] = fram.read8(universalIndex*22+12) + .01*gimbal_angleDecimalZ;
    }


    servoPos1 = fram.read8(universalIndex*22+14) -90;
    servoPos2 = fram.read8(universalIndex*22+15) -90;

    altitude = 100*fram.read8(universalIndex*22+16) + fram.read8(universalIndex*22+17) + .01*fram.read8(universalIndex*22+18);
    pressure = 90000+100*fram.read8(universalIndex*22+19) + fram.read8(universalIndex*22+20) + .01*fram.read8(universalIndex*22+21);

//    Serial.print(F("\t "));
    Serial.print(universalIndex);
//    Serial.print(F(" flightStage= "));
    Serial.print(F(" \t "));
    Serial.print(flightStage);
//    Serial.print(F(" thetaX= "));
    Serial.print(F(" \t "));
    Serial.print(thetaX);
//    Serial.print(F(" thetaY= "));
    Serial.print(F(" \t "));
    Serial.print(thetaY[0]);
//    Serial.print(F(" thetaZ= "));
    Serial.print(F(" \t "));
    Serial.print(thetaZ[0]);
    
//    Serial.print(F("gimbal_angleY= "));
    Serial.print(F(" \t "));
    Serial.print(gimbal_angleY[0]);
//    Serial.print(F(" gimbal_angleZ= "));
    Serial.print(F(" \t "));
    Serial.print(gimbal_angleZ[0]);
//    Serial.print(F(" servoPos1= "));
    Serial.print(F(" \t "));
    Serial.print(servoPos1);
//    Serial.print(F(" servoPos2= "));
    Serial.print(F(" \t "));
    Serial.print(servoPos2);
//    Serial.print(F(" altitude= "));
    Serial.print(F(" \t "));
    Serial.print(altitude);
//    Serial.print(F(" pressure= "));
    Serial.print(F(" \t "));
    Serial.println(pressure);
    
    universalIndex++;
  }

  poweredFlightIndex = universalIndex;
  universalIndex = 0;
  flightStage = 3;

  //for (short n = 0; n <= 100; n++){
  while (flightStage == 3 or flightStage == 4) {
    
    //16bit interpreter
    thetaX = fram.read8(poweredFlightIndex*22 + universalIndex*16+1) + .01*fram.read8(poweredFlightIndex*22 + universalIndex*16+2);
    if (fram.read8(poweredFlightIndex*22 + universalIndex*16+3) == 1) {
      thetaX = thetaX*(-1);
    }
    thetaY[0] = fram.read8(poweredFlightIndex*22 + universalIndex*16+4) + .01*fram.read8(poweredFlightIndex*22 + universalIndex*16+5);
    if (fram.read8(poweredFlightIndex*22 + universalIndex*16+6) == 1) {
      thetaY[0] = thetaY[0]*(-1);
    }
    thetaZ[0] = fram.read8(poweredFlightIndex*22 + universalIndex*16+7) + .01*fram.read8(poweredFlightIndex*22 + universalIndex*16+8);
    if (fram.read8(poweredFlightIndex*22 + universalIndex*16+9) == 1) {
      thetaZ[0] = thetaZ[0]*(-1);
    }

    flightStage = fram.read8(poweredFlightIndex*22 + universalIndex*16);

    altitude = 100*fram.read8(poweredFlightIndex*22 + universalIndex*16+10) + fram.read8(poweredFlightIndex*22 + universalIndex*16+11) + .01*fram.read8(poweredFlightIndex*22 + universalIndex*16+12);
    pressure = 90000+100*fram.read8(poweredFlightIndex*22 + universalIndex*16+13) + fram.read8(poweredFlightIndex*22 + universalIndex*16+14) + .01*fram.read8(poweredFlightIndex*22 + universalIndex*16+15);

    if (flightStage !=5) {
  //    Serial.print(F("\t "));
      Serial.print(poweredFlightIndex + universalIndex);
  //    Serial.print(F(" flightStage= "));
      Serial.print(F(" \t "));
      Serial.print(flightStage);
  //    Serial.print(F(" thetaX= "));
      Serial.print(F(" \t "));
      Serial.print(thetaX);
  //    Serial.print(F(" thetaY= "));
      Serial.print(F(" \t "));
      Serial.print(thetaY[0]);
  //    Serial.print(F(" thetaZ= "));
      Serial.print(F(" \t "));
      Serial.print(thetaZ[0]);
      
  //    Serial.print(F(" altitude= "));
      Serial.print(F(" \t "));
      Serial.print(altitude);
  //    Serial.print(F(" pressure= "));
      Serial.print(F(" \t "));
      Serial.println(pressure);
    }

    universalIndex++;
  }

  unpoweredFlightIndex = universalIndex-1;
  universalIndex = 0;
  flightStage = 5;
  
  while(flightStage == 5) {

    //7bit interpreter
    flightStage = fram.read8(poweredFlightIndex*22 + unpoweredFlightIndex*16 + universalIndex*7);

    altitude = 100*fram.read8(poweredFlightIndex*22 + unpoweredFlightIndex*16 + universalIndex*7+1) + fram.read8(poweredFlightIndex*22 + unpoweredFlightIndex*16 + universalIndex*7+2) + .01*fram.read8(poweredFlightIndex*22 + unpoweredFlightIndex*16 + universalIndex*7+3);
    pressure = 90000+100*fram.read8(poweredFlightIndex*22 + unpoweredFlightIndex*16 + universalIndex*7+4) + fram.read8(poweredFlightIndex*22 + unpoweredFlightIndex*16 + universalIndex*7+5) + .01*fram.read8(poweredFlightIndex*22 + unpoweredFlightIndex*16 + universalIndex*7+6);

//    Serial.print(F("\t "));
    Serial.print(poweredFlightIndex + unpoweredFlightIndex + universalIndex);
//    Serial.print(F(" flightStage= "));
    Serial.print(F(" \t "));
    Serial.print(flightStage);
//    Serial.print(F(" altitude= "));
    Serial.print(F(" \t "));
    Serial.print(altitude);
//    Serial.print(F(" pressure= "));
    Serial.print(F(" \t "));
    Serial.println(pressure);
    


    universalIndex++;
  }
    

}




void loop() {
}
