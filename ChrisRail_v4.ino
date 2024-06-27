///////////////////////////////////////////////////////////////////////////
//  ChrisRail - MQTT-JMRI client for Chris's Railway
//
//  This file is the main sketch for the ESP8266 or ESP32 client
///////////////////////////////////////////////////////////////////////////

#include "ChrisRail.h"

ChrisRail myLayout(8266);

unsigned long currentMillis = millis();
int timeTimer = 1000;  //once per second

void setup() {

  Serial.begin(115200);
  Serial.println("Starting setup");

  myLayout.addStdLED(13, 100, 0);   //D7
  myLayout.addStdLED(15, 101, 0);   //D8
  //myLayout.addStdPinSensor(14, 200);  //D5
  //myLayout.addStdPinSensor(12, 201);  //D6
 
  myLayout.add595NLedPins(0, 1, 400, 0);
  myLayout.add595NLedPins(0, 2, 401, 0);
  myLayout.add595NLedPins(0, 3, 402, 0);
  //myLayout.add165PinSensor(0, 1, 250);
  //myLayout.add165PinButton(0, 3, 500, 1, 0);
  
  myLayout.addPCA9685Servo(0x40, 1 ,500, 45, 135);
  
  myLayout.init();
}

void loop() {

  myLayout.runLayout();
}