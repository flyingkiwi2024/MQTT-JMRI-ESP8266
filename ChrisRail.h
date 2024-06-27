/*
 *  © 2024 Chris Hall
 *  All rights reserved.
 *
 *  This file is part of ChrisRail
 *
 *  This is free software: you can redistribute it and/or modify
 *  as you wish.
 *
 *  It is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
 */

 #ifndef CHRISRAIL_h
 #define CHRISRAIL_h

 #include "Arduino.h"

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include "Config.h"
#include "User_Setup.h"

#if defined(MAXPCA9685SERVOBOARDS)
#include <Wire.h>
#endif


class ChrisRail {
  public:

  ChrisRail(int name);
  static void          init();                                                     //Set up of system,  adds functions to run before loop
  void          runLayout();                                                //This is where the main layout processes incoming and outgoing messages as well as buttons/sensors/turnouts etc
  static void   setup_wifi();                                               //Set up Wifi and connect to RPi-JMRI
  static void   reconnect();                                                //This functions reconnects your ESP8266 to your MQTT broker. Change the function below if you want to subscribe to more topics with your ESP8266 
  static void   MQTT();                                                     //This checks the MQTT connection each loop, if it has disconnected it reconnects automatically.
  static void   OTA();                                                      //This allows Arduino OTA to function allowing you to update the firmware without connecting a serial cable
  static void   publish(String topic, String payload);                      //This publishes payloads to topics and send to broker
  static void   callback(char* topic, byte* payload, unsigned int length);  //This receive messages from broker and send to relevant turnout/light etc


  void          addStdPinSensor(int pin, int senNum);                       //This will add a Sensor connected to a standard pin on the 8266
  void          addStdLED(int pin, int LEDNum, int dir);                    //Adds LED's to system using board pins only
  void          buttonsPressed(void);                                       //Checks for button presses on 74HC165's
  
  #if defined(NUM595NCHIPS)
  void          setup595N(int latchPin, int clockPin, int dataPin);         // Sets up the pins
  void          add595NLedPins(int chip, int pin, int accNum, int dir);     //Adds a panel LED connected to 74HC595N
  static void   update595N();
  #endif

  #if defined(NUM165CHIPS)
  void          setup165(int latchPin, int clockPin, int dataPin);                       // Sets up the pins
  void          add165PinButton(int chip, int pin, int accNum, int press1, int press2);  //Adds std pin accessories to system
  void          add165PinSensor(int chip, int pin, int senNum);
  #endif

  #if defined(MAXPCA9685SERVOBOARDS)
  void addPCA9685Servo(byte board, byte port, int accNum, int angle0, int angle1);
  #endif
    
  private:

  // MQTT   
  static PubSubClient       mqttclient; 
  static WiFiClient         mqtt_wifi_client;
  

  //Sensors & buttons
  unsigned long _sensorDebounceMillis;
  unsigned long _buttonDebounceMillis;
  void sensorUpdate(void);                                                   //This goes through each sensor and looks for a change in state
  int _stdPinSensors[20][3];  //  pin, senNum, LastState
  byte _stdPinSensorsCount;   //  counts how mant Std pin sensors on board
  //75HC165 Shift Registers for buttons/sensors...system to work like _stdPinButtons
  #if defined(NUM165CHIPS) //75HC165 Shift Registers for buttons/sensors...system to work like _stdPinButtons
  
  int _165PinButtons[NUM165CHIPSNUMBUTTONS][6];  //allows for multiple triggers per pin)items> chip,pin,accid,press 1, press 2,buttonstate
  int _165PinCount;                                 //how many buttons have been added to system...needs to be int
  byte _165byteData[NUM165CHIPS];
  int _165BoardPins[3];                              //latch pin, clockpin, datapin
  static void latch165(int funcLatchPin);                     //latch pin function
  byte get165Byte(int funcDataPin, int funcClockPin);  //gets the byte of data
  //74HC165 sensors
  int _165PinSensors[NUM165CHIPS * 8][4];  //chip, pin, senNum, LastState
  int _165SensorPinCount; //sensor pins added
  #endif

  //LEDs
  static void setLED(ushort sysname, char* message);                         // Receives message from callback and turns LED on or off
  static int _StdLED[9][3];   //  max 20 //only 20 allowed...pin LEDNum Diritems>
  int _StdLEDCount;    //  how many LED's have been added to system
  #if defined(NUM595NCHIPS)   //75HC595 shift registers - allows 8 LED's per board
  //static void update595N(void);
  static int _595NbyteData[NUM595NCHIPS];  //creates array, 1 byte per board
  static int _595NBoardPins[3];
  static int _595NLedPins[NUM595NCHIPS * 8][4];  //chip, pin, accNUm, dir
  byte _595NLedPinCount;                   //keeps track of LED pins added
 
  #endif


  //Turnouts
  #if defined(MAXPCA9685SERVOBOARDS)
  static int _pca9685Servos[16][5];  //board, port, accNum, angle0, angle1
  int _pca9685ServoCount;
  byte _pca9685AddressesCount;
  byte _pca9685Addresses[MAXPCA9685SERVOBOARDS];
  void setupPCA9685Board(byte boardAddress);                      //sets up servo boards
  static void setPCA695Servo(byte boardAddress, byte port, byte angle);  //move servo to an angle
  static void  setTurnout(ushort sysname, char* message);                   //Receives message from callback and throws or closes turnout
  #endif
};
#endif

