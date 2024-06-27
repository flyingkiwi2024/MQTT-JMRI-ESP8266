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

#include "Arduino.h"
#include "ChrisRail.h"

unsigned long _currentMillis = millis();
unsigned long timeMillis;// This is maybe not required - to be deleted?

const char* MQTT_ClientID = MQTTCLIENT;
const char* MQTT_username = MQTTUSER; 
const char* MQTT_password = MQTTPASS; 
const char* mqtt_server = "192.168.6.1";

const bool Sensor_Active = 0;
const bool Sensor_Inactive = 1;

//LED & 595 static ints
int ChrisRail::_StdLED[9][3];
int ChrisRail::_595NbyteData[NUM595NCHIPS];  //creates array, 1 byte per board
int ChrisRail::_595NBoardPins[3];
int ChrisRail::_595NLedPins[NUM595NCHIPS * 8][4];
//int ChrisRail::_StdLEDCount;
byte led = 0;

//Turnout and PCA9685 static ints
int ChrisRail::_pca9685Servos[16][5];

WiFiClient ChrisRail::mqtt_wifi_client;
PubSubClient ChrisRail::mqttclient(ChrisRail::mqtt_wifi_client);

// Layout Functions

//Sets up the layout address and starts system
  ChrisRail::ChrisRail(int name)  {
}

//Set up of system, functions to run before loop
void ChrisRail::init() {
  ChrisRail::setup_wifi();    //  Starts Wifi and connects to RPi-JMRI
  ChrisRail::OTA();           //  Sets up OTA capabilities
  ChrisRail::reconnect();     //  initial MQTT connection
}  

//  This is where the main layout processes incoming and outgoing messages as well as buttons/sensors/turnouts etc
void ChrisRail::runLayout(void) {
  _currentMillis = millis();    //  update timer
  ArduinoOTA.handle();          //  Check for OTA inputs
  ChrisRail::MQTT();            //  Checks MQTT is still connected
  ChrisRail::sensorUpdate();    //  checks sensors
 
}
//End of Layout Functions

//MQTT Functions

//Set up Wifi and connect to RPi-JMRI
void ChrisRail::setup_wifi() {

  pinMode(0, OUTPUT);
  pinMode(2, OUTPUT);
  
  delay(10);
  Serial.println();
  Serial.println(MQTT_ClientID);

  // We start by connecting to a WiFi network
  Serial.print("Connecting to " + String(SSID));
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println(" connected. IP address: " + WiFi.localIP().toString());
  digitalWrite(0, HIGH);
}

//This functions reconnects your ESP8266 to your MQTT broker. Change the function below if you want to subscribe to more topics with your ESP8266 
void ChrisRail::reconnect() {  // Loop until we're reconnected
int q;

  while (!mqttclient.connected()) {
    Serial.print("Attempting MQTT connection...");
    mqttclient.setServer(mqtt_server, 1883);
    if (mqttclient.connect(MQTT_ClientID, MQTT_username, MQTT_password)) {
      Serial.println("connected");
      mqttclient.setCallback(ChrisRail::callback);
        
      // // Subscribe or resubscribe to a topic need to come up with a way to subscribe automatically to messages
      Serial.println("Subscribing to topics");
      // subscribe to lights
        for (q=0; q < LEDSTD; q++);{

          Serial.println("Led Number" + std::string(_StdLED[q][1]));          
          char buf[16];
          char buffer[126];
          itoa(_StdLED[q][1], buf, 10);
          sprintf(buffer,"/track/light/""%03d",buf);
          Serial.println(buffer);
          mqttclient.subscribe(buffer);
        }    //  how many LED's have been added to system

        mqttclient.subscribe("track/turnout/500");
        Serial.println("Subscribed to /track/light/ & /track/turnout");
        digitalWrite(2, HIGH);

    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttclient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

//This checks the MQTT connection each loop, if it has disconnected it reconnects automatically.
void ChrisRail::MQTT()  {   
    if (!mqttclient.connected())
    ChrisRail::reconnect();
  
  mqttclient.loop();
}

//This publishes payloads to topics and send to broker
void ChrisRail::publish(String topic, String payload) {
    Serial.println("Publish topic: " + topic + " message: " + payload);
    mqttclient.publish(topic.c_str() , payload.c_str(), true);
}

//This receive messages from broker and send to relevant turnout/light etc
void ChrisRail::callback(char* topic, byte* payload, unsigned int length) {         
char* T[5];  // previous setting 5
  uint8_t k = 0;
 // int tempLEDstate; delete if no error
  ushort sysname;

  char* token = strtok(topic, "/");

  //get topic from topic char array
  if (token) {
    T[k++] = strdup(token);
    token = strtok(NULL, "/");
    while (token) {
      T[k++] = strdup(token);
      token = strtok(NULL, "/");
    }
  }

  char cmessage[length + 1];

  if (strcmp(T[1], "turnout") != 0 && strcmp(T[1], "light") != 0) {  // previous setting 2
    Serial.println("Message error");
    return;
  }

  strncpy(cmessage, (char*)payload, length);
  cmessage[length] = '\0';
  sysname = atoi(T[2]);

  if (strcmp(T[1], "turnout") == 0) {
    Serial.println("Turnout Message");
    ChrisRail::setTurnout(sysname, cmessage);
    

  } else if (strcmp(T[1], "light") == 0) {
    Serial.println("LED message");
    ChrisRail::setLED(sysname, cmessage);
      //
  }
}
//End of MQTT functions

// Arduino OTA functions

// This allows Arduino OTA to function allowing you to update the firmware without connecting a serial cable
void ChrisRail::OTA() {  
ArduinoOTA.onStart([]() {
  Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

// End of Arduino OTA functions

//Sensor functions

//This will add a Sensor connected to a standard pin on the 8266
void ChrisRail::addStdPinSensor(int pin, int senNum) {
  if (_stdPinSensorsCount < 20) {
    _stdPinSensors[_stdPinSensorsCount][0] = pin;     //  GPIO pin on 8266/32
    _stdPinSensors[_stdPinSensorsCount][1] = senNum;  //  JMRI sensor number i.e 100, NOT MS100
    _stdPinSensors[_stdPinSensorsCount][2] = 2;       //  Default value for first start of system
    Serial.println("addStdPinSensor: Sensor added:" + String(pin) + "senNum: " + String(senNum));
    _stdPinSensorsCount++;
    pinMode(pin, INPUT);
  } else {
    Serial.println("addStdPinSensor: Sensor not added, too many in system. Pin:" + String(pin) + "senNum: " + String(senNum));
  }
}
#if defined(NUM165CHIPS)
//This will add a Sensor connected to a  pin on a 74HC165 board 
void ChrisRail::add165PinSensor(int chip, int pin, int senNum) {
  if (_165SensorPinCount < NUM165CHIPS * 8) {
    if (chip < NUM165CHIPS) {
      _165PinSensors[_165SensorPinCount][0] = chip;
      _165PinSensors[_165SensorPinCount][1] = pin;
      _165PinSensors[_165SensorPinCount][2] = senNum;
      _165PinSensors[_165SensorPinCount][3] = 2;  //default start value
      _165SensorPinCount++;


    } else {
      Serial.println("add165PinSensor: item not added, INVALID Chip Number: " + String(chip) + " Increase NUM165CHIPS in user_setup.h");
    }
  } else {
    Serial.println("add165PinSensor: item not added, too many in system. Add more NUM165CHIPS in user_setup.h");
  }
}

void ChrisRail::setup165(int latchPin, int clockPin, int dataPin) {
  _165BoardPins[LATCHPIN] = latchPin;
  _165BoardPins[CLOCKPIN] = clockPin;
  _165BoardPins[DATAPIN] = dataPin;
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, INPUT);
}

void ChrisRail::latch165(int funcLatchPin) {
  digitalWrite(funcLatchPin, HIGH);
  delayMicroseconds(5);  //works... I want the smallest stable value possible
  digitalWrite(funcLatchPin, LOW);
}

byte ChrisRail::get165Byte(int funcDataPin, int funcClockPin) {
  byte registerContent;
  int bitContent;
  for (int idx = 0; idx < 8; idx++) {
    bitContent = digitalRead(funcDataPin);
    if (bitContent == 1) {
      bitWrite(registerContent, idx, 1);
    } else {
      bitWrite(registerContent, idx, 0);
    }
    digitalWrite(funcClockPin, LOW);
    delayMicroseconds(5);  //works... I want the smallest stable value possible
    digitalWrite(funcClockPin, HIGH);
  }
  return registerContent;
}


#endif
//END 74HC165 functions

//This goes through each sensor and looks for a change in state
void ChrisRail::sensorUpdate() {
  int q;
  int senRead;
  
  bool tmpsenRead;
  if (_currentMillis - _sensorDebounceMillis >= SENSORDEBOUNCE) {
    _sensorDebounceMillis = _currentMillis;
    for (q = 0; q < _stdPinSensorsCount; q++) {
      senRead = digitalRead(_stdPinSensors[q][0]);
      if (senRead != _stdPinSensors[q][2]) {  //check if reading has changed
        _stdPinSensors[q][2] = senRead;       //update the sensor status
        if (senRead == Sensor_Active){
            
          String publishMessage = "ACTIVE";
          String topic = SensorTopic + _stdPinSensors[q][1];

          publish(topic, publishMessage);
        }
        else
          if (senRead == Sensor_Inactive){
                      
          String publishMessage = "INACTIVE";
          String topic = SensorTopic + _stdPinSensors[q][1];

          publish(topic, publishMessage);
        }
      }
    }
  #if defined(NUM165CHIPS) //Now its the 74HC165 registers
  {
  digitalWrite(0, HIGH);
  delayMicroseconds(5);  //works... I want the smallest stable value possible
  digitalWrite(0, LOW);
  }
    for (q = 0; q < NUM165CHIPS; q++) {
      _165byteData[q] = shiftIn(_165BoardPins[DATAPIN], _165BoardPins[CLOCKPIN], LSBFIRST);
    }
    for (q = 0; q < _165SensorPinCount; q++) {  //work through the array of sensors
      senRead = bitRead(_165byteData[_165PinSensors[q][0]], _165PinSensors[q][1]);

      if (senRead != _165PinSensors[q][3]) {
        _165PinSensors[q][3] = senRead;
        if (senRead == Sensor_Active){
            
          String publishMessage = "ACTIVE";
          String topic = SensorTopic + _165PinSensors[q][1];

          publish(topic, publishMessage);
        }
        else
          if (senRead == Sensor_Inactive){
                      
          String publishMessage = "INACTIVE";
          String topic = SensorTopic + _165PinSensors[q][1];

          publish(topic, publishMessage);
        }
      }
    }
#endif
  }
}

// End of sensor functions

//Standard LED Functions

//Adds LED's to system using board pins only
void ChrisRail::addStdLED(int pin, int LEDNum, int dir) {
  if (_StdLEDCount < 20 ) {
    _StdLED[_StdLEDCount][0] = pin;      //  GPIO pin on 8266/32
    _StdLED[_StdLEDCount][1] = LEDNum;   //  JMRI number of LED
    _StdLED[_StdLEDCount][2] = dir;    //  last state of LED
    Serial.println("addStdLED:LED added: " + String(pin) + " LEDNum:" + String(LEDNum));
        pinMode(pin, OUTPUT);  //put all GPIO's used for LED's as OUTPUT

    _StdLEDCount++;

  } else {
    Serial.println("addStdLED: LED not added, too many in system. Pin:" + String(pin) + "LEDNum: " + String(LEDNum));
  }
}

//adds an led connected to a 74HC595N pin
void ChrisRail::add595NLedPins(int chip, int pin, int accNum, int dir) {
  if (_595NLedPinCount < NUM595NCHIPS * 8) {
    _595NLedPins[_595NLedPinCount][0] = chip;       // Which board the LED is on in the daisy chain - remember starts at 0
    _595NLedPins[_595NLedPinCount][1] = pin;        // Pin number on board
    _595NLedPins[_595NLedPinCount][2] = accNum;     // JMRI accessory number
    _595NLedPins[_595NLedPinCount][3] = dir;        // holder for last state of LED
    Serial.println("add595NLedPins:LED added: Board " + String(chip) + String(pin) + " accNum:" + String(accNum));
    _595NLedPinCount++;
  } else {
    Serial.println("ERROR:add595NLed to many LED's - add more boards");
  }
}

//74HC595N Shift register functions
#if defined(NUM595NCHIPS)
//Set up pins for 74HC595N control
void ChrisRail::setup595N(int latchPin, int clockPin, int dataPin) {  // Sets up the pins
  _595NBoardPins[LATCHPIN] = latchPin;
  _595NBoardPins[CLOCKPIN] = clockPin;
  _595NBoardPins[DATAPIN] = dataPin;
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  Serial.println(_595NBoardPins[LATCHPIN]);
  Serial.println(_595NBoardPins[DATAPIN]);
  Serial.println(_595NBoardPins[CLOCKPIN]);
  //Set chips to start up values
  ChrisRail::update595N();
}
void ChrisRail::update595N() {          
  digitalWrite(_595NBoardPins[LATCHPIN], LOW);
  for (int q = NUM595NCHIPS; q > 0; q--) {  //cycle through the chips v0_3 fix was cycling in wrong order
    shiftOut(_595NBoardPins[DATAPIN], _595NBoardPins[CLOCKPIN], MSBFIRST, led);

  }
  digitalWrite(_595NBoardPins[LATCHPIN], HIGH);
}
#endif

//Receives message from callback and turns LED on or off
void ChrisRail::setLED(ushort sysname, char* cmessage){ 
  char    newstate  = 'N';
  int     q;
  byte myBitSet;

  Serial.println("LED Message to be processed");
  Serial.println(sysname);
  Serial.println(cmessage);
  for (q = 0; q < 16; q++) {
    if (_StdLED[q][1] == sysname)           //compare name from message to LED name
      if( strcmp(cmessage, "OFF")==0 ){      //action for message OFF
        newstate = 'F';
        digitalWrite(_StdLED[q][0], LOW);
        }
      if( strcmp(cmessage, "OFF")!=0 ){                                //action for message ON
        newstate = 'N';
        digitalWrite(_StdLED[q][0], HIGH);
        }
  }
      #if defined(NUM595NCHIPS)
          myBitSet = 0;
          for (q = 0; q < 16; q++) {
            if (_595NLedPins[q][2] == sysname) {
              //set the bit in the byte array
              if( strcmp(cmessage, "ON")==0 ){
                bitSet(led, _595NLedPins[q][1]);
                Serial.println("it works");

              } else {
                bitClear(_595NbyteData[_595NLedPins[q][0]], _595NLedPins[q][1]);
              }
              myBitSet = 1;  //set a flag that change has been made
            }
          }
          //do this after cycling through all items
          if (myBitSet > 0) {  //if I set somethiing update the shift registers
            ChrisRail::update595N();
          }
          #endif
}

// End of Standard LED Functions

// Turnout Functions

//adds servos into system
#if defined(MAXPCA9685SERVOBOARDS)
void ChrisRail::addPCA9685Servo(byte board, byte port, int accNum, int angle0, int angle1) {
  byte validServo = 0;
  byte existingBoard = 0;
  int q;
  _pca9685Servos[_pca9685ServoCount][0] = board;
  _pca9685Servos[_pca9685ServoCount][1] = port;
  _pca9685Servos[_pca9685ServoCount][2] = accNum;
  _pca9685Servos[_pca9685ServoCount][3] = angle0;
  _pca9685Servos[_pca9685ServoCount][4] = angle1;

  if (port < 0 || port > 15) {
    Serial.println("ERROR:addPCA9685Servo invalid port address");
    validServo = 1;
  }
  for (q = 0; q < _pca9685AddressesCount; q++) {
    if (_pca9685Addresses[q] == _pca9685Servos[_pca9685ServoCount][0]) {
      existingBoard = 1;
    }
  }

  if (existingBoard < 1 && _pca9685AddressesCount < MAXPCA9685SERVOBOARDS) {

    _pca9685Addresses[_pca9685AddressesCount] = byte(_pca9685Servos[_pca9685ServoCount][0]);  //store new board address in array
    //now set it up as a servo board
    if (_pca9685AddressesCount < 1) {
      Wire.begin();
      delay(200);  //won't matter in start up, allow wire to get started
    }
    ChrisRail::setupPCA9685Board(_pca9685Addresses[_pca9685AddressesCount]);
    _pca9685AddressesCount++;
  } else {
    Serial.println("ERROR:addPCA9685Servo to0 many boards increase MAXPCA9685SERVOBOARDS in user_setup.h");
  }

  if (validServo < 1) {
    _pca9685ServoCount++;
  }
}

void ChrisRail::setupPCA9685Board(byte boardAddress) {
  int error;             //use of you want to check status
  int pca9685Freq = 50;  //50 hz
  Serial.print("boardAddress: ");
  Serial.println(boardAddress, HEX);
  //set up stuff
  Wire.beginTransmission(boardAddress);  //Start transmissions to board
  Wire.write(0x00);                      //the adress on the board to write to...MODE Register 1
  Wire.write(0x21);                      //sets auto increment to next location...0x21 = 00100001  bit 5 set to 1 = auto increment bit 0 = 1 responds to all led call
  error = Wire.endTransmission();

  //not really needed as this just resets the default configuration for MODE 2...see data sheet
  Wire.beginTransmission(boardAddress);  //Start transmissions to board
  Wire.write(0x01);                      //the address on the board to write to.... MODE register 2
  Wire.write(0x04);                      //sets 2nd variable to write to...........0100  INVT value to use when no externa; driver used.
  error = Wire.endTransmission();

  long reqHZ = pca9685Freq;
  int preScale = 25000000 / (4096 * reqHZ) - 1;
  if (preScale < 3) {  //Min prescale value
    preScale = 3;
  }
  if (preScale > 255) {  //Max prescale value
    preScale = 255;
  }
  Serial.print("preScale: ");  //
  Serial.println(preScale);    //Should come out at about 121 for servos

  //Put board to sleep
  Wire.beginTransmission(boardAddress);  //Start transmissions to board
  Wire.write(0x00);                      //the adress on the board to write to...MODE Register 1
  Wire.write(0b01000000);                //sets auto increment to next location...0x21 = 00100001  bit 5 set to 1 = auto increment bit 0 = 1 responds to all led call
  error = Wire.endTransmission();
  Serial.print("sleep: ");
  Serial.println(error);

  //change preScale
  Wire.beginTransmission(boardAddress);  //Start transmissions to board
  Wire.write(0xFE);                      //the address on the board to write to...MODE Register 1
  Wire.write(preScale);                  //sets auto increment to next location...0x21 = 00100001  bit 5 set to 1 = auto increment bit 0 = 1 responds to all led call
  error = Wire.endTransmission();
  Serial.print("prescale: ");
  Serial.println(error);

  //return board to awake and the state I want.
  Wire.beginTransmission(boardAddress);  //Start transmissions to board
  Wire.write(0x00);                      //the adress on the board to write to...MODE Register 1
  Wire.write(0x21);                      //sets auto increment to next location...0x21 = 00100001  bit 5 set to 1 = auto increment bit 0 = 1 responds to all led call
  error = Wire.endTransmission();
}

void ChrisRail::setPCA695Servo(byte boardAddress, byte port, byte angle) {  //addr = PCA9685 address 0x40, byte port 0-15, int angle...angle to set servo to
  int sendPulse;                                                          //pulse to send to board
  int error;                                                              //stores return value if you want to test it.
  if (port < 16) {
    if (angle > 179) {  //don't allow out of range angles
      angle = 179;
    }
    sendPulse = map(angle, 0, 179, SERVOMIN, SERVOMAX);
    //Serial.println(sendPulse);
    Wire.beginTransmission(boardAddress);  //Start transmissions to board
    Wire.write((port * 4) + 6);            //6 = port 0.............               sets the start transmission address
    Wire.write(0x00);                      //pulse start time..using fixed on time... writes date to addrsss and increments...start time low byte
    Wire.write(0x00);                      //pulse start time                         writes date to addrsss and increments.... high byte...both zero in this example
    Wire.write(byte(sendPulse));           //value between 0 and 4095..writes date to addrsss and increments...finish time low byte
    Wire.write(byte(sendPulse >> 8));      //high byte              writes date to addrsss and increments...finish time high byte byte
    error = Wire.endTransmission();        //end transmission, check if any errores occured with the writes...0 = good news.
    //Serial.println(error);//print return value 0 = OK
  } else {
    Serial.println("invalid PCA9685 port Max = 15");
  }
}
void  ChrisRail::setTurnout(ushort sysname, char* cmessage){
  int q;
  Serial.println("Turnout Message to be processed");
  Serial.println(sysname);
  Serial.println(cmessage);

  for (q = 0; q < 16; q++) {
    if (_pca9685Servos[q][2] == sysname) {
      if ( strcmp(cmessage, "THROWN")==0 ){
        ChrisRail::setPCA695Servo(_pca9685Servos[q][0],_pca9685Servos[q][1],_pca9685Servos[q][3]);
        } else {
        ChrisRail::setPCA695Servo(_pca9685Servos[q][0], _pca9685Servos[q][1], _pca9685Servos[q][4]);
        }
      }
    }
  }
#endif
//Receives message from callback and throws or closes turnout
//static void  setTurnout(ushort sysname, char* message); 

// End of Turnout Functions

//This will go through each button on 74HC165 registers and check for an input

//Adds buttons to system
void ChrisRail::add165PinButton(int chip, int pin, int accNum, int press1, int press2) {  //74HC165 pin/button
  if (_165PinCount < NUM165CHIPSNUMBUTTONS) {
    if (chip < NUM165CHIPS) {
      _165PinButtons[_165PinCount][0] = chip;
      _165PinButtons[_165PinCount][1] = pin;
      _165PinButtons[_165PinCount][2] = accNum;
      _165PinButtons[_165PinCount][3] = press1;
      _165PinButtons[_165PinCount][4] = press2;
      _165PinCount++;
    } else {
      Serial.println("add165PinButton: item not added, INVALID Chip Number: " + String(chip) + " Increase NUM165CHIPS in user_setup.h");
    }
  } else {
    Serial.println("add165PinButton: item not added, too many in system. Increase NUM165CHIPSNUMBUTTONS in user_setup.h");
  }
}

void ChrisRail::buttonsPressed(void) {
  int q;
  byte senRead;
  if (_currentMillis - _buttonDebounceMillis >= BUTTONDEBOUNCE) {
    _buttonDebounceMillis = _currentMillis;
    // May add this function at a later date if required
    // if (nowMomentButton)
    //   nowMomentButton();  //check momentary buttons

    // //Now works through buttons added in addStdPinButton(int pin,int accNum,int press1,int press2)
    // for (q = 0; q < _stdPinButtonsCount; q++) {     //work though the pins
    //   if (digitalRead(_stdPinButtons[q][0]) > 0) {  //if the pin is being pressed
    //     if (_stdPinButtons[q][4] > 2) {
    //       _stdPinButtons[q][4] = 2;  //set to target for this button press
    //     } else {
    //       _stdPinButtons[q][4] = 3;  //set to target for this button press
    //     }
    //     //Now process the button
    //     nowRail::sendAccessoryCommand(_stdPinButtons[q][1], _stdPinButtons[q][_stdPinButtons[q][4]], MESSRESPREQ);  //1 response required
    //   }
    }
  #if defined(NUM165CHIPS) //Now it processes the 74HC165 registers

    ChrisRail::latch165(_165BoardPins[LATCHPIN]);
    for (q = 0; q < NUM165CHIPS; q++) {

      _165byteData[q] = shiftIn(_165BoardPins[DATAPIN], _165BoardPins[CLOCKPIN], LSBFIRST);
    }
    for (q = 0; q < _165PinCount; q++) {  //work through the array

      if (bitRead(_165byteData[_165PinButtons[q][0]], _165PinButtons[q][1]) == 1) {
        if (_165PinButtons[q][5] > 3) {
          _165PinButtons[q][5] = 3;  //set to target for this button press
        } else {
          _165PinButtons[q][5] = 4;  //set to target for this button press
        }
        //sendAccessoryCommand(_CD4021PinButtons[q][2], _CD4021PinButtons[q][_CD4021PinButtons[q][5]], MESSRESPREQ);  //1 response required
      }
    }
#endif
  }



