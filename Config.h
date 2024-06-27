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

#ifndef CREDENTIALS_h
#define CREDENTIALS_h

#define   VERSION             4.0  //ChrisRail Software version
//wifi ssid and password.
#define   SSID        "RPi-JMRI"                  
#define   PASSWORD    "rpI-jmri" 

//MQTT server, port and ESP8266 or ESP32 device name

#define   MQTTCLIENT  "8266_test"   // Change this for each node if issues connecting to MQTT broker
#define   MQTTSERVER  "192.168.6.1"
#define   MQTTPORT    "1883"

//MQTT Broker username and password. Leave blank if unsecured
#define   MQTTUSER    "ChrisRail"
#define   MQTTPASS    "Rail123"

#define   DEFAULTMQTTTOPIC    "/track/#"                  //default topic

//SHIFT REGISTERS
#define LATCHPIN 14
#define CLOCKPIN 13
#define DATAPIN 12


/* pin mapping ESP8266
 * D0 -> 16 (GPO) No Interrupt (No input)
 * D1 -> 5  (GPIO & PWM)
 * D2 -> 4  (GPIO & PWM)  
 * D3 -> 0  (GPO & PWM) (No input)
 * D4 -> 2  (GPO & PWM) (No input)
 * D5 -> 14 (GPIO & PWM) 
 * D6 -> 12 (GPIO & PWM)
 * D7 -> 13 (GPIO & PWM)
 * D8 -> 15 (GPO & PWM) (No input)
 * I2C 
 * (PCF8574) 0-7  (GPIO - no PWM)
 * (PCF8575) 0-15 (GPIO - no PWM)
 * (PCA9685) 0-15 (Output only PWM+digital)
 */
 
//map break out board pin labels to ESP8266 pins d0-d8
            
#endif