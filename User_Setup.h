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

 //Uncomment items that are required for your layout.

#ifndef USER_SETUP_h
#define USER_SETUP_h

#pragma once

//PCA9685 - Output for Servo driven turnouts
#define MAXPCA9685SERVOBOARDS 5 //default is 5, increase if required
#define SERVOMIN 450    //servo min value
#define SERVOMAX 2000   //servo max value

#define LEDSTD  2
#define LED595  3 

//74HC595N - Outputs for LEDs, lights etc
#define NUM595NCHIPS   1   //Number of 74HC595N shift registers daisy chained on board (output i.e. LED's)

//74HC165 - Inputs for Sensors, buttons etc
#define NUM165CHIPS     2   //Number of 74HC165 shift registers daisy chained on board (input i.e. sensors)
#define NUM165CHIPSNUMBUTTONS 16 

#define SENSORDEBOUNCE 100  //Sensors checked every 0.1 seconds 

#define BUTTONDEBOUNCE 250  //Set the button debounce speed to 0.25 seconds...increasing value reduces double presses but too high and system is sluggish

//char* LEDTopic = "track/light";

// topics for publishing sensor data will be built in setup()
const String SensorTopic  = "track/sensor/"; 
//char LEDTopic     = "track/light/";
const String TurnoutTopic = "track/turnout/";
#endif