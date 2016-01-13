/**
* [TI2726-B] Embedded Software
* Code for line following robot.
* 
* This code is responsible for the following actions:
*  - Controlling the motors via PWM and timers.
*  - Listen to Twist messages for movement.
*  - Measure if an object is blocking the path, and stopping if needed.
*  - Stop when no Twist messages have been received in some time, often
*      often indicating loss of connection to the main station.
*
* For specifications, see: 
*   http://www.st.ewi.tudelft.nl/~koen/ti2726-b/robot-manual.pdf
*

* Created by:
* Group number: 31
* Student 1:
*   Bart van Oort, 4343255
*
* Student 2:
*   Steven Meijer, 4368061
*
* Current config is as follows:
*  W = forward
*  S = backward
*  A = left
*  D = right
*  Z = slow down
*  X = speed up
*  F = stop
*
*/
#include <stdlib.h>
#include <ros.h>
#include <ArduinoHardware.h>
#include <SimpleTimer.h>

#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

SimpleTimer timer;

int LEFT_FWD = 6;        // LCHB-100 H Bridge 1FWD
int LEFT_BCK = 7;        // LCHB-100 H Bridge 1REV
int LEFT_ENABLE = 24;    // LCHB-100 H Bridge 1EN
int RIGHT_FWD = 2;       // LCHB-100 H Bridge 2FWD
int RIGHT_BCK = 3;       // LCHB-100 H Bridge 2REV
int RIGHT_ENABLE = 25;   // LCHB-100 H Bridge 2EN
int BLUETOOTH_TX = 18;   // HC-05 Bluetooth dongle TX
int BLUETOOTH_RX = 19;   // HC-05 Bluetooth dongle RX
int US_ECHO = 22;        // HC-SR04 ultrasonic sensor ECHO
int US_TRIGGER = 23;     // HC-SR04 ultrasonic sensor TRIGGER
int LED = 13;            // Yellow LED light

// Needed to set up a bluetooth connection to the main station
class NewHardware : public ArduinoHardware {
    public: 
        NewHardware() : ArduinoHardware(&Serial1, 57600) {};
};

void processTwist(const geometry_msgs::Twist& msg) {
  //timer.restartTimer(0);
  
  updateBot(msg.linear.x, msg.angular.z);
}

// Adds a new subscriber
ros::NodeHandle_<NewHardware> nh;
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &processTwist);

void enableBot() {
	digitalWrite(LEFT_ENABLE, HIGH);
	digitalWrite(RIGHT_ENABLE, HIGH);
}

void disableBot() {
	digitalWrite(LEFT_ENABLE, LOW);
	digitalWrite(RIGHT_ENABLE, LOW);
}

void stopBot() {
	analogWrite(LEFT_FWD, 0);
	analogWrite(RIGHT_FWD, 0);
	analogWrite(LEFT_BCK, 0);
	analogWrite(RIGHT_BCK, 0);
}

void updateBot(double lineair_x, double angular_z) {
  if (lineair_x == 0.0 && angular_z == 0.0) return;
	
    if (lineair_x > 0.0 && angular_z == 0.0) {
	analogWrite(LEFT_BCK, 0);
	analogWrite(RIGHT_BCK, 0);
	analogWrite(LEFT_FWD, 25.5 * lineair_x);
	analogWrite(RIGHT_FWD, 25.5 * lineair_x);
    } else if (lineair_x < 0.0 && angular_z == 0.0) {
	analogWrite(LEFT_FWD, 0);
	analogWrite(RIGHT_FWD, 0);
	analogWrite(LEFT_BCK, 25.5 * abs(lineair_x));
	analogWrite(RIGHT_BCK, 25.5 * abs(lineair_x));
    } else if (lineair_x == 0.0 && angular_z > 0.0) {
	// TODO Tweak values
	analogWrite(LEFT_FWD, 25.5 * angular_z);
	analogWrite(RIGHT_FWD, 0);
	analogWrite(LEFT_BCK, 0);
	analogWrite(RIGHT_BCK, 25.5 * angular_z);
    } else if (lineair_x == 0.0 && angular_z < 0.0) {
	// TODO Tweak values
	analogWrite(LEFT_FWD, 0);
	analogWrite(RIGHT_FWD, 25.5 * abs(angular_z));
	analogWrite(LEFT_BCK, 25.5 * abs(angular_z));
	analogWrite(RIGHT_BCK, 0);
    }
}

void setup() {
  Serial.println("Initializing Arduino");
  Serial.println("     Wuq©");
  
  nh.initNode();
  nh.subscribe(sub);
  enableBot();
  
  pinMode(LEFT_FWD, OUTPUT);
  pinMode(LEFT_BCK, OUTPUT);
  pinMode(LEFT_ENABLE, OUTPUT);
  pinMode(RIGHT_FWD, OUTPUT);
  pinMode(RIGHT_BCK, OUTPUT);
  pinMode(RIGHT_ENABLE, OUTPUT);
  
  pinMode(BLUETOOTH_TX, INPUT);
  pinMode(BLUETOOTH_RX, INPUT);
  pinMode(US_ECHO, INPUT);
  pinMode(US_TRIGGER, OUTPUT);
  pinMode(LED, OUTPUT);
  
  analogWrite(LEFT_FWD, 0);
  analogWrite(LEFT_BCK, 0);
  analogWrite(RIGHT_FWD, 0);
  analogWrite(RIGHT_BCK, 0);
  
  timer.setTimeout(1000, stopBot);
}

void loop() {
  nh.spinOnce();
  
  timer.run();
}
