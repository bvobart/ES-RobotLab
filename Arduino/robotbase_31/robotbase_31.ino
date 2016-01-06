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
*wsws
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

#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

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

void cmd_vel_cb(const geometry_msgs::Twist& msg) {
    updateBot(msg.linear.x, msg.angular.z);
}

// Adds a new subscriber
ros::NodeHandle_<NewHardware> nh;
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &cmd_vel_cb);

void startup() {
  digitalWrite(LEFT_FWD, LOW);
  digitalWrite(LEFT_BCK, LOW);
  digitalWrite(RIGHT_FWD, LOW);
  digitalWrite(RIGHT_BCK, LOW);
  
  enableBot();
}

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

int updateBot(double lineair_x, double angular_z) {
  if (lineair_x == 0.0 && angular_z == 0.0) return -1;
	
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
	analogWrite(LEFT_FWD, (25.5 * lineair_x) / angular_z);
	analogWrite(RIGHT_FWD, 25.5 * lineair_x);
	analogWrite(LEFT_BCK, 0);
	analogWrite(RIGHT_BCK, 0);
    } else if (lineair_x == 0.0 && angular_z < 0.0) {
	// TODO Tweak values
	analogWrite(LEFT_FWD, (25.5 * lineair_x) / abs(angular_z));
	analogWrite(RIGHT_FWD, 25.5 * lineair_x);
	analogWrite(LEFT_BCK, 0);
	analogWrite(RIGHT_BCK, 0);
    }
    return 0;
}

void setup() { // No clue if any of this is correct
  Serial1.println("Initializing Arduino");
  Serial1.println("     Wuq©");
  
  Serial.begin(57600);                 // Set the speed of the USB connection (in baud)
  Serial1.begin(57600);                // Set the speed of the bluetooth connection (in baud)
  
  nh.initNode();
  nh.subscribe(sub);
  
  pinMode(LEFT_FWD, OUTPUT);
  pinMode(LEFT_BCK, OUTPUT);
  pinMode(LEFT_ENABLE, INPUT);
  pinMode(RIGHT_FWD, OUTPUT);
  pinMode(RIGHT_BCK, OUTPUT);
  pinMode(RIGHT_ENABLE, INPUT);
  
  pinMode(BLUETOOTH_TX, INPUT);
  pinMode(BLUETOOTH_RX, INPUT);
  pinMode(US_ECHO, INPUT);
  pinMode(US_TRIGGER, INPUT);
  pinMode(LED, OUTPUT);
  
  startup();
}

void loop() {
  nh.spinOnce();
    
}
