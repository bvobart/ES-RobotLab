
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
#include <ros.h>
#include <ArduinoHardware.h>

#include <ArduinoRobotMotorBoard.h>
#include <LineFollow.h>
#include <EasyTransfer2.h>
#include <Multiplexer.h>

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

char input = 'S';        // Input from the sender
int velocity = 0;        // Velocity of the robot

// Needed to set up a bluetooth connection to the main station
class NewHardware : public ArduinoHardware {
    public: 
        NewHardware() : ArduinoHardware(&Serial1, 57600) {};
};

//Motor m1(7, 24, 6); //?
//Motor m2(3, 25, 2); //?
//MotorSet motor(&m1, &m2); //?

//void cmd_vel_cb(const geometry_msgs::Twist& cmd_vel_msg) {
//    motor.update(cmd_vel_msg.linear.x, cmd_vel_msg.angular.z);
//}

// Adds a new subscriber
ros::NodeHandle_<NewHardWare> nh;
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &cmd_vel_cb);

void setup() { // No clue if any of this is correct
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
  
  Serial1.println("Initializing Arduino");
  Serial1.println("     Wuq©");
  
  // Turn motors off?
  // Do stuffs
}

void loop() {
  nh.spinOnce();
  
  char det = check();
  
  switch(det) {
    case 'W':          // W pressed, so go forward
    // 
    break;
    
    case 'S':          // S pressed, so go backward
    // 
    break;
    
    case 'A':          // A pressed, so go left
    // 
    break;
    
    case 'D':          // D pressed, so go right
    // 
    break;
    
    case 'F':          // F pressed, so halt
    // 
    break;
    
  }
    
}

/**
* Method that checks for input from the sender
* and puts it in a variable, to be used in the loop.
* Also configures the speed of the robot.
*/
int check() {
  
  // Return -1 when the serial is not available
  if (Serial1.available() <= 0) {
    return -1;
  }
  
  input = Serial1.read();
  
  // Set the velocity of the robot
  if (input == 'Z' && velocity >= 10) {
    velocity -= 10;
    Serial1.write("Set speed to: " + velocity);
  } else if (input == 'X' && velocity <= 90) {
    velocity += 10;
    Serial1.write("Set speed to: " + velocity);
  }
  
  return input;
}
