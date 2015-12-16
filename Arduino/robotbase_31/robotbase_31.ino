/**
* [TI2726-B] Embedded Software
* Code for line following robot.
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
*/
#include <ros.h>
#include <std_msgs/Int32.h>

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

void setup() { // No clue if any of this is correct
  Serial1.begin(57600);
  
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
  char det = check();
  
  switch(det) {
    case 'S':
    // Stop
    break;
    
  }
    
}

/**
* Method that checks for input from the sender
* and puts it in a variable, to be used in the loop.
*/
int check() {
  
  // Return -1 when the serial is not available
  if (Serial1.available() <= 0) {
    return -1;
  }
  
  input = Serial1.read();
  return input;
}
