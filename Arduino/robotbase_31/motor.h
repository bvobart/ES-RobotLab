/**
* [TI2726-B] Embedded Software
* Header file for implementation of the robot
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
#include <Arduino>
#include <inttypes>

#define LEFT_FWD = 6;        // LCHB-100 H Bridge 1FWD
#define LEFT_BCK = 7;        // LCHB-100 H Bridge 1REV
#define LEFT_ENABLE = 24;    // LCHB-100 H Bridge 1EN
#define RIGHT_FWD = 2;       // LCHB-100 H Bridge 2FWD
#define RIGHT_BCK = 3;       // LCHB-100 H Bridge 2REV
#define RIGHT_ENABLE = 25;   // LCHB-100 H Bridge 2EN
#define BLUETOOTH_TX = 18;   // HC-05 Bluetooth dongle TX
#define BLUETOOTH_RX = 19;   // HC-05 Bluetooth dongle RX
#define US_ECHO = 22;        // HC-SR04 ultrasonic sensor ECHO
#define US_TRIGGER = 23;     // HC-SR04 ultrasonic sensor TRIGGER
#define LED = 13;            // Yellow LED light

class Motor {
	Motor();
}

class MotorSet {
	MotorSet(Motor m1, Motor m2);
	void enable();
	void disable();
	int update(double lineair_x, double angular_z);
	void stop();
}
