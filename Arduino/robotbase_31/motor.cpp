/**
* [TI2726-B] Embedded Software
* Implementation of the robot.
* Regulates the motors of the robot, making sure the Twist messages
*	are interpreted and translated into the speed of the two motors.
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
#include <cmath>


Motor::Motor() {
	//
}

MotorSet::MotorSet(Motor m1, Motor m2) {
	//
}

void MotorSet::enable() {
	digitalWrite(LEFT_ENABLE, HIGH);
	digitalWrite(RIGHT_ENABLE, HIGH);
}

void MotorSet::disable() {
	digitalWrite(LEFT_ENABLE, LOW);
	digitalWrite(RIGHT_ENABLE, LOW);
}

//z neg = rechts
int MotorSet::update(double lineair_x, double angular_z) {
	if (lineair_x == 0.0 && angular_z == 0.0) return -1;
	
	if (lineair_x > 0.0 && angular_z == 0.0) {
		analogWrite(LEFT_BCK, 0);
		analogWrite(RIGHT_BCK, 0);
		analogWrite(LEFT_FWD, 25,5 * lineair_x);
		analogWrite(RIGHT_FWD, 25,5 * lineair_x);
	} else if (lineair_x < 0.0 && angular_z == 0.0) {
		analogWrite(LEFT_FWD, 0);
		analogWrite(RIGHT_FWD, 0);
		analogWrite(LEFT_BCK, 25,5 * std::abs (lineair_x));
		analogWrite(RIGHT_BCK, 25,5 * std::abs (lineair_x));
	} else if (lineair_x == 0.0 && angular_z > 0.0) {
		// TODO Tweak values
		analogWrite(LEFT_FWD, (25,5 * lineair_x) / angular_z);
		analogWrite(RIGHT_FWD, 25,5 * lineair_x);
		analogWrite(LEFT_BCK, 0);
		analogWrite(RIGHT_BCK, 0);
	} else if (lineair_x == 0.0 && angular_z < 0.0) {
		// TODO Tweak values
		analogWrite(LEFT_FWD, (25,5 * lineair_x) / std::abs (angular_z));
		analogWrite(RIGHT_FWD, 25,5 * lineair_x);
		analogWrite(LEFT_BCK, 0);
		analogWrite(RIGHT_BCK, 0);
	}
}

void MotorSet::stop() {
	analogWrite(LEFT_FWD, 0);
	analogWrite(RIGHT_FWD, 0);
	analogWrite(LEFT_BCK, 0);
	analogWrite(RIGHT_BCK, 0);
}
