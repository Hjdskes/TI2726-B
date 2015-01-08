/**
 * Group number: 28
 * Student 1:
 * Jente Hidskes, 4335732
 * Student 2:
 * Piet van Agtmaal, 4321278
 */

#include <ros.h>
#include <geometry_msgs/Twist.h>

#include "engine.h"
#include "motor.h"

/* Overload the standard settings that are used
 * by ros_lib to use the specified Serial port and baud rate. */
class NewHardware : public ArduinoHardware { 
	public: 
		NewHardware() : ArduinoHardware(&Serial1, 57600) {
		};
};

/* Forward declare callback. */
void parseTwistCb(const geometry_msgs::Twist& twist);

/* Setup engines. FIXME: move to setup(). */
int LEFT_MOTOR[] = { 6, 7, 24 };
int RIGHT_MOTOR[] = { 2, 3, 25 };

Motor left(LEFT_MOTOR[0], LEFT_MOTOR[1], LEFT_MOTOR[2]);
Motor right(RIGHT_MOTOR[0], RIGHT_MOTOR[1], RIGHT_MOTOR[2]);
Engine engine(&left, &right);

/* Setup ROS. FIXME: move to setup(). */
ros::NodeHandle<NewHardware> nh;
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &parseTwistCb);

/* FIXME: decide best practice. */
bool forward;
int speed;
int angular;

void setup() {
	nh.initNode();
	nh.subscribe(sub);
}

void loop() {
	engine.move(forward, speed, angular);
	delay(2000);
	engine.stop();

	delay(2000);
	nh.spinOnce();
}

void parseTwistCb(const geometry_msgs::Twist& twist) {
	forward = true;
	speed = 50;
	angular = 0;
}

