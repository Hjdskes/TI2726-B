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

/* TODO: stop when no messages have been received for some time.
   TODO: add a timer to the sketch to make the motor control interrupt based?
   TODO: stop when the proximity sensor senses an object nearby.
*/

/* Overload the standard settings that are used
 * by ros_lib to use the specified Serial port and baud rate. */
class ArduinoBluetooth : public ArduinoHardware { 
	public: 
		ArduinoBluetooth() : ArduinoHardware(&Serial1, 57600) {
		};
};

Engine *engine;
ros::NodeHandle_<ArduinoBluetooth> nh;

static void act(const geometry_msgs::Twist& twist) {
	engine->move(twist.linear.x > 0, twist.linear.x, twist.angular.z);
	/* FIXME: not sure if we want this... */
	//delay(2000);
	//engine.stop();
}

void setup() {
	const int LEFT_MOTOR[] = { 6, 7, 24 };
	const int RIGHT_MOTOR[] = { 2, 3, 25 };
	const int SENSOR[] = { 22, 23 };

	/* Setup engine. */
	Motor left(LEFT_MOTOR[0], LEFT_MOTOR[1], LEFT_MOTOR[2]);
	Motor right(RIGHT_MOTOR[0], RIGHT_MOTOR[1], RIGHT_MOTOR[2]);
	engine = new Engine(&left, &right);

	/* Setup proximity sensor. */
	Sensor sensor(engine, SENSOR[1], SENSOR[0]);
	// TODO: add a timer so it does its thing!

	/* Setup ROS. */
	nh.initNode();
	ros::Subscriber<geometry_msgs::Twist> sub = nh.subscribe("cmd_vel", 5, act);
}

void loop() {
	nh.spinOnce();
	delay(100);
}
