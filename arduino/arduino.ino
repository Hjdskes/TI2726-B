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
#include "sensor.h"

/* TODO: test Twist message parsing.
   TODO: add a timer to the sketch to make the motor control interrupt based?
   TODO: test stopping when the proximity sensor senses an object nearby.
   FIXME: when to restart engine.
*/

/* Overload the standard settings that are used
 * by ros_lib to use the specified Serial port and baud rate. */
class ArduinoBluetooth : public ArduinoHardware { 
	public: 
		ArduinoBluetooth() : ArduinoHardware(&Serial1, 57600) {
		};
};

Engine *engine = NULL;
Sensor *sensor = NULL;
ros::NodeHandle_<ArduinoBluetooth> nh;
int isStoppedBySensor;

static void act(const geometry_msgs::Twist& twist) {
	/* Reset counter upon processing ROS message. */
	//TCNT5 = 0;
	//if (isStoppedBySensor == 0 && engine->isStopped()) {
	//	engine->start();
	//}
	engine->move(twist.linear.x > 0, twist.linear.x, twist.angular.z);
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &act);

void setup() {
	const int LEFT_MOTOR[] = { 6, 7, 24 };
	const int RIGHT_MOTOR[] = { 2, 3, 25 };
	const int SENSOR[] = { 22, 23 };

	/* Setup Timer5 to reliably fire a interrupt every second.
	 *
	 * To do so, our value in the compare match register must be:
	 * (16000000 / (256 * 1)) - 1 = 62499, where 16000000 is the Arduino
	 * that runs at 16MHz, 256 is our prescaler value (smaller is more accurate)
	 * and 1Hz means once every second.
	 *
	 * See libraries/sensor/sensor.cpp for a list on which timers are available
	 * and comments with all instructions. */
	/*noInterrupts();
	TCCR5A = 0;
	TCCR5B = 0;
	TCNT5 = 0;
	OCR5A = 62499;
	TCCR5B |= (1 << WGM12);
	TCCR5B |= (1 << CS52);
	TIMSK5 |= (1 << OCIE5A);
	interrupts();*/

	/* Setup engine. */
	Motor *left = new Motor(LEFT_MOTOR[0], LEFT_MOTOR[1], LEFT_MOTOR[2]);
	Motor *right = new Motor(RIGHT_MOTOR[0], RIGHT_MOTOR[1], RIGHT_MOTOR[2]);
	engine = new Engine(left, right);

	/* Setup proximity sensor. */
	sensor = new Sensor(engine, SENSOR[1], SENSOR[0]);
	isStoppedBySensor = 0;

	/* Setup ROS. */
	nh.initNode();
        nh.subscribe(sub);
}

ISR(TIMER5_COMPA_vect) {
	/* If this interrupt launches it means that no ROS message has been received
	 * in the passed second (because we reset the counter in the ROS callback),
	 * so stop the engine. */
	//if (engine) {
	//	engine->stop();
	//}
}

void loop() {
	isStoppedBySensor = sensor->poll();
        if (isStoppedBySensor == 0) {
                nh.spinOnce();
	}
        delay(100);
}
