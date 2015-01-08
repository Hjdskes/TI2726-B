/**
 * Group number: 28
 * Student 1:
 * Jente Hidskes, 4335732
 * Student 2:
 * Piet van Agtmaal, 4321278
 */

#include "engine.h"
#include "motor.h"

/* Setup engines. */
int LEFT_MOTOR[] = { 6, 7, 24 };
int RIGHT_MOTOR[] = { 2, 3, 25 };

Motor left(LEFT_MOTOR[0], LEFT_MOTOR[1], LEFT_MOTOR[2]);
Motor right(RIGHT_MOTOR[0], RIGHT_MOTOR[1], RIGHT_MOTOR[2]);
Engine engine(&left, &right);

void setup() {
	Serial.begin(9600);
}

void loop() {
	engine.move(true, 50, 0);
	delay(1000);
	engine.stop();

	engine.move(false, 50, 0);
	delay(1000);
	engine.stop();

	engine.move(true, 50, -2);
	delay(1000);
	engine.stop();

	engine.move(true, 50, 2);
	delay(1000);
	engine.stop();

	delay(2000);
}

