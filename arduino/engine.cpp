/**
 * Group number: 28
 * Student 1:
 * Jente Hidskes, 4335732
 * Student 2:
 * Piet van Agtmaal, 4321278
 */

#include "engine.h"
#include "motor.h"

Engine::Engine(Motor *left, Motor *right) : left(left), right(right) {
}

static void turnLeft() {
	left.backward();
	right.forward();
}

static void turnRight() {
	left.forward();
	right.backward();
}

static void moveForward() {
	left.forward();
	right.forward();
}

static void moveBackward() {
	left.backward();
	right.backward();
}

void Engine::move(bool forward, int speed, int angular) {
	// FIXME: might be moved inside conditions to process angular.
	left.setSpeed(speed);
	right.setSpeed(speed);

	if (angular == 0) {
		if (forward) {
			moveForward();
		} else {
			moveBackward();
		}
	} else if (angular < 0) {
		if (forward) {
			turnLeft();
		} else {
			turnRight();
		}
	} else {
		if (forward) {
			turnRight();
		} else {
			turnLeft();
		}
	}
}

void Engine::stop() {
	left.stop();
	right.stop();
	//delay(25); // FIXME: Original code had this. (Why) Was this needed?
}

