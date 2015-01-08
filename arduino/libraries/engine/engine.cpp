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

void Engine::turnLeft(int speed) {
	left->backward(speed);
	right->forward(speed);
}

void Engine::turnRight(int speed) {
	left->forward(speed);
	right->backward(speed);
}

void Engine::moveForward(int speed) {
	left->forward(speed);
	right->forward(speed);
}

void Engine::moveBackward(int speed) {
	left->backward(speed);
	right->backward(speed);
}

void Engine::move(bool forward, int speed, int angular) {
	// FIXME: perhaps use angular to control speed when taking corners.
	if (angular == 0) {
		if (forward) {
			moveForward(speed);
		} else {
			moveBackward(speed);
		}
	} else if (angular < 0) {
		if (forward) {
			turnLeft(speed);
		} else {
			turnRight(speed);
		}
	} else {
		if (forward) {
			turnRight(speed);
		} else {
			turnLeft(speed);
		}
	}
}

void Engine::start() {
	left->start();
	right->start();
}

void Engine::stop() {
	left->stop();
	right->stop();
}

