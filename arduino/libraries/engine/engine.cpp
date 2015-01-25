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

void Engine::moveForward(const int speed) {
	left->forward(speed);
	right->forward(speed);
}

void Engine::moveBackward(const int speed) {
	left->backward(speed);
	right->backward(speed);
}

void Engine::turnLeft(const int speed) {
	left->forward(speed / 3);
	right->forward(speed);
}

void Engine::turnRight(const int speed) {
	left->forward(speed);
	right->forward(speed / 3);
}

void Engine::move(const bool forward, const int speed, const int angular) {
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

bool Engine::isStopped() {
	return (left->isStopped() && right->isStopped());
}
