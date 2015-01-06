/**
 * Group number: 28
 * Student 1:
 * Jente Hidskes, 4335732
 * Student 2:
 * Piet van Agtmaal, 4321278
 */

Engine::Engine(Motor *left, Motor *right) : left(left), right(right) {
}

void Engine::forward(int speed, float angle) {
	// FIXME: might be moved inside conditions.
	left.setSpeed(speed);
	right.setSpeed(speed);

	if (angle != 0.0f) {
		if (angle < 0.0f) {
			// TODO: turn left.
		} else {
			// TODO: turn right.
		}
	} else {
		left.forward();
		right.forward();
	}
}

void Engine::backward(int speed, float angle) {
	// FIXME: possible code-duplication with Engine::forward.
	// FIXME: might be moved inside conditions.
	left.setSpeed(speed);
	right.setSpeed(speed);

	if (angle != 0.0f) {
		if (angle < 0.0f) {
			// TODO: turn left.
		} else {
			// TODO: turn right.
		}
	} else {
		left.backward();
		right.backward();
	}
}

void Engine::stop() {
	left.stop();
	right.stop();
	//delay(25); //Original code had this. (Why) Was this needed?
}

