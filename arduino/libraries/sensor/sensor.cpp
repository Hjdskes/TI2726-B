/**
 * Group number: 28
 * Student 1:
 * Jente Hidskes, 4335732
 * Student 2:
 * Piet van Agtmaal, 4321278
 */

#include "sensor.h"
#include "engine.h"
#include "Arduino.h"

/**
 * From the manual:
 *
 * A short ultrasonic pulse is transmitted at the time 0, reflected by an
 * object. The sensor receives this signal and converts it to an electric
 * signal. The next pulse can be transmitted when the echo is faded away.
 * This time period is c alled cycle period. The recommend cycle period
 * should be no less than 50ms. If a 10μs width trigger pulse is sent to 
 * the signal pin, the Ultrasonic module will output eight 40kHz ultrasonic
 * signal and detect the echo back. The measured distance is proportional
 * to the echo pulse width and can be calculated by the formula above. If
 * no obstacle is detected, the output pin will give a 38ms high level signal.
 */

Sensor::Sensor(Engine *engine, int trigger, int echo) : engine(engine), trigger(trigger), echo(echo) {
	pinMode(trigger, OUTPUT);
	pinMode(echo, INPUT);
}

void Sensor::generatePulse() {
	/* The trigger pulse needs to be at least 10 μs long.
	 * See the HC-SR04 datasheet. */
	static const int MIN_PULSE_LENGTH = 10;

	/* Generate a pulse. */
	digitalWrite(this->trigger, HIGH);
	delayμs(MIN_PULSE_LENGTH);
	digitalWrite(this->trigger, LOW);
}

long Sensor::receivePulse() {
	long duration;

	duration = pulseIn(this->echo, HIGH);
	// FIXME: not sure if needed.
	//digitalWrite(this->echo, LOW);
}

void Sensor::poll() {
	/* The maximum distance at which the robot will stop.
	 * FIXME: not sure if this is the proper distance. */
	static const int MAX_DISTANCE = 12;
	/* Divide by this receive distance in cm, see the HC-SR04 datasheet. */ 
	static const float PULSE_DIVIDE = 58.0;
	long duration, distance;

	generatePulse();
	duration = receivePulse();

	distance = duration / PULSE_DIVIDE;
	if (distance < MAX_DISTANCE) {
		engine.stop();
	} else {
		engine.start();
	}
}
