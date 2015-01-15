/**
 * Group number: 28
 * Student 1:
 * Jente Hidskes, 4335732
 * Student 2:
 * Piet van Agtmaal, 4321278
 */

#ifndef _SENSOR_H_
#define _SENSOR_H_

/* Forward declaration. */
class Engine;

/**
 * This class respresents the proximity sensor. It takes the engine pointer as
 * an instance variable, so that it can poll on itself based on the timers and
 * make the robot stop whenever necessary.
 */
class Sensor {
	public:
		/**
		 * Constructs a new Sensor instance, to poll for objects and
		 * stop/start the engine appropriately.
		 * It takes care of initialising the pinModes itself and configures
		 * Timer1 for its own use.
		 */
		Sensor(const Engine *engine, const int trigger, const int echo);
		/**
		 * Generates a pulse.
		 */
		void generatePulse();
		/**
		 * Polls the sensor. It processes the data and detects obstacles,
		 * stopping the engine when needed.
		 * Returns true iff an object has been detected.
		 */
		bool poll();

	private:
		const Engine *engine;
		const int trigger;
		const int echo;

		/**
		 * Receives a pulse. The returned value is the pulse duration in μs.
		 */
		unsigned long receivePulse();
};

/* Accessor, so that the ISR in sensor.cpp can access the sensor object declared
 * in the sketch. Because this is extern (instead of controlling it from within
 * the Sensor class), the user of the class is in control and as such, it is not
 * up to us anymore to enforce only one instance at a time. */
extern Sensor *sensor;

#endif /* _SENSOR_H_ */
