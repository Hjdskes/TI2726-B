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
		 * It takes care of initialising the pinModes itself.
		 */
		Sensor(Engine *engine, int trigger, int echo);

	private:
		Engine *engine;
		int trigger;
		int echo;

		/**
		 * Generates a pulse. Implicitly sets the ECHO pin to HIGH.
		 */
		void generatePulse();
		/**
		 * Receives a pulse. The returned value is the duration in μs.
		 */
		long receivePulse();
		/**
		 * Controls the sensor. It calls generate- and receivePulse to detect
		 * obstacles, stopping the engine when needed.
		 */
		void poll();
};

#endif /* _SENSOR_H_ */
