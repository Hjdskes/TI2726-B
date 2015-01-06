/**
 * Group number: 28
 * Student 1:
 * Jente Hidskes, 4335732
 * Student 2:
 * Piet van Agtmaal, 4321278
 */

#ifndef _ENGINE_H_
#define _ENGINE_H_

/**
 * This class represent the actual engine in totality. It makes use of two Motor
 * instances; one for the left motor and one for the right motor.
 *
 * Speeds are taken as percentages and thus should be between 0 and 100
 * inclusive.
 *
 * Negative angles mean turning left. Positive angles mean turning right. An
 * angle of zero means going straight.
 */
class Engine {
	public:
		/**
		 * Constructs a new Engine instance, to control both Motors.
		 */
		Engine(Motor *left, Motor *right);
		/**
		 * Makes the robot move forward, using the given speed and angle.
		 */
		void forward(int speed, float angle);
		/**
		 * Makes the robot move backward, using the given speed and angle.
		 */
		void backward(int speed, float angle);
		/**
		 * Stops the robot.
		 */
		void stop();

	private:
		Motor *left;
		Motor *right;
};
#endif /* _ENGINE_H_ */
