/**
 * Group number: 28
 * Student 1:
 * Jente Hidskes, 4335732
 * Student 2:
 * Piet van Agtmaal, 4321278
 */

#ifndef _ENGINE_H_
#define _ENGINE_H_

/* Forward declaration. */
class Motor;

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
		 * Makes the robot move in the specified direction, using the given speed and angular value.
		 * Setting forward to true means forward movement, whereas false means backward.
		 */
		void move(bool forward, int speed, int angular);
		/**
		 * Stops the robot.
		 */
		void stop();

	private:
		Motor *left;
		Motor *right;
		void moveForward(int speed);
		void moveBackward(int speed);
		void turnLeft(int speed);
		void turnRight(int speed);
};
#endif /* _ENGINE_H_ */
