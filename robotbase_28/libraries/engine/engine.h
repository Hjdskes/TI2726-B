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
 * This class represents the actual engine in totality. It makes use of two
 * Motor instances; one for the left motor and one for the right motor.
 */
class Engine {
	public:
		/**
		 * Constructs a new Engine instance, to control both Motors.
		 */
		Engine(Motor *left, Motor *right);
		/**
		 * Makes the robot move in the specified direction, using the
		 * given speed. Speed is taken as a percentage and thus should
		 * be between 0 and 100 inclusive.
		 * Negative angles mean turning left, whereas positive angles
		 * mean turning right. An angle of zero means going straight.
		 * The tracks turn forwards iff forward is true.
		 */
		void move(const bool forward, const int speed, const int angular);
		/**
		 * Starts the engine.
		 */
		void start();
		/**
		 * Stops the engine.
		 */
		void stop();
		/**
		 * Returns true iff this engine is currently stopped.
		 */
		bool isStopped();

	private:
		Motor *left;
		Motor *right;

		void moveForward(const int speed);
		void moveBackward(const int speed);
		void turnLeft(const int speed);
		void turnRight(const int speed);
};

#endif /* _ENGINE_H_ */
