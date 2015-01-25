/**
 * Group number: 28
 * Student 1:
 * Jente Hidskes, 4335732
 * Student 2:
 * Piet van Agtmaal, 4321278
 */

#ifndef _MOTOR_H_
#define _MOTOR_H_

/**
 * This class represents one half of the LCHB-100 component.
 *
 * Speeds are taken as percentages and thus should be between 0 and 100
 * inclusive.
 */
class Motor {
	public:
		/**
		 * Constructs a new motor instance that will use the supplied pin layout.
		 * It takes care of initialising the pinModes itself.
		 */
		Motor(const int fwd, const int rev, const int en);
		/**
		 * Makes this motor go forward, using the given speed.
		 */
		void forward(const int speed);
		/**
		 * Makes this motor go backward, using the given speed.
		 */
		void backward(const int speed);
		/**
		 * Starts this motor.
		 */
		void start();
		/**
		 * Stops this motor.
		 */
		void stop();
		/**
		 * Returns true iff this motor is currently stopped.
		 */
		bool isStopped();

	private:
		const int fwd;
		const int rev;
		const int en;
};

#endif /* _MOTOR_H_ */
