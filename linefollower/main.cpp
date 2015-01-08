/**
 * Group number: 28
 * Student 1:
 * Jente Hidskes, 4335732
 * Student 2:
 * Piet van Agtmaal, 4321278
 */

#include <ros/ros.h>

#include "linefollower_18.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "image_stream");
	LineFollower lf;
	ros::spin();
	return 0;
}
