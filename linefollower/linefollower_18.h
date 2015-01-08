/**
 * Group number: 28
 * Student 1:
 * Jente Hidskes, 4335732
 * Student 2:
 * Piet van Agtmaal, 4321278
 */

#ifndef _LINEFOLLOWER_H_
#define _LINEFOLLOWER_H_

#include <image_transport/image_transport.h>

class LineFollower {
	public:
		LineFollower();

	private:
		ros::NodeHandle nh;
		image_transport::ImageTransport it;
		image_transport::Subscriber image_sub;
		void imageCallback(const sensor_msgs::ImageConstPtr& color_img);
};

#endif /* _LINEFOLLOWER_H_ */
