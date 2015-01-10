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
#include <opencv2/imgproc/imgproc.hpp>

class LineFollower {
	public:
		LineFollower();
		const char* determineDirection(cv::Mat& img);

	private:
		ros::NodeHandle nh;
		image_transport::ImageTransport it;
		image_transport::Subscriber image_sub;
		void imageCallback(const sensor_msgs::ImageConstPtr& color_img);
		void findBestRowCol(cv::Mat img, int& best_row, int& best_col, int& row_red, int& col_red);
		void updateRedCount(uint8_t red, unsigned int& count);
		cv::Mat translateImage(cv::Mat& img, int x, int y);
};

#endif /* _LINEFOLLOWER_H_ */
