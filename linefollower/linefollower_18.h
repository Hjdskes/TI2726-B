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
		const char* findDirection(std::vector<cv::Vec4i>& lines);

	private:
		ros::NodeHandle nh;
		image_transport::ImageTransport it;
		image_transport::Subscriber image_sub;
		void imageCallback(const sensor_msgs::ImageConstPtr& color_img);
		void toCVImg(const sensor_msgs::ImageConstPtr& src, cv::Mat& dest);
		void toBinary(cv::Mat& src, cv::Mat& dest);
		void toCanny(cv::Mat& src, cv::Mat& dest);
		void toHough(cv::Mat& src, std::vector<cv::Vec4i>& lines);	
		void drawDetectedLines(cv::Mat& img, std::vector<cv::Vec4i>& lines);
};

#endif /* _LINEFOLLOWER_H_ */
