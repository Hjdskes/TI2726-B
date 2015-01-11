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

	private:
		ros::NodeHandle nh;
		ros::Publisher twist_pub;
		image_transport::ImageTransport it;
		image_transport::Subscriber image_sub;

		/* Global used to keep track of the times the callback has been run. */
		uint8_t cb_count;
		/* Global used to keep track of the best directions while not averaging. */
		int calc_dirs;

		void imageCallback(const sensor_msgs::ImageConstPtr& color_img);
		bool toCVImg(const sensor_msgs::ImageConstPtr& src, cv::Mat& dest);
		void toBinary(cv::Mat& src, cv::Mat& dest);
		void toCanny(cv::Mat& src, cv::Mat& dest);
		void toHough(cv::Mat& src, std::vector<cv::Vec4i>& lines);	
		void drawDetectedLines(cv::Mat& img, std::vector<cv::Vec4i>& lines);
		int findDirection(std::vector<cv::Vec4i>& lines);
		void bestDirection(std::vector<cv::Vec4i>& lines);
		void generateTwist(int dir);
};

#endif /* _LINEFOLLOWER_H_ */
