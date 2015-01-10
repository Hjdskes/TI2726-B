/**
 * Group number: 28
 * Student 1:
 * Jente Hidskes, 4335732
 * Student 2:
 * Piet van Agtmaal, 4321278
 */

#include <math.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

#include "linefollower_18.h"

LineFollower::LineFollower() : it(nh) {
    /* Set compress image stream enabled. */
    image_transport::TransportHints hints("compressed", ros::TransportHints());

	/* Subscribe to input channel. */
    image_sub = it.subscribe("/camera/image", 1, &LineFollower::imageCallback, this, hints);
}

void LineFollower::imageCallback(const sensor_msgs::ImageConstPtr& img) {
	/* Declare all images needed */
	cv::Mat bgr_img, bin_img, edge_img;
	/* Create a vector to save detected lines */
	std::vector<cv::Vec4i> lines;
	
	/* Convert ROS image to an OpenCV BGR image */
	toCVImg(img, bgr_img);
	/* Transpose and flip image so it is oriented properly */
	cv::transpose(bgr_img, bgr_img);
	cv::flip(bgr_img, bgr_img, 1);

	/* Convert the BGR image to a binary image */
	toBinary(bgr_img, bin_img);

	/* Perform edge detection */
	toCanny(bin_img, edge_img);   

	/* Perform line detection */
	toHough(edge_img, lines); 

	/* Draw lines detected with Hough */
	drawDetectedLines(bgr_img, lines);

	/* Temporarily print found direction */
	ROS_INFO("%s", findDirection(lines));

	/* Resize image to suitable size and show the result. */
	cv::resize(bgr_img, bgr_img, cv::Size(540, 960));
	cv::imshow("Detected line image", bgr_img);
	cv::waitKey(3);
}

void LineFollower::toCVImg(const sensor_msgs::ImageConstPtr& src, cv::Mat& dest) {
	/* convert ROS image stream to OpenCV image, using color channels BGR (8-bit) */
	cv_bridge::CvImagePtr img_ptr;
	try {
		img_ptr = cv_bridge::toCvCopy(src, 
			sensor_msgs::image_encodings::BGR8);
		dest = img_ptr->image;
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		std::exit(1);
    }
}

void LineFollower::toBinary(cv::Mat& src, cv::Mat& dest) {
	/*BGR. adjust for different color detection. */
	/* FIXME: find perfect values */
	cv::Scalar min_vals(0, 100, 0);
	cv::Scalar max_vals(200, 200, 200);
	/* create binary image from the bgr image */
	cv::inRange(src, min_vals, max_vals, dest);
}

void LineFollower::toCanny(cv::Mat& src, cv::Mat& dest) {
	/* blur img. Needed for detecting edges */
	unsigned int kernel_size = 3;
	cv::blur(src, dest, cv::Size(kernel_size, kernel_size));

	/* detect edges */
	double low_thr = 40; /* lower threshold */
	cv::Canny(dest, dest, low_thr, 4*low_thr, kernel_size);
}

void LineFollower::toHough(cv::Mat& src, std::vector<cv::Vec4i>& lines) {
	/* Hough transform. Saves detected lines in vector 'lines'
	FIXME: find perfect values */
	double r = 1;
	double theta = CV_PI/180; /* max angle */
	unsigned int min_intersects = 50; /* mininum line intersections */
	unsigned int min_points = 30; /* minimum points */
	unsigned int max_gap = 10; /* maximum gap for lines to be connected */
	cv::HoughLinesP(src, lines, r, theta, min_intersects, 
		min_points, max_gap);
}

void LineFollower::drawDetectedLines(cv::Mat& img, std::vector<cv::Vec4i>& lines) {
	//FIXME: needed?
	cv::Scalar line_color(0,0,255);
	unsigned int line_width = 3;

	for (size_t i = 0; i < lines.size(); i++) {
		/* add each seperate line to the image */
		cv::Point point1(lines[i][0], lines[i][1]);
		cv::Point point2(lines[i][2], lines[i][3]);
		/* draw anti-aliased lines */
		cv::line(img, point1, point2, line_color, 
			line_width, CV_AA);
	}
}

const char* LineFollower::findDirection(std::vector<cv::Vec4i>& lines) {
	/* Keep track of the maximum horizontal and vertical length. */
	unsigned int max_h = 0;
	unsigned int max_v = 0;
	cv::Vec4i max_line;

	/* Loop through all lines and find the longest one. */
	for (int i = 0; i < lines.size(); i++) {
		cv::Vec4i line = lines.at(i);

		int h = abs(line[0] - line[2]);
		if (h > max_h) {
			max_h = h;
			max_line = line;
		}

		int v = abs(line[1] - line[3]);
		if (v > max_v) {
			max_v = v; 
		}
	}

	/* Calculate the angle between the longest line and the x-axis */
	double angle = atan2(max_line[3] - max_line[1], max_line[2] - max_line[0]);
	if (max_v > max_h) {
		return "forward";
	/* FIXME: tweak */
	} else if (angle > .2) {
		return "left";
	} else {
		return "right";	
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "image_stream");
	LineFollower lf;
	ros::spin();
	return 0;
}
