/**
 * Group number: 28
 * Student 1:
 * Jente Hidskes, 4335732
 * Student 2:
 * Piet van Agtmaal, 4321278
 */

#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

#include "linefollower_18.h"

LineFollower::LineFollower() : it(nh) {
    /* Set compress image stream enabled. */
    image_transport::TransportHints hints("compressed", ros::TransportHints());

	/* Subscribe to input channel. */
    image_sub = it.subscribe("/camera/image", 1, &LineFollower::imageCallback, this, hints);

	twist_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

	/* Init globals needed for averaging calculated direction. */
	cb_count = 0;
	calc_dirs = 0;
}

void LineFollower::imageCallback(const sensor_msgs::ImageConstPtr& img) {
	/* Declare all images needed. */
	cv::Mat bgr_img, bin_img, edge_img;
	/* Create a vector to save detected lines. */
	std::vector<cv::Vec4i> lines;
	
	/* Convert ROS image to an OpenCV BGR image. */
	if (!toCVImg(img, bgr_img)) return;

	/* Crop the image to only look at close lines. */
	bgr_img = cv::Mat(bgr_img, cv::Rect(960, 60, 960, 960));
	/* Transpose and flip the image so it is oriented properly. */
	cv::transpose(bgr_img, bgr_img);
	cv::flip(bgr_img, bgr_img, 1);
 
	/* Convert the BGR image to a binary image. */
	toBinary(bgr_img, bin_img);

	/* Perform edge detection. */
	toCanny(bin_img, edge_img);   

	/* Perform line detection. */
	toHough(edge_img, lines); 

	/* Extract the best direction from the lines. */
	bestDirection(lines);

	/* Code below is for debugging purposes only.
	/* Draw lines detected with Hough.
	drawDetectedLines(bgr_img, lines);
	/* Resize image to suitable size and show the result.
	cv::resize(bgr_img, bgr_img, cv::Size(540, 540));
	cv::imshow("Detected line image", bgr_img);
	cv::waitKey(3); */
}

bool LineFollower::toCVImg(const sensor_msgs::ImageConstPtr& src, cv::Mat& dest) {
	/* Convert ROS image stream to OpenCV image, using color channels BGR (8-bit). */
	cv_bridge::CvImagePtr img_ptr;
	try {
		img_ptr = cv_bridge::toCvCopy(src, 
			sensor_msgs::image_encodings::BGR8);
		dest = img_ptr->image;
		return true;
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return false;
    }
}

void LineFollower::toBinary(cv::Mat& src, cv::Mat& dest) {
	/*BGR. Adjust for different color detection. */
	/* FIXME: find perfect values */
	cv::Scalar min_vals(0, 100, 0);
	cv::Scalar max_vals(200, 200, 200);
	/* Create a binary image from the BGR image. */
	cv::inRange(src, min_vals, max_vals, dest);
}

void LineFollower::toCanny(cv::Mat& src, cv::Mat& dest) {
	/* Blur img. Needed for detecting edges. */
	unsigned int kernel_size = 3;
	cv::blur(src, dest, cv::Size(kernel_size, kernel_size));

	/* Detect edges. */
	double low_thr = 40; /* lower threshold */
	cv::Canny(dest, dest, low_thr, 4*low_thr, kernel_size);
}

void LineFollower::toHough(cv::Mat& src, std::vector<cv::Vec4i>& lines) {
	/* Hough transform. Saves detected lines in vector 'lines'.
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
	cv::Scalar line_color(0,0,255);
	unsigned int line_width = 3;

	for (size_t i = 0; i < lines.size(); i++) {
		/* Add each seperate line to the image. */
		cv::Point point1(lines[i][0], lines[i][1]);
		cv::Point point2(lines[i][2], lines[i][3]);
		/* Draw anti-aliased lines. */
		cv::line(img, point1, point2, line_color, 
			line_width, CV_AA);
	}
}

int LineFollower::findDirection(std::vector<cv::Vec4i>& lines) {
	/* Keep track of the maximum horizontal and vertical length. */
	unsigned int max_h = 0;
	unsigned int max_v = 0;
	cv::Vec4i max_line;

	/* Loop through all lines and find the longest one. */
	for (int i = 0; i < lines.size(); i++) {
		cv::Vec4i line = lines.at(i);

		int h = abs(line[2] - line[0]);
		if (h > max_h) {
			max_h = h;
			max_line = line;
		}

		int v = abs(line[3] - line[1]);
		if (v > max_v) {
			max_v = v; 
		}
	}

	/* Calculate the angle between the longest line and the x-axis. */
	double angle = atan2(max_line[3] - max_line[1], max_line[2] - max_line[0]);
	if (max_v > max_h) {
		return 0;
	/* FIXME: tweak */
	} else if (angle > .1) {
		return -1;
	} else {
		return 1;	
	}
}

void LineFollower::bestDirection(std::vector<cv::Vec4i>& lines) {
	/* Add the calculated direction to the direction total. */
	calc_dirs += findDirection(lines);
	/* The callback has been called another time, so increment the counter. */
	cb_count++;
	/* Only return a direction if the callback has been run 5 times. */
	if (cb_count < 5) {
		return;
	}

	if (calc_dirs > 2) {
		generateTwist(1);	
	} else if (calc_dirs < -2) {
		generateTwist(-1);
	} else {
		generateTwist(0);
	}

	ROS_INFO("> 2: right, < -2: left, else forward: %d", calc_dirs);
	
	/* Reset the globals. */
	calc_dirs = 0;
	cb_count = 0;
}

void LineFollower::generateTwist(int dir) {
	/* Create and publish a Twist message indicating the speed and direction. */
	geometry_msgs::Twist twist;
    twist.angular.z = dir;
    twist.linear.x = dir == 0 ? 100 : 40;
	twist_pub.publish(twist);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "image_stream");
	LineFollower lf;
	ros::spin();
	return 0;
}
