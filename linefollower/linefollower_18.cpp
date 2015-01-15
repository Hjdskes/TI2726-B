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

/* Constants used for binary conversion.
 * BGR. Adjust for different color detection. */
static const cv::Scalar MIN_VALS(0, 100, 0);
static const cv::Scalar MAX_VALS(200, 200, 200);

/* Constants used for edge detection. */
static const uint8_t KERNEL_SIZE = 3;
static const uint8_t LOW_THR = 40;

/* Direction constants. */
static const uint8_t FORWARD = 0;
static const uint8_t RIGHT = 1;
static const int8_t LEFT = -1;

/* Speed constants. */
static const uint8_t FW_SPEED = 70;
static const uint8_t TURN_SPEED = 30;

/* Dimension constants */
static const int WIDTH = 1920;
static const int HEIGHT = 1080;

LineFollower::LineFollower() : it(nh) {
    /* Set compress image stream enabled. */
    image_transport::TransportHints hints("compressed", ros::TransportHints());

	/* Subscribe to input channel. */
    image_sub = it.subscribe("/camera/image", 1, &LineFollower::imageCallback, this, hints);

	/* Publish to /cmd_vel. */
	twist_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

	/* Init globals needed for averaging calculated direction. */
	cb_count = 0;
	calc_dirs = 0;
}

void LineFollower::imageCallback(const sensor_msgs::ImageConstPtr& img) {
	/* Declare all images needed. */
	cv::Mat bgr_img(WIDTH, HEIGHT, CV_8UC3);
	cv::Mat bin_img(WIDTH, HEIGHT, CV_8UC1);
	cv::Mat edge_img(WIDTH, HEIGHT, CV_8UC1);
	/* Create a vector to save detected lines. */
	std::vector<cv::Vec4i> lines;
	
	/* Convert ROS image to an OpenCV BGR image. */
	if (!toCVImg(img, bgr_img)) return;

	/* Crop the image to only look at close lines. */
	bgr_img = cv::Mat(bgr_img, cv::Rect(WIDTH / 2, HEIGHT / 18, WIDTH / 2, WIDTH / 2));
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

	/* Code below is for debugging purposes only.*/

	/* Draw lines detected with Hough.*/
	drawDetectedLines(bgr_img, lines);

	/* Resize image to suitable size and show the result. */
	cv::resize(bgr_img, bgr_img, cv::Size(HEIGHT / 2, HEIGHT / 2));
	cv::imshow("Detected line image", bgr_img);
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
	/* Create a binary image from the BGR image. */
	cv::inRange(src, MIN_VALS, MAX_VALS, dest);
}

void LineFollower::toCanny(cv::Mat& src, cv::Mat& dest) {
	/* Blur img. Needed for smoothly detecting edges. */
	cv::GaussianBlur(src, dest, cv::Size(5, 5), 1.4);

	/* Detect edges. */
	cv::Canny(dest, dest, LOW_THR, 3*LOW_THR, KERNEL_SIZE);
}

void LineFollower::toHough(cv::Mat& src, std::vector<cv::Vec4i>& lines) {
	/* Hough transform. Saves detected lines in vector 'lines'.
	 * http://docs.opencv.org/modules/imgproc/doc/feature_detection.html?highlight=houghlinesp#houghlinesp 
	 */ 
	cv::HoughLinesP(src, lines, 1, CV_PI/180, 100, 50, 30);
}

void LineFollower::drawDetectedLines(cv::Mat& img, std::vector<cv::Vec4i>& lines) {
	cv::Scalar line_color(0,0,255);
	unsigned int line_width = 3;

	for (size_t i = 0; i < lines.size(); i++) {
		/* Add each seperate line to the image. */
		cv::Point point1(lines[i][0], lines[i][1]);
		cv::Point point2(lines[i][2], lines[i][3]);
		/* Draw anti-aliased lines. */
		cv::line(img, point1, point2, line_color, line_width, CV_AA);
	}
}

int LineFollower::findDirection(std::vector<cv::Vec4i>& lines) {
	max_h = 0;
	max_v = 0;

	/* Loop through all lines and find the longest one. */
	for (uint8_t i = 0; i < lines.size(); i++) {
		cv::Vec4i line = lines.at(i);

		uint16_t h = abs(line[2] - line[0]);
		if (h > max_h) {
			max_h = h;
			max_line = line;
		}

		uint16_t v = abs(line[3] - line[1]);
		if (v > max_v) {
			max_v = v; 
		}
	}

	/* Calculate the angle between the longest line and the x-axis. */
	float angle = atan2(max_line[3] - max_line[1], max_line[2] - max_line[0]);
	if (max_v > max_h) {
		return FORWARD;
	} else if (angle > .1) {
		return LEFT;
	} else {
		return RIGHT;	
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
		generateTwist(RIGHT);	
	} else if (calc_dirs < -2) {
		generateTwist(LEFT);
	} else {
		generateTwist(FORWARD);
	}

	/* Only activate for debugging.
	ROS_INFO("> 2: right, < -2: left, else forward: %d", calc_dirs); */
	
	/* Reset the globals. */
	calc_dirs = 0;
	cb_count = 0;
}

void LineFollower::generateTwist(int dir) {
	/* Create and publish a Twist message indicating the speed and direction. */
	geometry_msgs::Twist twist;
    twist.angular.z = dir;
    twist.linear.x = dir == FORWARD ? FW_SPEED : TURN_SPEED;
	twist_pub.publish(twist);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "image_stream");
	LineFollower lf;
	/* Exit on any key. */
	while (cv::waitKey(3) < 0) {
		ros::spinOnce();
	}
	return 0;
}
