/**
 * Group number: 28
 * Student 1:
 * Jente Hidskes, 4335732
 * Student 2:
 * Piet van Agtmaal, 4321278
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

#include "linefollower_18.h"

/* Minimum delay between 2 successive Twist commands. */
static const uint8_t MIN_DELAY = 250;

/* Maximum angle allowed before adjusting speed for taking corners. */
static const uint8_t FW_CORNER_ALLOWED = 20;
/* Maximum corner allowed to be taken. */
static const uint8_t MAX_CORNER = 100;

/* Constants used for binary conversion.
 * BGR. Adjust for different color detection. */
static const cv::Scalar MIN_VALS(20, 80, 0);
static const cv::Scalar MAX_VALS(180, 150, 120);

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

/* Dimension constants. */
static const int WIDTH = 1920;
static const int HEIGHT = 1080;
static const cv::Point ORIGIN(480,960);

LineFollower::LineFollower() : it(nh) {
    /* Set compress image stream enabled. */
    image_transport::TransportHints hints("compressed", ros::TransportHints());

	/* Subscribe to input channel. */
    image_sub = it.subscribe("/camera/image", 1, &LineFollower::imageCallback, this, hints);

	/* Publish to /cmd_vel. */
	twist_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
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
	bestDirection(lines, bgr_img);

	/* Code below is for debugging purposes only.*/

	/* Resize image to suitable size and show the result. */
	cv::resize(bgr_img, bgr_img, cv::Size(HEIGHT / 2, HEIGHT / 2));
	cv::imshow("Detected line image", bgr_img);
	cv::waitKey(3);
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

cv::Vec4i LineFollower::findClosest(std::vector<cv::Vec4i>& lines) {
	cv::Vec4i closest;
	/* Initialize the minimum distance to the largest value possible. */
	float min_dist = ORIGIN.y;
	float dist;

	/* Find the line that is the closest to the origin. */
	for (int i = 0; i < lines.size(); i++) {
		cv::Vec4i line = lines.at(i);
		dist = std::min(
			cv::norm(cv::Point2i(lines[i][0], lines[i][1]) - ORIGIN), 
			cv::norm(cv::Point2i(lines[i][2], lines[i][3]) - ORIGIN)
		);
		if (dist < min_dist) {
			min_dist = dist;
			closest = line;
		}
	}
	return closest;
}

void LineFollower::bestDirection(std::vector<cv::Vec4i>& lines, cv::Mat& img) {
	/* Initialize statically, less than ros::Time::now() because otherwise
	 * the first 250ms are wasted. */
	static ros::Time last = ros::Time::now() - ros::Duration(1);

	/* Don't evaluate the line if the previous command was sent less than 250ms ago. */
	if ((ros::Time::now() - last).toSec() * 1000 < MIN_DELAY) {
		return;
	}

	cv::Vec4i c = findClosest(lines);
	cv::Point2i p1(c[0], c[1]);
	cv::Point2i p2(c[2], c[3]);

	/* Draw the closest line. */ 
	cv::line(img, p1, p2, cv::Scalar(255,0,0), 3, CV_AA);

	/* Calculate the angle in degrees w.r.t the y-axis. */
	int angle = (atan2(c[3] - c[1], c[2] - c[0]) - CV_PI / 2) * 180 / CV_PI;

	/* Generate a Twist message. */
	if (angle > FW_CORNER_ALLOWED) {
		generateTwist(RIGHT, angle);
	} else if (angle < -FW_CORNER_ALLOWED) {
		generateTwist(LEFT, angle);	
	} else {
		generateTwist(FORWARD, angle);
	}
}

void LineFollower::generateTwist(int dir, int angle) {
	/* Correct for angles going out of bounds. */	
	if (angle < -MAX_CORNER)
		angle += 180;
	else if (angle > MAX_CORNER)
		angle -= 180;

	/* Create and publish a Twist message indicating the speed, angle and direction. */
	geometry_msgs::Twist twist;
    twist.angular.z = angle;
	/* Adjust speed if a corner is to be taken. */
    twist.linear.x = dir == FORWARD ? FW_SPEED : TURN_SPEED;
	twist_pub.publish(twist);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "image_stream");
	LineFollower lf;
	ros::spin();
	return 0;
}
