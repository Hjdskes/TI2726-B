/**
 * Group number: 28
 * Student 1:
 * Jente Hidskes, 4335732
 * Student 2:
 * Piet van Agtmaal, 4321278
 */

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "linefollower_18.h"

// FIXME: ik wil nog steeds weten hoe het met die it(nh) zit :p
LineFollower::LineFollower() : it(nh) {
    /* Set compress image stream enabled. */
    image_transport::TransportHints hints("compressed", ros::TransportHints());

	/* Subscribe to input channel. */
    image_sub = it.subscribe("/camera/image", 1, &LineFollower::imageCallback, this, hints);
}

void LineFollower::imageCallback(const sensor_msgs::ImageConstPtr& color_img) {
	// convert ROS image stream to OpenCV image, using color channels BGR (8-bit)
	cv_bridge::CvImagePtr img_ptr;
	cv::Mat img_bgr;
	try {
		img_ptr = cv_bridge::toCvCopy(color_img, 
			sensor_msgs::image_encodings::BGR8);
		img_bgr = img_ptr->image;
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
    }

	cv::Mat img_binary;
	//BGR. adjust for different color detection.
	// FIXME: find perfect values
	cv::Scalar min_vals(0, 100, 0);
	cv::Scalar max_vals(200, 200, 200);
	// create binary image from the bgr image
	cv::inRange(img_bgr, min_vals, max_vals, img_binary);

	// blur img. Needed for detecting edges
	cv::Mat img_edges;	
	unsigned int kernel_size = 3;
	cv::blur(img_binary, img_edges, cv::Size(kernel_size, kernel_size));

	// detect edges
	//FIXME: find perfect values
	double low_thr = 50;
	cv::Canny(img_edges, img_edges, low_thr, 4*low_thr, kernel_size);    

	// Create a vector to save detected lines
	std::vector<cv::Vec4i> lines;

	// Hough transform. Saves detected lines in vector 'lines'
	// FIXME: find perfect values
	double r = 1;
	double theta = CV_PI/180;
	unsigned int min_intersects = 50;
	unsigned int min_points = 22;
	unsigned int max_gap = 50;
	cv::HoughLinesP(img_edges, lines, r, theta, min_intersects, 
		min_points, max_gap);

	// draw lines detected with Hough
	cv::Mat img_detected = img_bgr.clone();
	cv::Scalar line_color(0,0,255);
	unsigned int line_width = 3;

	for (size_t i = 0; i < lines.size(); i++) {
		// add each seperate line to the image
		cv::Point point1(lines[i][0], lines[i][1]);
		cv::Point point2(lines[i][2], lines[i][3]);
		// draw anti-aliased lines
		cv::line(img_detected, point1, point2, line_color, 
			line_width, CV_AA);
	}

	/* Show result. */
	cv::imshow("Detected line image", img_detected);
	cv::waitKey(3);	

}

int main(int argc, char** argv) {
	ros::init(argc, argv, "image_stream");
	LineFollower lf;
	ros::spin();
	return 0;
}
