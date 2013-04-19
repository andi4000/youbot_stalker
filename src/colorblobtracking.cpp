#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"

#include <string.h>

/**
 * //TODO
 * - use ros::NodeHandle::setParam to send resolution information to parameter server
 * - develop better image recognition!!
 * - use getopt for better flag handling
 * 
 */

using namespace cv;
using namespace std;

bool displayWindows = false;
bool verbose = false;

Mat getHSVThresholdedImg(const Mat& src, int lo_h, int lo_s, int lo_v, int hi_h, int hi_s, int hi_v)
{
	Mat matHSV;
	cvtColor(src, matHSV, CV_BGR2HSV);
	
	Mat matThresh;
	inRange(matHSV, Scalar(lo_h, lo_s, lo_v), Scalar(hi_h, hi_s, hi_v), matThresh);
	
	erode(matThresh, matThresh, Mat());
	
	return matThresh;
}

void doContourProcessing(const Mat& matSrc, const Mat& matThresh, Point& out_centroid, bool& out_gotIt, double& out_maxArea, Mat& out_mat)
{
	double minimalArea = 2000;
	Mat matOutput(matSrc);
	
	vector< vector<Point> > contours;
	findContours(matThresh, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	
	vector<double> areas(contours.size());
	for (unsigned int i = 0; i < contours.size(); i++)
		areas[i] = contourArea(Mat(contours[i]));
	
	double max_area;
	Point maxPosition;
	minMaxLoc(Mat(areas), 0, &max_area, 0, &maxPosition);
	//ROS_INFO("biggest area = %f", max_area);
	
	bool got_it = false;
	if (max_area > minimalArea) got_it = true;
	
	// draw only the biggest contour
	drawContours(matOutput, contours, maxPosition.y, Scalar(0, 0, 255), CV_FILLED);
	
	// GETTING THE CENTROID BEGIN
	Point image_centroid(0, 0);
	Point relative_centroid(0, 0);
	if (got_it){
		vector<Moments> mu(contours.size());
		for ( unsigned int i = 0; i < contours.size(); i++ ) {
			mu[i] = moments(contours[i], false);
		}
		
		vector<Point> mc(contours.size());
		for ( unsigned int i = 0; i < contours.size(); i++ ) {
			mc[i] = Point( mu[i].m10/mu[i].m00, mu[i].m01/mu[i].m00 );
		}
		
		// draw the centroid of the biggest contour
		image_centroid = mc[maxPosition.y];
		circle(matOutput, image_centroid, 4, Scalar(255, 0, 0), -1, 8, 0);
		//ROS_INFO("image centroid (x, y) = (%d, %d)", image_centroid.x, image_centroid.y);
		
		// getting the relative centroid (middle point as 0,0)
		relative_centroid.x = image_centroid.x - (matSrc.cols / 2);
		relative_centroid.y = -image_centroid.y + (matSrc.rows / 2);
		if (verbose)
			ROS_INFO("relative centroid (x, y) = (%d, %d) area %.2f", relative_centroid.x, relative_centroid.y, max_area);
	} else {
		if (verbose)
			ROS_INFO("nothing");
	}
	// GETTING THE CENTROID END
	
	//TODO: this is ugly
	out_centroid = relative_centroid;
	out_maxArea = max_area;
	out_gotIt = got_it;
	out_mat = matOutput;
}

//Ref: http://opencv.willowgarage.com/documentation/cpp/imgproc_feature_detection.html#cv-houghcircles
void doGetTheBall(const Mat& matSrc, Mat& out_mat)
{
	//Mat matOutput(matSrc);
	Mat gray;
	if (matSrc.channels() > 1)
		cvtColor(matSrc, gray, CV_BGR2GRAY);
	else
		gray = matSrc;
	GaussianBlur(gray, gray, Size(9, 9), 2, 2);
	vector<Vec3f> circles;
	HoughCircles(gray, circles, CV_HOUGH_GRADIENT, 2, gray.rows/4, 200, 100);
	
	for (size_t i = 0; i < circles.size(); i++){
		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		int radius = cvRound(circles[i][2]);
		
		circle(out_mat, center, 3, Scalar(0, 0, 255), -1, 8, 0);
		circle(out_mat, center, radius, Scalar(0, 0, 255), 3, 8, 0);
	}
	
	//out_mat = matOutput;
}

int main(int argc, char** argv)
{
	int capSizeX = 640;
	int capSizeY = 480;
	
	int camIndex = 0;
	for (int i = 0; i < 5; i++){
		VideoCapture tmpCap(i);
		if (tmpCap.isOpened()){
			camIndex = i;
			tmpCap.release();
			ROS_INFO("Using /dev/video%d", camIndex);
			break;
		}
		tmpCap.release();
	}
	VideoCapture cap(camIndex);
	
	if (!cap.isOpened()){
		ROS_ERROR("Failed to open camera at /dev/video%d", camIndex);
		return -1;
	} else {
		ROS_INFO("Opening /dev/video%d", camIndex);
	}
	
	if (argv[1] && strcmp(argv[1], "-d") == 0)
		displayWindows = true;
	else
		ROS_INFO("To display image, use option \"-d\"");
	
	// ROS MESSAGE BEGIN
	// Ref: http://www.ros.org/wiki/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
	ros::init(argc, argv, "object_tracking");
	ros::NodeHandle n;
	ros::Publisher object_detected_pub = n.advertise<std_msgs::Bool>("object_tracking/object_detected", 1000);
	ros::Publisher x_pos_pub = n.advertise<std_msgs::Int32>("object_tracking/x_pos", 1000);
	ros::Publisher y_pos_pub = n.advertise<std_msgs::Int32>("object_tracking/y_pos", 1000);
	ros::Publisher area_pub = n.advertise<std_msgs::Float32>("object_tracking/area", 1000);
	ros::Rate loop_rate(50);
	// ROS MESSAGE END
	
	n.setParam("/object_tracking/captureSizeX", capSizeX);
	n.setParam("/object_tracking/captureSizeY", capSizeY);
	
	
	
	if (displayWindows){
		namedWindow("Output", CV_WINDOW_AUTOSIZE);
		namedWindow("Threshold", CV_WINDOW_AUTOSIZE);
		moveWindow("Output", 0*capSizeX, 0);
		moveWindow("Threshold", 1*capSizeX, 0);
	}

	// HSV max and min values in opencv
	//int lo_h = 0, lo_s = 0, lo_v = 0, hi_h = 180, hi_s = 255, hi_v = 255;
	// this is for green
	int lo_h = 23, lo_s = 68, lo_v = 81, hi_h = 60, hi_s = 174, hi_v = 228;
	createTrackbar("Lo H", "Threshold", &lo_h, 180);
	createTrackbar("Lo S", "Threshold", &lo_s, 255);
	createTrackbar("Lo V", "Threshold", &lo_v, 255);
	createTrackbar("Hi H", "Threshold", &hi_h, 180);
	createTrackbar("Hi S", "Threshold", &hi_s, 255);
	createTrackbar("Hi V", "Threshold", &hi_v, 255);
	
	
	ROS_INFO("Starting object detection");
	// ROS SIGINT handler
	while(ros::ok()){
		Mat srcFrame;
		cap >> srcFrame;
		resize(srcFrame, srcFrame, Size(capSizeX, capSizeY));
		
		Mat matThresh;
		matThresh = getHSVThresholdedImg(srcFrame, lo_h, lo_s, lo_v, hi_h, hi_s, hi_v);
		dilate(matThresh, matThresh, Mat());
		erode(matThresh, matThresh, Mat());
		
		Mat matOutput(srcFrame);
		Point relative_centroid(0, 0);
		double max_area;
		bool got_it;
		
		//TODO: object detection call here
		doContourProcessing(srcFrame, matThresh, relative_centroid, got_it, max_area, matOutput); 
		//doGetTheBall(matThresh, matOutput);
		
		if (displayWindows){
			imshow("Output", matOutput);
			imshow("Threshold", matThresh);
		}
				
		// ROS MESSAGE BEGIN
		std_msgs::Bool msg_obj_det;
		std_msgs::Int32 msg_x_pos, msg_y_pos;
		std_msgs::Float32 msg_area;
		
		msg_obj_det.data = got_it;
		msg_x_pos.data = relative_centroid.x;
		msg_y_pos.data = relative_centroid.y;
		msg_area.data = max_area;
		
		object_detected_pub.publish(msg_obj_det);
		x_pos_pub.publish(msg_x_pos);
		y_pos_pub.publish(msg_y_pos);
		area_pub.publish(msg_area);
		
		ros::spinOnce();
		loop_rate.sleep();
		// ROS MESSAGE END
		
		// press ESC to exit
		if ( (waitKey(10) & 255) == 27 ) break;
	}
	
	//cvReleaseCapture(&capture);
	if (displayWindows)
		destroyAllWindows();
	
	return 0;
}


