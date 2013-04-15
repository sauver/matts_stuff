
// Programmed by Matt
// Converted by Iljoo APR 5 2013


#include <ros/ros.h>
#include <ros/node_handle.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <string.h>
#include <math.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>

#define PI 3.14159265

static const char WIN_SRC[] = "UAV Color Threshold";
 
int g_slider_position_h1 = 0;
int g_slider_position_s1 = 0;
int g_slider_position_v1 = 0;
 
int g_slider_position_h2 = 255;
int g_slider_position_s2 = 255;
int g_slider_position_v2 = 255;
 
int g_slider_position_smooth = 0;

int g_slider_position_cam = 1 ;
 

CvScalar uct_min = cvScalar(120, 120, 120, 120);
CvScalar uct_max = cvScalar(134, 134, 134, 134);


class uavColorThresh
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	ros::Publisher pos_pub;
	ros::Publisher center_pub;
	
	IplImage *hsv_img;
	IplImage *thresholded_img;  // red
	IplImage *thresholded_img_y;  // yellow
	IplImage *added;

	CvMoments *moments;  // red
	CvMoments *moments_y; // yellow

	double area;
	double area_y;

	//geometry_msgs::Point uav_pos;
	//geometry_msgs::PoseArray centers;
	//geometry_msgs::Pose red_center;
	//geometry_msgs::Pose yellow_center;
  
public:
  	uavColorThresh()
    	: it_(nh_)
  	{
		image_sub_ = it_.subscribe("/network/image_raw_filt", 1, &uavColorThresh::imageCb, this,
					image_transport::TransportHints("raw"));
		pos_pub = nh_.advertise<geometry_msgs::Point>( "/uav/pose", 1);
		center_pub = nh_.advertise<geometry_msgs::PoseArray>( "/uav/centers", 1);

		hsv_img = NULL;
		thresholded_img = NULL;
		thresholded_img_y = NULL;
		moments = NULL;
		moments_y = NULL;
		area = 0;
		area_y = 0;
		cvNamedWindow( WIN_SRC,0);
  	}

	~uavColorThresh()
	{
		cv::destroyWindow(WIN_SRC);
	}

	void imageCb(const sensor_msgs::ImageConstPtr& msg_ptr)
	{
		sensor_msgs::CvBridge bridge_;
		IplImage *cv_ptr = NULL;
		try
		{
			cv_ptr = bridge_.imgMsgToCv(msg_ptr, "bgr8");
		}
		catch (sensor_msgs::CvBridgeException error)
		{
			ROS_ERROR("error");
		}

	  geometry_msgs::Point uav_pos;
	  geometry_msgs::PoseArray centers;
 	  geometry_msgs::Pose red_center;
	  geometry_msgs::Pose yellow_center;

		// blur the source image to reduce color noise e
		cvSmooth( cv_ptr, cv_ptr, CV_MEDIAN, 3, 0, 0, 0 );

		// convert the image to hsv(Hue, Saturation, Value) so its
		// easier to determine the color to track(hue)
		hsv_img = cvCreateImage(cvGetSize(cv_ptr), 8, 3);
		cvCvtColor(cv_ptr, hsv_img, CV_BGR2HSV);

		// limit all pixels that don't match our criteria, in this case we are
		// looking for purple but if you want you can adjust the first value in
		// both turples which is the hue range(120,140). OpenCV uses 0-180 as
		// a hue range for the HSV color model
		thresholded_img = cvCreateImage(cvGetSize(hsv_img), 8, 1);
    	thresholded_img_y = cvCreateImage(cvGetSize(hsv_img), 8, 1);
    	uct_min.val[0] = 0; // h
		uct_min.val[1] = 0; // s
		uct_min.val[2] = 222; // v
		uct_max.val[0] = 180; // h
		uct_max.val[1] = 255; // s
		uct_max.val[2] = 255; // v
    	cvInRangeS (hsv_img, uct_min, uct_max, thresholded_img); // red

    	uct_min.val[0] = 31; // h
		uct_min.val[1] = 178; // s
		uct_min.val[2] = 64; // v
		uct_max.val[0] = 180; // h
		uct_max.val[1] = 255; // s
		uct_max.val[2] = 255; // v
		cvInRangeS (hsv_img, uct_min, uct_max, thresholded_img_y); // yellow

		// determine the objects moments and check that the area is large
		// enough to be our object
		moments = (CvMoments*)malloc(sizeof(CvMoments));
		moments_y = (CvMoments*)malloc(sizeof(CvMoments));

    	cvErode(thresholded_img_y,thresholded_img_y,NULL,4);
		cvDilate(thresholded_img_y, thresholded_img_y,NULL,4);
		cvErode(thresholded_img, thresholded_img,NULL,4);
		cvDilate(thresholded_img, thresholded_img,NULL,4);
		cvMoments(thresholded_img, moments, 1);
		area = cvGetCentralMoment(moments, 0, 0);
		cvMoments(thresholded_img_y, moments_y, 1);
		area_y = cvGetCentralMoment(moments_y, 0, 0);


		// there can be noise in the video so ignore objects with small areas
		if((area > 100) && (area_y > 100)) {
			double x = cvGetSpatialMoment(moments, 1, 0)/area;
			double y = cvGetSpatialMoment(moments, 0, 1)/area;
			double x_y = cvGetSpatialMoment(moments_y, 1, 0)/area_y;
			double y_y = cvGetSpatialMoment(moments_y, 0, 1)/area_y;
			red_center.position.x = y;
			red_center.position.y = x;
			yellow_center.position.x = y_y;
			yellow_center.position.y = x_y;
			centers.poses.push_back(red_center);
			centers.poses.push_back(yellow_center);

			double angle = atan2(x - x_y, y - y_y)*180/PI;
			// X and y are flipped to put it into the robot frame
			double uav_x = (y+y_y)/2;
			double uav_y = (x+x_y)/2;
			uav_pos.x = uav_x;
			uav_pos.y = uav_y;
			uav_pos.z = angle;
			// flip x and y to put it in the UAV's frame
			cvCircle(cv_ptr,cvPoint(int(uav_y), int(uav_x)),5,CV_RGB(0,255,255));
			cvLine(cv_ptr,cvPoint(int(uav_y), int(uav_x)),cvPoint(int(uav_y - 20*sin(angle*PI/180)), int(uav_x - 20*cos(angle*PI/180))), CV_RGB(0,255,255),3,8,0);
		}
		pos_pub.publish(uav_pos);
		center_pub.publish(centers);
		added = cvCreateImage(cvGetSize(cv_ptr), 8, 1);
		cvAdd(thresholded_img, thresholded_img_y, added);

		//cvShowImage(WIN_SRC,cv_ptr);
		cvShowImage(WIN_SRC,added);
		// Wait for a keypress
		int c = cvWaitKey(1);
		if(c!=-1)
		{
			// If pressed, break out of the loop
			return ;
		}

		cvReleaseImage(&hsv_img);
		cvReleaseImage(&thresholded_img);
		cvReleaseImage(&thresholded_img_y);
		cvReleaseImage(&added);

		delete moments;
		delete moments_y;

	} // end of void imageCb(const sensor_msgs::ImageConstPtr& msg)
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "uavColorThresh");
	uavColorThresh ucf;
	ros::spin();
}

