#include "ros/ros.h"
#include "ros/time.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"

// Module "core"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

// Module "highgui"
#include <opencv2/highgui/highgui.hpp>

// Module "imgproc"
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/features2d/features2d.hpp>
// Module "video"
#include <opencv2/video/video.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/opencv.hpp>

#include <image_transport/image_transport.h>

#include <sensor_msgs/image_encodings.h>
// Output
#include <iostream>
#include <sstream>
// Vector
#include <vector>

using namespace std;
using namespace cv;
// >>>>> Color to be tracked
#define MIN_H_BLUE 200
#define MAX_H_BLUE 300

int camera_video_number;
bool print_tracker_changes = false;
bool first_loop = true;

int max_hue = 90, max_saturation = 255, max_gain = 63, max_exposure = 255;
int exposure = 120,hue = 46, gain = 16,saturation = 255;
const int max_value_H = 360/2;
const int max_value = 255;
const String window_capture_name = "Video Capture";
const String window_detection_name = "Object Detection";
int low_H = 0, low_S = 0, low_V = 254;
int high_H = max_value_H, high_S = 68, high_V = max_value;
int pause_button = 0;
int frames_to_average = 2;
int max_frames_to_average = 15;
static void on_low_H_thresh_trackbar(int, void *)
{
    low_H = min(high_H-1, low_H);
    setTrackbarPos("Low H", window_detection_name, low_H);
    if (print_tracker_changes)
	    ROS_INFO_STREAM("low_H: " << low_H);
}
static void on_high_H_thresh_trackbar(int, void *)
{
    high_H = max(high_H, low_H+1);
    setTrackbarPos("High H", window_detection_name, high_H);
    if (print_tracker_changes)
	    ROS_INFO_STREAM("high_H: " << high_H);
}
static void on_low_S_thresh_trackbar(int, void *)
{
    low_S = min(high_S-1, low_S);
    setTrackbarPos("Low S", window_detection_name, low_S);
    if (print_tracker_changes)
	    ROS_INFO_STREAM("low_S: " << low_S);
}
static void on_high_S_thresh_trackbar(int, void *)
{
    high_S = max(high_S, low_S+1);
    setTrackbarPos("High S", window_detection_name, high_S);
    if (print_tracker_changes)
	    ROS_INFO_STREAM("high_S: " << high_S);
}
static void on_low_V_thresh_trackbar(int, void *)
{
    low_V = min(high_V-1, low_V);
    setTrackbarPos("Low V", window_detection_name, low_V);
    if (print_tracker_changes)
	    ROS_INFO_STREAM("low_V: " << low_V);
}
static void on_high_V_thresh_trackbar(int, void *)
{
    high_V = max(high_V, low_V+1);
    setTrackbarPos("High V", window_detection_name, high_V);
    if (print_tracker_changes)
	    ROS_INFO_STREAM("high_V: " << high_V);
}
bool clear_deque = false;
static void on_average_frames_trackbar(int, void *)
{
    setTrackbarPos("Average n frames", window_detection_name, frames_to_average);
    if (print_tracker_changes)
	    ROS_INFO_STREAM("frames_to_average: " << frames_to_average);
    clear_deque = true;
}

static void on_pause_button_trackbar(int, void *){
    setTrackbarPos("Pause", window_detection_name, pause_button);
    ROS_INFO_STREAM("pause_button: " << pause_button);
}

void setV4lParameter(int idx, const char*data, int value){
    //Create string container
	stringstream ss;
    //Sub in camera id number (idx), and required parameter and value
	ss << "uvcdynctrl --device=/dev/video" << idx <<  " -s '" << data << "' -- " << value;
    //Print string to shell to call uvcdynctrl to set parameter in v4l 
	system( ss.str().c_str() );
    //Clear ss
	ss.str("");
}

void Hue(int, void *){
	setV4lParameter(camera_video_number, "Hue", hue);
	return;
}
void Saturation(int, void *){
	setV4lParameter(camera_video_number, "Saturation", saturation);
	return;
}
void Gain(int, void *){
	setV4lParameter(camera_video_number, "Gain", gain);
	return;
}
void Exposure(int, void *){
	setV4lParameter(camera_video_number, "Exposure", exposure);
	return;
}



void createTrackbars(void){
    /// Create Windows
    namedWindow(window_detection_name, CV_WINDOW_NORMAL);

    createTrackbar("Low H", window_detection_name, &low_H, max_value_H, on_low_H_thresh_trackbar);
    createTrackbar("High H", window_detection_name, &high_H, max_value_H, on_high_H_thresh_trackbar);
    createTrackbar("Low S", window_detection_name, &low_S, max_value, on_low_S_thresh_trackbar);
    createTrackbar("High S", window_detection_name, &high_S, max_value, on_high_S_thresh_trackbar);
    createTrackbar("Low V", window_detection_name, &low_V, max_value, on_low_V_thresh_trackbar);
    createTrackbar("High V", window_detection_name, &high_V, max_value, on_high_V_thresh_trackbar);
    createTrackbar("Average n frames", window_detection_name, &frames_to_average, max_frames_to_average, on_average_frames_trackbar);
    createTrackbar("Pause", window_detection_name, &pause_button, 1, on_pause_button_trackbar);

    createTrackbar("Hue",window_detection_name, &hue, max_hue, Hue);
    createTrackbar("Saturation",window_detection_name, &saturation, max_saturation, Saturation);
    createTrackbar("Gain",window_detection_name, &gain, max_gain, Gain);
    createTrackbar("Exposure",window_detection_name, &exposure, max_exposure, Exposure);

}

int mX = -1;
int mY = -1;
bool clicked = false;
void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
	if  ( event == EVENT_LBUTTONDOWN ){
                clicked = true;
		mX = x;
		mY = y;
		cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
	}
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "opencv_laser_node");
	ros::NodeHandle nh("~");
	ros::Rate loop_rate(200);	
	nh.param("camera_ID", camera_video_number, 0);
  	ROS_INFO_STREAM("camera_ID: " << camera_video_number);


        namedWindow(window_capture_name);

	createTrackbars();
	cv::VideoCapture cap;
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 240);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 320);
	cap.set(CV_CAP_PROP_FPS, 187);

	if (!cap.open(camera_video_number))
	{
		cout << "Webcam not connected.\n" << "Please verify\n";
		return EXIT_FAILURE;
	}

   	// Set parameters in qv4l. Hue, saturation, gain and exposure can be modified with trackbars
	setV4lParameter(camera_video_number, "White Balance, Automatic", 0);
	setV4lParameter(camera_video_number, "Gain, Automatic", 0);
	// set auto exposure off, for some reason it's inverted in v4l
	setV4lParameter(camera_video_number, "Auto Exposure", 1);
	setV4lParameter(camera_video_number, "Hue", hue);
	setV4lParameter(camera_video_number, "Saturation", saturation);
	setV4lParameter(camera_video_number, "Gain", gain);
	setV4lParameter(camera_video_number, "Exosure", exposure);

	char ch = 0;
        Mat frame, frame_HSV, frame_threshold, last_threshold;
   	deque<Mat> ring;
	cap >> frame;

	while ((ch != 'q' && ch != 'Q') && ros::ok())
	{

		if(frame.empty())
			break;

		if (clicked){
			//cap >> frame;
			Mat HSV;
			ROS_INFO_STREAM("x: " << mX << ". y: " << mY);
			Mat RGB=frame(Rect(mX,mY,1,1));
			cvtColor(RGB, HSV,CV_BGR2HSV);
			Vec3b hsv=HSV.at<Vec3b>(0,0);
			int H=hsv.val[0]; //hue
			int S=hsv.val[1]; //saturation
			int V=hsv.val[2]; //value
                        ROS_INFO_STREAM("Hue: " << H << " Sat: " << S << " Value: " << V);
			clicked = false;
		}
		if (pause_button == 0)
			cap >> frame;
		setMouseCallback(window_capture_name, CallBackFunc, NULL);

		// Convert from BGR to HSV colorspace
		cvtColor(frame, frame_HSV, COLOR_BGR2HSV);
		// Detect the object based on HSV Range Values
		inRange(frame_HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), frame_threshold);

		// Show the frames
		imshow(window_capture_name, frame);

		Mat converted_threshold;
		frame_threshold.convertTo(converted_threshold, CV_32FC3);
		if (clear_deque){
			ring.clear();
			clear_deque = false;
		}
		ring.push_back(converted_threshold);
		if (ring.size() > frames_to_average) 
		{
		    ring.pop_front();

		    Mat avgImg(converted_threshold.size(), converted_threshold.type(), Scalar());

		    for (int i=0; i<frames_to_average; i++)
		        accumulate(ring[i], avgImg);
		imshow(window_detection_name, avgImg);		
		}
	        ros::spinOnce();
		loop_rate.sleep();
		ch = cv::waitKey(1);
	}
}



