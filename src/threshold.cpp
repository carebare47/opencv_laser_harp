#include "ros/ros.h"
#include "ros/time.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include <opencv_laser_harp/threshold.h>

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
#include <stdlib.h>
#include <chrono>
#include "portaudio.h"

typedef std::chrono::high_resolution_clock Clock;


using namespace std;
using namespace cv;
// >>>>> Color to be tracked
#define MIN_H_BLUE 200
#define MAX_H_BLUE 300

int camera_video_number;
bool print_tracker_changes = false;

int frame_width = 240;
int frame_height = 320;

float corner_fraction = 9.0/10.0;

int max_hue = 90, max_saturation = 255, max_gain = 63, max_exposure = 255;
const int max_value_H = 360/2;
const int max_value = 255;



// Mid-Day & sunny:
//int exposure = 81, hue = 90, gain = 7, saturation = 255;
//int low_H = 0, high_H= 180, low_S= 0, high_S= 16, low_V= 254, high_V= 255;
// Mid-Day & reasonable:
//int exposure = 67, hue = 58, gain = 11, saturation = 255;
//int low_H = 0, high_H= 180, low_S= 0, high_S= 158, low_V= 254, high_V= 255;
// Better for night: 
int exposure = 120,hue = 46, gain = 16,saturation = 255;
int low_H = 0, low_S = 0, low_V = 254;
int high_H = max_value_H, high_S = 68, high_V = max_value;	

int audio_threshold = 2000;

const String camera_feed_window_name = "Video Capture";
const String threshold_window_name = "Laser Detection";
int pause_button = 0;
int print_button = 0;
float image_show_magnification = 1.0;

int frames_to_average = 2;
int max_frames_to_average = 15;
static void on_low_H_thresh_trackbar(int, void *)
{
    low_H = min(high_H-1, low_H);
    setTrackbarPos("Low H", threshold_window_name, low_H);
    if (print_tracker_changes)
	    ROS_INFO_STREAM("low_H: " << low_H);
}
static void on_high_H_thresh_trackbar(int, void *)
{
    high_H = max(high_H, low_H+1);
    setTrackbarPos("High H", threshold_window_name, high_H);
    if (print_tracker_changes)
	    ROS_INFO_STREAM("high_H: " << high_H);
}
static void on_low_S_thresh_trackbar(int, void *)
{
    low_S = min(high_S-1, low_S);
    setTrackbarPos("Low S", threshold_window_name, low_S);
    if (print_tracker_changes)
	    ROS_INFO_STREAM("low_S: " << low_S);
}
static void on_high_S_thresh_trackbar(int, void *)
{
    high_S = max(high_S, low_S+1);
    setTrackbarPos("High S", threshold_window_name, high_S);
    if (print_tracker_changes)
	    ROS_INFO_STREAM("high_S: " << high_S);
}
static void on_low_V_thresh_trackbar(int, void *)
{
    low_V = min(high_V-1, low_V);
    setTrackbarPos("Low V", threshold_window_name, low_V);
    if (print_tracker_changes)
	    ROS_INFO_STREAM("low_V: " << low_V);
}
static void on_high_V_thresh_trackbar(int, void *)
{
    high_V = max(high_V, low_V+1);
    setTrackbarPos("High V", threshold_window_name, high_V);
    if (print_tracker_changes)
	    ROS_INFO_STREAM("high_V: " << high_V);
}
bool clear_deque = false;
static void on_average_frames_trackbar(int, void *)
{
    setTrackbarPos("Average n frames", threshold_window_name, frames_to_average);
    if (print_tracker_changes)
	    ROS_INFO_STREAM("frames_to_average: " << frames_to_average);
    clear_deque = true;
}

static void on_pause_button_trackbar(int, void *){
    setTrackbarPos("Pause", threshold_window_name, pause_button);
    ROS_INFO_STREAM("pause_button: " << pause_button);
}

bool print_flag = false;
static void on_print_button_trackbar(int, void *){
    setTrackbarPos("Print Parameters", threshold_window_name, print_button);
    print_flag = true;
    ROS_INFO_STREAM("Print: " << print_button);
}

static void on_change_in_audio_threshold_trackbar(int, void *){
    setTrackbarPos("Pixel Threshold", threshold_window_name, audio_threshold);
    ROS_INFO_STREAM("audio_threshold: " << audio_threshold);
}

   /// createTrackbar("Pixel Threshold",threshold_window_name, &audio_threshold, 10000, audio_threshold);

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
    namedWindow(threshold_window_name, CV_WINDOW_AUTOSIZE);

    createTrackbar("Low H", threshold_window_name, &low_H, max_value_H, on_low_H_thresh_trackbar);
    createTrackbar("High H", threshold_window_name, &high_H, max_value_H, on_high_H_thresh_trackbar);
    createTrackbar("Low S", threshold_window_name, &low_S, max_value, on_low_S_thresh_trackbar);
    createTrackbar("High S", threshold_window_name, &high_S, max_value, on_high_S_thresh_trackbar);
    createTrackbar("Low V", threshold_window_name, &low_V, max_value, on_low_V_thresh_trackbar);
    createTrackbar("High V", threshold_window_name, &high_V, max_value, on_high_V_thresh_trackbar);
    createTrackbar("Average n frames", threshold_window_name, &frames_to_average, max_frames_to_average, on_average_frames_trackbar);
    createTrackbar("Pause", threshold_window_name, &pause_button, 1, on_pause_button_trackbar);
    createTrackbar("Print Parameters", threshold_window_name, &print_button, 1, on_print_button_trackbar);

    createTrackbar("Hue",threshold_window_name, &hue, max_hue, Hue);
    createTrackbar("Saturation",threshold_window_name, &saturation, max_saturation, Saturation);
    createTrackbar("Gain",threshold_window_name, &gain, max_gain, Gain);
    createTrackbar("Exposure",threshold_window_name, &exposure, max_exposure, Exposure);
    createTrackbar("Pixel Threshold",threshold_window_name, &audio_threshold, 10000, on_change_in_audio_threshold_trackbar);

//    createTrackbar("Pan Across",threshold_window_name, &x_camera_offset, max_pan, pan_camera_across);
//    createTrackbar("Pan Up",threshold_window_name, &y_camera_offset, max_pan, pan_camera_up);
//    createTrackbar("Stretch and Squish Across",threshold_window_name, &camera_zoom_amount_a, max_zoom, update_camera_zoom_a);
//    createTrackbar("Stretch and Squish Down",threshold_window_name, &camera_zoom_amount_b, max_zoom, update_camera_zoom_b);
}

int mX = -1;
int mY = -1;
bool clicked = false;
void CallBackFuncCameraFeed(int event, int x, int y, int flags, void* userdata)
{
	if  ( event == EVENT_LBUTTONDOWN ){
                clicked = true;
		mX = x;
		mY = y;
		cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
	}
}


void CallBackFuncThreshold(int event, int x, int y, int flags, void* userdata)
{
	if  ( event == EVENT_LBUTTONDOWN ){
                // clicked = true;
		ROS_INFO_STREAM("(frame_width/corner_fraction): " << (frame_width*corner_fraction) << ". (frame_height/corner_fraction): " << (frame_height - (frame_height*corner_fraction)));
		if ((x > (frame_width/corner_fraction)) && ((y > (frame_height - (frame_height/corner_fraction))))){
			ROS_INFO_STREAM("SUCCESS!");
		}
		cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
	}
}

void print_current_values(){
	ROS_INFO_STREAM("Exposure: " << exposure << ". Hue: " << hue << ". Gain: " << gain << ". Saturation: " << saturation);
	ROS_INFO_STREAM("low_H: " << low_H << ". high_H: " << high_H << ". low_S: " << low_S << ". high_S: " << high_S << ". low_V: " << low_V << ". high_V: " << high_V);
	ROS_INFO_STREAM("frames_to_average: " << frames_to_average << ". image_show_magnification: " << image_show_magnification << ". pause_button: " << pause_button);
        setTrackbarPos("Print Parameters", threshold_window_name, 0);
	print_flag = false;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "opencv_laser_node");
	ros::NodeHandle nh("~");
	ros::Rate loop_rate(200);	
	nh.param("camera_ID", camera_video_number, 0);
  	ROS_INFO_STREAM("camera_ID: " << camera_video_number);

	namedWindow(camera_feed_window_name, CV_WINDOW_AUTOSIZE);

	createTrackbars();
	cv::VideoCapture cap;
	cap.set(CV_CAP_PROP_FRAME_WIDTH, frame_width);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, frame_height);
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
        Mat frame, frame_HSV, frame_threshold;
   	deque<Mat> ring;
	cap >> frame;
	setMouseCallback(camera_feed_window_name, CallBackFuncCameraFeed, NULL);
	setMouseCallback(threshold_window_name, CallBackFuncThreshold, NULL);
	bool audio_on = false;

	while ((ch != 'q' && ch != 'Q') && ros::ok())
	{

		if (ch == '+')
			image_show_magnification += 0.2;

		if (ch == '-')
			image_show_magnification -= 0.2;

		if (ch == 'o')
			audio_on = !audio_on;

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

		if (print_button == 1)
			print_current_values();


		// Convert from BGR to HSV colorspace
		cvtColor(frame, frame_HSV, COLOR_BGR2HSV);
		// Detect the object based on HSV Range Values
		inRange(frame_HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), frame_threshold);

		// Show the frames
		Mat frame_scaled;
		resize(frame, frame_scaled, cv::Size(), image_show_magnification, image_show_magnification);
		imshow(camera_feed_window_name, frame_scaled);

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
		    Mat avgImg_large;		
		    resize(avgImg, avgImg_large, cv::Size(), image_show_magnification, image_show_magnification);
		    // putText(avgImg_large, "Click here", cvPoint(60,60), FONT_HERSHEY_COMPLEX_SMALL, 2.8, cvScalar(1,1,1), 1, CV_AA);
		    imshow(threshold_window_name, avgImg_large);
		    int count = cv::countNonZero(avgImg_large);
        	    ROS_INFO_STREAM("White Pixels: " << count);
	            if ((count < audio_threshold) && audio_on){
		        //system("canberra-gtk-play -f /home/user/tone.wav");
			std::string sp_test = "speaker-test -t sine -f ";
			std::string sp_args = " -c 2 -s 1 &";
			std::string result;
			
			result = sp_test + std::to_string(count * 0.044) + sp_args;
			system(result.c_str());
		        //system("mplayer /home/user/tone.wav &");
		        //usleep(2*1000*1000);
		    }
		}

	        ros::spinOnce();
		loop_rate.sleep();
		ch = cv::waitKey(1);
	}
}



