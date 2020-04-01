#include "opencv2/opencv.hpp"
#include <time.h>
#include "ros/ros.h"

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "camera_fps_test_node");
	ros::NodeHandle nh("~");
	int camera_video_number, width, height, num_frames;
	double desired_fps;
	bool monochrome;
	nh.param("monochrome", monochrome, false);
	nh.param("camera_ID", camera_video_number, 0);
	nh.param("width", width, 240);
	nh.param("height", height, 320);
	nh.param("desired_fps", desired_fps, 187.0);
	nh.param("num_frames", num_frames, 120);
  	ROS_INFO_STREAM("camera_ID: " << camera_video_number);
  	ROS_INFO_STREAM("width: " << width);
  	ROS_INFO_STREAM("height: " << height);
  	ROS_INFO_STREAM("desired_fps: " << desired_fps);
  	ROS_INFO_STREAM("num_frames: " << num_frames);
	if (monochrome)
	  	ROS_INFO_STREAM("Black and White");
	else
	  	ROS_INFO_STREAM("Colour");

    // Start default camera
    VideoCapture video(camera_video_number);
    video.release();
    if (!video.open(camera_video_number))
    {
        cout << "Webcam not connected.\n" << "Please verify\n";
        return EXIT_FAILURE;
    }
    if (monochrome) {
      video.set(CAP_PROP_MODE, CAP_MODE_GRAY);
    }
    
    // With webcam get(CV_CAP_PROP_FPS) does not work.
    // Let's see for ourselves.
	video.set(CV_CAP_PROP_FRAME_WIDTH, width);
	video.set(CV_CAP_PROP_FRAME_HEIGHT, height);
	video.set(CV_CAP_PROP_FPS, desired_fps);
          //video.set(CV_CAP_PROP_FPS, 100);
    
    double fps_read = video.get(CV_CAP_PROP_FPS);
    double width_read = video.get(CV_CAP_PROP_FRAME_WIDTH);
    double height_read = video.get(CV_CAP_PROP_FRAME_HEIGHT);
    // If you do not care about backward compatibility
    // You can use the following instead for OpenCV 3
    // double fps_read = video.get(CAP_PROP_FPS);
    cout << "Frames per second using video.get(CV_CAP_PROP_FPS) : " << fps_read << endl;
    
    // Start and end times
    time_t start, end;
    
    // Variable for storing video frames
    Mat frame;

    // Grab a few frames
    for(int i = 0; i < 50; i++)
    {
        video >> frame;
    }

    ROS_INFO_STREAM("width_read x height_read " << width_read << ", " << height_read);
    ROS_INFO_STREAM("frame.cols x frame.rows: " << frame.cols << ", " << frame.rows);
    
    cout << "Capturing " << num_frames << " frames" << endl ;

    // Start time
    time(&start);
    
    // Grab a few frames
    for(int i = 0; i < num_frames; i++)
    {
        video >> frame;
    }
    
    // End Time
    time(&end);
    
    // Time elapsed
    double seconds = difftime (end, start);
    cout << "Time taken : " << seconds << " seconds" << endl;
    
    // Calculate frames per second
    fps_read  = num_frames / seconds;
    cout << "Estimated frames per second : " << fps_read << endl;
    
    // Release video
    video.release();
    return 0;
}
