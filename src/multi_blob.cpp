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
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include <chrono>
#include "portaudio.h"

using namespace std;
using namespace cv;

int camera_video_number = 0;
bool print_tracker_changes = false;
bool pause_processing = false;

int frame_width = 240;
int frame_height = 320;

float corner_fraction = 9.0 / 10.0;

int max_hue = 90, max_saturation = 255, max_gain = 63, max_exposure = 255;
const int max_value_H = 360 / 2;
const int max_value = 255;

// Mid-Day & sunny:
//int exposure = 81, hue = 90, gain = 7, saturation = 255;
//int low_H = 0, high_H= 180, low_S= 0, high_S= 16, low_V= 254, high_V= 255;
// Mid-Day & reasonable:
//int exposure = 67, hue = 58, gain = 11, saturation = 255;
//int low_H = 0, high_H= 180, low_S= 0, high_S= 158, low_V= 254, high_V= 255;
// Better for night:
int exposure = 255;
int hue = 32;
int gain = 49;
int saturation = 86;
int low_H = 0;
int high_H = 180;
int low_S = 0;
int high_S = 87;
int low_V = 254;
int high_V = 255;





int audio_threshold = 2000;

const String camera_feed_window_name = "Video Capture";
const String threshold_window_name = "Laser Detection";
int pause_button = 0;
int print_button = 0;
float image_show_magnification = 1.0;
int max_min_area_threshold = 200;
int min_area_threshold = 1.0;

int frames_to_average = 2;
int max_frames_to_average = 15;
static void on_low_H_thresh_trackbar(int, void *)
{
	low_H = min(high_H - 1, low_H);
	setTrackbarPos("Low H", threshold_window_name, low_H);
	if (print_tracker_changes)
		ROS_INFO_STREAM("low_H: " << low_H);
}
static void on_high_H_thresh_trackbar(int, void *)
{
	high_H = max(high_H, low_H + 1);
	setTrackbarPos("High H", threshold_window_name, high_H);
	if (print_tracker_changes)
		ROS_INFO_STREAM("high_H: " << high_H);
}
static void on_low_S_thresh_trackbar(int, void *)
{
	low_S = min(high_S - 1, low_S);
	setTrackbarPos("Low S", threshold_window_name, low_S);
	if (print_tracker_changes)
		ROS_INFO_STREAM("low_S: " << low_S);
}
static void on_high_S_thresh_trackbar(int, void *)
{
	high_S = max(high_S, low_S + 1);
	setTrackbarPos("High S", threshold_window_name, high_S);
	if (print_tracker_changes)
		ROS_INFO_STREAM("high_S: " << high_S);
}
static void on_low_V_thresh_trackbar(int, void *)
{
	low_V = min(high_V - 1, low_V);
	setTrackbarPos("Low V", threshold_window_name, low_V);
	if (print_tracker_changes)
		ROS_INFO_STREAM("low_V: " << low_V);
}
static void on_high_V_thresh_trackbar(int, void *)
{
	high_V = max(high_V, low_V + 1);
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

static void on_pause_button_trackbar(int, void *)
{
	setTrackbarPos("Pause", threshold_window_name, pause_button);
	ROS_INFO_STREAM("pause_button: " << pause_button);
}

bool print_flag = false;
static void on_print_button_trackbar(int, void *)
{
	setTrackbarPos("Print Parameters", threshold_window_name, print_button);
	print_flag = true;
	ROS_INFO_STREAM("Print: " << print_button);
}

static void on_change_in_audio_threshold_trackbar(int, void *)
{
	setTrackbarPos("Pixel Threshold", threshold_window_name, audio_threshold);
	ROS_INFO_STREAM("audio_threshold: " << audio_threshold);
}

static void on_change_in_min_contour_area(int, void *)
{
	setTrackbarPos("Min Contour Area", threshold_window_name, min_area_threshold);
	ROS_INFO_STREAM("min_area_threshold: " << min_area_threshold);
}

/// createTrackbar("Pixel Threshold",threshold_window_name, &audio_threshold, 10000, audio_threshold);

void setV4lParameter(int idx, const char *data, int value)
{
	//Create string container
	stringstream ss;
	//Sub in camera id number (idx), and required parameter and value
	ss << "uvcdynctrl --device=/dev/video" << idx << " -s '" << data << "' -- " << value;
	//Print string to shell to call uvcdynctrl to set parameter in v4l
	system(ss.str().c_str());
	//Clear ss
	ss.str("");
}

void save_picture(Mat image, String path)
{
	//Mat gray_image;
	//cvtColor(image, gray_image, CV_BGR2GRAY);
	imwrite(path, image);
}

void Hue(int, void *)
{
	setV4lParameter(camera_video_number, "Hue", hue);
	return;
}
void Saturation(int, void *)
{
	setV4lParameter(camera_video_number, "Saturation", saturation);
	return;
}
void Gain(int, void *)
{
	setV4lParameter(camera_video_number, "Gain", gain);
	return;
}
void Exposure(int, void *)
{
	setV4lParameter(camera_video_number, "Exposure", exposure);
	return;
}

void createTrackbars(void)
{
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

	createTrackbar("Hue", threshold_window_name, &hue, max_hue, Hue);
	createTrackbar("Saturation", threshold_window_name, &saturation, max_saturation, Saturation);
	createTrackbar("Gain", threshold_window_name, &gain, max_gain, Gain);
	createTrackbar("Exposure", threshold_window_name, &exposure, max_exposure, Exposure);
	createTrackbar("Pixel Threshold", threshold_window_name, &audio_threshold, 10000, on_change_in_audio_threshold_trackbar);
	createTrackbar("Min Contour Area", threshold_window_name, &min_area_threshold, max_min_area_threshold, on_change_in_min_contour_area);
}

int mX = -1;
int mY = -1;
bool clicked = false;
void CallBackFuncCameraFeed(int event, int x, int y, int flags, void *userdata)
{
	if (event == EVENT_LBUTTONDOWN)
	{
		clicked = true;
		mX = x;
		mY = y;
		cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
	}
}

bool clicked_2 = false;
void CallBackFuncThreshold(int event, int x, int y, int flags, void *userdata)
{
	if (event == EVENT_LBUTTONDOWN)
	{
		clicked_2 = true;
		ROS_INFO_STREAM("(frame_width/corner_fraction): " << (frame_width * corner_fraction) << ". (frame_height/corner_fraction): " << (frame_height - (frame_height * corner_fraction)));
		if ((x > (frame_width / corner_fraction)) && ((y > (frame_height - (frame_height / corner_fraction)))))
		{
			ROS_INFO_STREAM("SUCCESS!");
		}
		cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
	}
}

void print_current_values()
{
	ROS_INFO_STREAM("VALUES: \nint exposure = " << exposure << ";\n"
			<< "int hue = " << hue  << ";\n"
			<< "int gain = " << gain << ";\n"
			<< "int saturation = " << saturation << ";\n"
			<< "int low_H = " << low_H << ";\n"
			<< "int high_H = " << high_H << ";\n"
			<< "int low_S = " << low_S << ";\n"
			<< "int high_S = " << high_S << ";\n"
			<< "int low_V = " << low_V << ";\n"
			<< "int high_V = " << high_V << ";\n"
			<< "int frames_to_average = " << frames_to_average << ";\n\nAlso: \n"
			<< "image_show_magnification: " << image_show_magnification << "\n"
			<< "pause_button: " << pause_button);
	setTrackbarPos("Print Parameters", threshold_window_name, 0);
	print_flag = false;
}

deque<Mat> ring;

Mat get_threshold_image(Mat frame, bool show = true)
{

	Mat frame_HSV, frame_threshold;
blur(frame, frame, Size(3, 3));

	// Convert from BGR to HSV colorspace
	cvtColor(frame, frame_HSV, COLOR_BGR2HSV);
	// Detect the object based on HSV Range Values
	inRange(frame_HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), frame_threshold);

	// Show the frames
	Mat frame_scaled;
	resize(frame, frame_scaled, cv::Size(), image_show_magnification, image_show_magnification);
	if (show)
		imshow(camera_feed_window_name, frame_scaled);

	Mat converted_threshold;
	frame_threshold.convertTo(converted_threshold, CV_32FC3);
	Mat avgImg(converted_threshold.size(), converted_threshold.type(), Scalar());

	if (clear_deque)
	{
		ring.clear();
		clear_deque = false;
	}
	ring.push_back(converted_threshold);
	if (ring.size() > frames_to_average)
	{
		ring.pop_front();

		for (int i = 0; i < frames_to_average; i++)
			accumulate(ring[i], avgImg);
	}
	return avgImg;
}

bool compareContourAreas ( vector<Point> contour1, vector<Point> contour2 ) {
    double i = fabs( contourArea(Mat(contour1)) );
    double j = fabs( contourArea((contour2)) );
    return ( i < j );
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "opencv_laser_node");
	ros::NodeHandle nh("~");
	ros::Rate loop_rate(200);
	nh.param("camera_ID", camera_video_number, 0);
	ROS_INFO_STREAM("camera_ID: " << camera_video_number);

	namedWindow(camera_feed_window_name, CV_WINDOW_AUTOSIZE);
        namedWindow("Contours", WINDOW_AUTOSIZE);

	createTrackbars();
	cv::VideoCapture cap;
	cap.set(CV_CAP_PROP_FRAME_WIDTH, frame_width);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, frame_height);
	cap.set(CV_CAP_PROP_FPS, 187);

	if (!cap.open(camera_video_number))
	{
		cout << "Webcam not connected.\n"
			 << "Please verify\n";
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
	Mat frame;
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

		if (ch == 'p')
			pause_processing = !pause_processing;

		if (frame.empty())
			break;

		if (pause_button == 0)
			cap >> frame;

		if (print_button == 1)
			print_current_values();

		if (clicked)
		{
			//cap >> frame;
			Mat HSV;
			ROS_INFO_STREAM("x: " << mX << ". y: " << mY);
			Mat RGB = frame(Rect(mX, mY, 1, 1));
			cvtColor(RGB, HSV, CV_BGR2HSV);
			Vec3b hsv = HSV.at<Vec3b>(0, 0);
			int H = hsv.val[0]; //hue
			int S = hsv.val[1]; //saturation
			int V = hsv.val[2]; //value
			ROS_INFO_STREAM("Hue: " << H << " Sat: " << S << " Value: " << V);
			clicked = false;
		}

		Mat thresholded_image;
		thresholded_image = get_threshold_image(frame, true);

		Mat thresholded_image_large;
		resize(thresholded_image, thresholded_image_large, cv::Size(), image_show_magnification, image_show_magnification);
		// putText(avgImg_large, "Click here", cvPoint(60,60), FONT_HERSHEY_COMPLEX_SMALL, 2.8, cvScalar(1,1,1), 1, CV_AA);
		if (clicked_2 == true)
		{
			save_picture(thresholded_image_large, "/home/user/threshold.jpg");
			clicked_2 = false;
		}

		imshow(threshold_window_name, thresholded_image_large);
		int count = cv::countNonZero(thresholded_image_large);
dilate(thresholded_image_large, thresholded_image_large, Mat(), Point(-1, -1), 2, 1, 1);


if (!pause_processing){

    Mat gray, hough_in;

    thresholded_image_large.convertTo(gray,CV_8UC3);
    thresholded_image_large.convertTo(hough_in,CV_8U);
    Mat img(thresholded_image_large.size(), CV_8U, Scalar(200));
ROS_INFO_STREAM("thresholded_image_large.type(): " << thresholded_image_large.type());
    //cvtColor(thresholded_image_large, hough_in, CV_BGR2GRAY);

    Mat canny_output;
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    vector<Vec3f> circles;
    namedWindow( "hough_in", 1 );
    imshow( "hough_in", hough_in );
 // HoughCircles(InputArray image, OutputArray circles, int method, double dp, double minDist, double param1=100, double param2=100, int minRadius=0, int maxRadius=0 )
    HoughCircles(hough_in, circles, CV_HOUGH_GRADIENT, 1, 6, 30, 6, 0, 6);
ROS_INFO_STREAM("img.type(): " << img.type());
ROS_INFO_STREAM("hough_in.type(): " << hough_in.type());
ROS_INFO_STREAM("gray.type(): " << gray.type());
ROS_INFO_STREAM("thresholded_image_large.type(): " << thresholded_image_large.type());
ROS_INFO_STREAM("circles.size(): " << circles.size());

    for( size_t i = 0; i < circles.size(); i++ )
    {
         Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
         int radius = cvRound(circles[i][2]);
         // draw the circle center
         circle( img, center, 3, Scalar(0,255,0), -1, 8, 0 );
         // draw the circle outline
         circle( img, center, radius, Scalar(0,0,255), 3, 8, 0 );
    }
if (circles.size() > 0){
    namedWindow( "circles", 1 );
    imshow( "circles", img );
}
    // detect edges using canny
    Canny(gray, canny_output, 50, 150, 3);

    // find contours
    findContours(canny_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
    for(Vec4i example : hierarchy)
    {
        ROS_INFO_STREAM("hierarchy: " << example);
    }


	if ( !contours.empty() && !hierarchy.empty() ) {

	    // loop through the contours/hierarchy
	    for ( int i=0; i<contours.size(); i++ ) {

		// look for hierarchy[i][3]!=-1, ie hole boundaries
		if ( hierarchy[i][3] == -1 ) {
			 contours.erase(contours.begin()+i);

//			contours.erase(contours[i]);
		}
	    }
	}
//    cv::waitKey();

Mat morphy_contours;
Mat structuringElement = getStructuringElement(MORPH_ELLIPSE, Size(10, 10));
morphologyEx( canny_output, morphy_contours, MORPH_CLOSE, structuringElement );
        namedWindow( "morphy_contours", 1 );
    imshow( "morphy_contours", morphy_contours );

    // sort contours
    if (contours.size() > 0){
        sort(contours.begin(), contours.end(), compareContourAreas);

        // grab contours
        vector<Point> biggestContour = contours[contours.size()-1];
        vector<Point> smallestContour = contours[0];
        ROS_INFO_STREAM("biggestContour.area: " << contourArea(biggestContour, false));
        ROS_INFO_STREAM("smallestContour.area: " << contourArea(smallestContour, false));
    }

    //for( int n = 0; n< contours.size(); n++ )
   // {
      //   double Area = contourArea( contours[i],false);  
    //       if(condition)
  //         contours.erase(contours[n]);
//    }

    
    vector<vector<Point>> area_filtered_contours;
    for (int i = 0; i < contours.size(); i++){
        if ((contourArea(contours[i])) > min_area_threshold){
            area_filtered_contours.push_back(contours[i]);
        }
    }

    // get the moments
    vector<Moments> mu(area_filtered_contours.size());
    for (int i = 0; i < area_filtered_contours.size(); i++)
    {
        mu[i] = moments(area_filtered_contours[i], false);
    }

    // get the centroid of figures.
    vector<Point2f> mc(area_filtered_contours.size());
    for (int i = 0; i < area_filtered_contours.size(); i++)
    {
        mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
    }

    // draw contours
    Mat drawing(canny_output.size(), CV_8UC3, Scalar(255, 255, 255));
    for (int i = 0; i < area_filtered_contours.size(); i++)
    {
        Scalar color = Scalar(167, 151, 0); // B G R values
        drawContours(drawing, area_filtered_contours, i, color, 2, 8, hierarchy, 0, Point());
        circle(drawing, mc[i], 4, color, -1, 8, 0);
    }

    vector<Point> joined;
    for (size_t i=0; i<area_filtered_contours.size(); i++) {
          joined.insert(joined.end(), area_filtered_contours[i].begin(), area_filtered_contours[i].end());
    }
    // boundingRect(joined);

    Mat rotated_points(canny_output.size(), CV_8UC3, Scalar(255, 255, 255));
    RotatedRect largest_rectangle;
    if (!joined.empty()){
        largest_rectangle = RotatedRect(minAreaRect(joined));
        ROS_INFO_STREAM("largest_rectangle.angle: " << largest_rectangle.angle);
        Point2f rect_points[4]; largest_rectangle.points( rect_points );
        //rectangle( drawing, rect_points[0], largest_rectangle.br(), color, 2, 8, 0 );
        for( int j = 0; j < 4; j++ ){
            Scalar color = Scalar(50, 151, 200); // B G R values
            line( drawing, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
         }
    }

        ROS_INFO_STREAM("contours.size(): " << contours.size());
        ROS_INFO_STREAM("area_filtered_contours.size(): " << area_filtered_contours.size());

//    if (rotated_points.size() == 14){
//    Mat rot_mat = getRotationMatrix2D( largest_rectangle.center, largest_rectangle.angle, 1.0 );
//}
    Mat rot_mat = getRotationMatrix2D( largest_rectangle.center, largest_rectangle.angle, 1.0 );
    warpAffine(drawing, rotated_points, rot_mat, canny_output.size());

    // show the resultant image
    namedWindow("Rotation_Corrected", WINDOW_AUTOSIZE);
    imshow("Rotation_Corrected", rotated_points);



    // show the resultant image
    namedWindow("Contours", WINDOW_AUTOSIZE);
    imshow("Contours", drawing);


		ROS_INFO_STREAM("White Pixels: " << count);
		if ((count < audio_threshold) && audio_on)
		{
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
