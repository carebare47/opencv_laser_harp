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

int main(int argc, char **argv)
{

    ros::init(argc, argv, "opencv_laser_node");
    ros::NodeHandle nh("~");
    ros::Rate loop_rate(200);

    Mat I = imread("/home/user/catkin_ws/src/opencv_laser_harp/images/threshold.jpg", 0);
    if (I.empty())
    {
        std::cout << "!!! Failed imread(): image not found" << std::endl;
        return -1;
        // don't let the execution continue, else imshow() will crash.
    }

    Mat image;
    image = imread("/home/user/catkin_ws/src/opencv_laser_harp/images/threshold.jpg"); // Read the file

    //    image = imread(argv[1], CV_LOAD_IMAGE_COLOR); // Read the file

    if (!image.data) // Check for invalid input
    {
        cout << "Could not open or find the image" << std::endl;
        return -1;
    }

    namedWindow("Display window", WINDOW_AUTOSIZE); // Create a window for display.
    imshow("Display window", image);                // Show our image inside it.

    waitKey(0); // Wait for a keystroke in the window

    namedWindow("Display window", CV_WINDOW_AUTOSIZE); // Create a window for display.
    imshow("Display window", I);
    waitKey(0);

    Mat gray;
    I.copyTo(gray);

    Mat canny_output;
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    // detect edges using canny
    Canny(gray, canny_output, 50, 150, 3);

    // find contours
    findContours(canny_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));

    // get the moments
    vector<Moments> mu(contours.size());
    for (int i = 0; i < contours.size(); i++)
    {
        mu[i] = moments(contours[i], false);
    }

    // get the centroid of figures.
    vector<Point2f> mc(contours.size());
    for (int i = 0; i < contours.size(); i++)
    {
        mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
    }

    // draw contours
    Mat drawing(canny_output.size(), CV_8UC3, Scalar(255, 255, 255));
    for (int i = 0; i < contours.size(); i++)
    {
        Scalar color = Scalar(167, 151, 0); // B G R values
        drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, Point());
        circle(drawing, mc[i], 4, color, -1, 8, 0);
    }

    // show the resultant image
    namedWindow("Contours", WINDOW_AUTOSIZE);
    imshow("Contours", drawing);
    waitKey(0);

    // // C++ : void findContours(InputOutputArray image, OutputArrayOfArrays contours, OutputArray hierarchy, int mode, int method, Point offset = Point())
    // // 		  C++ : void findContours(InputOutputArray image, OutputArrayOfArrays contours, int mode, int method, Point offset = Point())
    // // 					Python : cv2.findContours(image, mode, method[, contours[, hierarchy[, offset]]]) → contours,
    // // 	hierarchy
    // // 	C : int cvFindContours(CvArr *image, CvMemStorage *storage, CvSeq **first_contour, int header_size = sizeof(CvContour), int mode = CV_RETR_LIST, int method = CV_CHAIN_APPROX_SIMPLE, CvPoint offset = cvPoint(0, 0))

    // // }
}