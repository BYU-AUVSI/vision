
//#include "opencv2/highgui/highgui.hpp"
//#include <iostream>

//using namespace cv;
//using namespace std;

//int main(int argc, char* argv[])
//{
//    VideoCapture cap(0); // open the video camera no. 0

//    if (!cap.isOpened())  // if not success, exit program
//    {
//        cout << "Cannot open the video cam" << endl;
//        return -1;
//    }

//   double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
//   double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video

//    cout << "Frame size : " << dWidth << " x " << dHeight << endl;

//    namedWindow("MyVideo",CV_WINDOW_AUTOSIZE); //create a window called "MyVideo"

//    while (1)
//    {
//        Mat frame;

//        bool bSuccess = cap.read(frame); // read a new frame from video

//         if (!bSuccess) //if not success, break loop
//        {
//             cout << "Cannot read a frame from video stream" << endl;
//             break;
//        }

//        imshow("MyVideo", frame); //show the frame in "MyVideo" window

//        if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
//       {
//            cout << "esc key is pressed by user" << endl;
//            break;
//       }
//    }
//    return 0;

//}
#ifndef SNIPER_CAM_PUBLISHER_H
#define SNIPER_CAM_PUBLISHER_H


#include <ros/ros.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CompressedImage.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <stdio.h>

using namespace cv;
using namespace std;
namespace sniper_cam
{

	class Sniper_cam
{

public:

	Sniper_cam();
	~Sniper_cam();

    // public class functions
    Mat create_frame(VideoCapture cap);
    Mat change_frame_size(Mat frame, double width, double height);
    Mat convert_to_rosmsg(Mat frame);

private:
    // node hands
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // image transport
    image_transport::ImageTransport _it;

    // image publisher
    image_transport::Publisher _img_pub;


    // class functions

    // private class variables
    bool firstTime;







};
}
#endif















