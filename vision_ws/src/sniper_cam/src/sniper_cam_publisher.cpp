#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>


int main(int argc, char** argv)
{


    ROS_INFO_STREAM("Sniper cam publisher");
    ros::init(argc, argv, "sniper_cam_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("sniper_cam/image", 1);
    cv::VideoCapture cap(0);


    // Check if viideo device can be opeed with the given index
    if(!(cap.isOpened())) return 1;

    cv::Mat frame;
    sensor_msgs::ImagePtr msg;

    ros::Rate loop_rate(15);

    while(nh.ok()) {
        cap >> frame;

        // Check if grabbed frame is actuall full with some content

        if(!frame.empty()) {
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            pub.publish(msg);
            cv::waitKey(1);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }


}





//#include "sniper_cam_publisher.h"


//using namespace cv;
//using namespace std;


//namespace sniper_cam
//{

//Sniper_cam::Sniper_cam():
//    _it(image_transport::ImageTransport(nh_))

//{
//    firstTime = true;
//    cvi;
//    image_pub = _it.advertise("sniper_cam", 1);



//}

//Sniper_cam::~Sniper_cam()
//{

//}



//// takes a VideoCapture object ane while doing some
//// simple checks. Then outputs the frame Matrix
//Mat Sniper_cam::create_frame(VideoCapture cap)
//{

//    if (!cap.isOpened())  // if not success, exit program
//    {
//        cout << "Cannot open the video cam" << endl;
//    }

//    if (firstTime)
//    {

//        double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH);
//        double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
//        firstTime = false;
//        cout << "Initial frame size Frame size : " << dWidth << " x " << dHeight << endl;
//    }


//    Mat frame;
//    return frame;
//}

//// changes the frame size of the image
//Mat Sniper_cam::change_frame_size(Mat frame, double width, double height)
//{


//}


//// converts an opencv message to a ros message
//sensor_msgs::Image Sniper_cam::cv_to_rosmsg(Mat frame){

//    ros::Time time = ros::Time::now();

//    cvi = frame;


//    sensor_msgs::Image im;
//    cvi.toImageMsg(im);

//    return im;

//}

//void Sniper_cam::publish_image(sensor_msgs::Image im){


//    image_pub.publish(im);
//}

//}

//int main(int argc, char **argv)
//{

//    VideoCapture cap(1);
//    ros::init(argc, argv, "rover_hub_node");
//    ros::NodeHandle nh_private("~");
//    int rate;
//    nh_private.param<int>("rate", rate, 100);


//    sniper_cam::Sniper_cam sc;



//    ros::Rate loop_rate(rate);
//    while(ros::ok())
//    {
//        Mat frame = sc.create_frame(cap);

//        bool bSuccess = cap.read(frame);
//        if (!bSuccess)
//        {
//            cout << "Cannot read a frame from the video stream" << endl;
//            break;
//        }
//        imshow("Video", frame);
//        if (waitKey(30) == 27)
//        {
//            cout << "esc key is pressed by user" << endl;
//            break;
//        }

//        sensor_msgs::Image image = sc.cv_to_rosmsg(frame);
//        sc.publish_image(image);

//        ros::spinOnce();
////        ro.publish_image();

//        loop_rate.sleep();
//        //ROS_WARN_STREAM("Spinning");
//    }

//////    ros::spin();
//    return 0;
//}



