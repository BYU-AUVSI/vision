#include "sniper_cam_publisher.h"


using namespace cv;
using namespace std;


namespace sniper_cam
{

Sniper_cam::Sniper_cam():
    _it(image_transport::ImageTransport(nh_))

{
    firstTime = true;



}

Sniper_cam::~Sniper_cam()
{

}



// takes a VideoCapture object ane while doing some
// simple checks. Then outputs the frame Matrix
Mat Sniper_cam::create_frame(VideoCapture cap)
{

    if (!cap.isOpened())  // if not success, exit program
    {
        cout << "Cannot open the video cam" << endl;
    }

    if (firstTime)
    {

        double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH);
        double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
        firstTime = false;
        cout << "Initial frame size Frame size : " << dWidth << " x " << dHeight << endl;
    }


    Mat frame;
    return frame;
}

// changes the frame size of the image
Mat Sniper_cam::change_frame_size(Mat frame, double width, double height)
{


}


// converts an opencv message to a ros message
Mat cv_to_rosmsg(Mat frame){



}

int main(int argc, char **argv)
{

    VideoCapture cap(1);
    ros::init(argc, argv, "rover_hub_node");
    ros::NodeHandle nh_private("~");
    int rate;
    nh_private.param<int>("rate", rate, 100);


    sniper_cam::Sniper_cam sc;



    ros::Rate loop_rate(rate);
    while(ros::ok())
    {
        Mat frame = sc.create_frame(cap);

        bool bSuccess = cap.read(frame);
        if (!bSuccess)
        {
            cout << "Cannot read a frame from the video stream" << endl;
            break;
        }
        imshow("Video", frame);
        if (waitKey(30) == 27)
        {
            cout << "esc key is pressed by user" << endl;
            break;
        }


        ros::spinOnce();
//        ro.publish_image();

        loop_rate.sleep();
        //ROS_WARN_STREAM("Spinning");
    }

////    ros::spin();
    return 0;
}



