#ifndef SNIPER_CAM_PUBLISHER_H
#define SNIPER_CAM_PUBLISHER_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>


namespace sniper_cam
{
class Sniper_cam

{
public:

    Sniper_cam();
    ~Sniper_cam();
    void publish_image();

private:

    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Publisher pub;


    cv::Mat frame;
    sensor_msgs::ImagePtr msg;


};
}










#endif //SNIPER_CAM_PUBLISHER_H
