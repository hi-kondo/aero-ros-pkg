#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <iostream>
#include <depth_image_proc/depth_conversions.h>
#include <depth_image_proc/depth_traits.h>

void chatterCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    cv_bridge::CvImageConstPtr Dest = cv_bridge::toCvCopy(msg);
    ROS_INFO("Width: %i", msg->width);
    ROS_INFO("Height: %i", msg->height);
    auto val = Dest->image.at<uint16_t>(msg->width/2,msg->height/2);
    auto distance = depth_image_proc::DepthTraits<uint16_t>::toMeters(val);
    ROS_INFO("Value:%f", distance);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "distance");

    ros::NodeHandle n;

    ros::AsyncSpinner spinner(1);

    spinner.start();

    ros::Subscriber sub = n.subscribe("camera/depth/image_rect_raw", 100, chatterCallback);
    ros::Duration(100).sleep();

    return 0;
}