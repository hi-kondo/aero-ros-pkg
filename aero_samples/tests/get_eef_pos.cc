#include "aero_std/AeroMoveitInterface.hh"
#include <fstream>

int main(int argc, char **argv){
    ros::init(argc, argv, "get_eef_pos");
    ros::NodeHandle nh;
    aero::interface::AeroMoveitInterface::Ptr robot(new aero::interface::AeroMoveitInterface(nh));
    ros::Publisher pub_ = nh.advertise<geometry_msgs::Vector3>("/eef_pos", 10);
    geometry_msgs::Vector3 msg;

    ros::Rate loop_rate(5);

    tf::TransformListener listener;  

    

    ROS_INFO("get_eef_pos inited");

 // create target to look at
    Eigen::Vector3d pos;  
    while (nh.ok()){
       tf::StampedTransform transform;
       try{
         listener.lookupTransform("/waist_link", "/r_eef_grasp_link",  
                                  ros::Time(0), transform);
       }
       catch (tf::TransformException ex){
         ROS_ERROR("%s",ex.what());
         ros::Duration(1.0).sleep();
       }

       msg.x = (double)transform.getOrigin().x();
       msg.y = (double)transform.getOrigin().y();
       msg.z = (double)transform.getOrigin().z();
       //ROS_INFO("x:%f, y:%f, z:%f", msg.x, msg.y, msg.z);
       
    
       pub_.publish(msg);
       


    }

    ros::shutdown();
    return 0;

}
