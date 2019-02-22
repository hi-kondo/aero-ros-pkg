#include <aero_std/AeroMoveitInterface.hh>

/// @file minimum.cc
/// @brief minimum code to use interface. This file initialize interface and move robot once.

int main(int argc, char **argv)
{
  // init ros
  ros::init(argc, argv, "minimum_sample_node");
  ros::NodeHandle nh;

  //system("rosnode kill /aero_hand_controller");
  
  // init robot interface
  aero::interface::AeroMoveitInterface::Ptr robot(new aero::interface::AeroMoveitInterface(nh));
  ROS_INFO("reseting robot pose");
  robot->setPoseVariables(aero::pose::reset_manip);
  robot->sendModelAngles(3000, aero::ikrange::arm_lifter);
  sleep(3);

  ROS_INFO("demo node finished");
  ros::shutdown();
  return 0;
}
