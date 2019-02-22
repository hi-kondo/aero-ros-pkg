#include <aero_std/AeroMoveitInterface.hh>

/// @file fk.cc
/// @brief how to set direcly model's joint angle and send it to real robot


int main(int argc, char **argv)
{
  // init ros
  ros::init(argc, argv, "fk_sample_node");
  ros::NodeHandle nh;
  
  // init robot interface
  aero::interface::AeroMoveitInterface::Ptr robot(new aero::interface::AeroMoveitInterface(nh));
  ROS_INFO("reseting robot pose");
  robot->setPoseVariables(aero::pose::reset_manip);
  robot->sendModelAngles(1000, aero::ikrange::upperbody);
  robot->waitInterpolation();
  //sleep(3);

  // set real robot's joint angles to the robot model in interface 
  robot->setRobotStateToCurrentState();

  // how to move selected joint
  double r_elbow_to = -0.12;
  aero::joint_angle_map joint_angles;
  robot->getRobotStateVariables(joint_angles);// save angles from robot model
  //ROS_INFO("left elbow moves from %f to %f", joint_angles[aero::joint::r_hand_y], r_elbow_to);
  joint_angles[aero::joint::waist_r] = r_elbow_to;// replace elbow's angle value

  ROS_INFO("moveing left elbow");
  robot->setRobotStateVariables(joint_angles);
  robot->sendModelAngles(1000, aero::ikrange::upperbody);// send to robot
  robot->waitInterpolation();
  //sleep(1);

  r_elbow_to = 0.12;
  robot->getRobotStateVariables(joint_angles);// save angles from robot model
  //ROS_INFO("left elbow moves from %f to %f", joint_angles[aero::joint::r_hand_y], r_elbow_to);
  joint_angles[aero::joint::waist_r] = r_elbow_to;// replace elbow's angle value

  ROS_INFO("moveing left elbow");
  robot->setRobotStateVariables(joint_angles);
  robot->sendModelAngles(2000, aero::ikrange::upperbody);// send to robot
  robot->waitInterpolation();
  //sleep(1);


  ROS_INFO("reseting robot pose");
  robot->setPoseVariables(aero::pose::reset_manip);
  robot->sendModelAngles(1000, aero::ikrange::upperbody);
  robot->waitInterpolation();
  //sleep(3);
  ROS_INFO("demo node finished");
  ros::shutdown();
  return 0;
}
