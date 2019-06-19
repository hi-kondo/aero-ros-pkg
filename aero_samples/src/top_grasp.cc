#include <aero_std/AeroMoveitInterface.hh>
#include <aero_std/TopGrasp-inl.hh>

/// @file top_grasp.cc
/// @brief how to use TopGrasp. This function wrapps sequence of grasping object from top of it.

int main(int argc, char **argv)
{
  // init ros
  ros::init(argc, argv, "top_grasp_sample_node");
  ros::NodeHandle nh;

  // init robot interface
  aero::interface::AeroMoveitInterface::Ptr robot(new aero::interface::AeroMoveitInterface(nh));
  ROS_INFO("reseting robot pose");
  robot->setPoseVariables(aero::pose::reset_manip);
  robot->setLifter(0, 0);
  robot->sendModelAngles(3000);
  robot->waitInterpolation();

  // grasp object from top
  Eigen::Vector3d obj;// target object's position
  obj << 0.7, 0.2, 1.0;

  aero::TopGrasp top;// grasping information
  top.arm = aero::arm::either;
  top.object_position = obj;
  top.height = 0.2;

  auto req = aero::Grasp<aero::TopGrasp>(top);
  req.mid_ik_range = aero::ikrange::wholebody;
  req.end_ik_range = aero::ikrange::wholebody;

  if (robot->sendPickIK(req)) {
    robot->waitInterpolation();
    ROS_INFO("success");
    sleep(1);
    robot->sendGrasp(req.arm);
    sleep(3);
    ROS_INFO("reseting robot pose");
    robot->setPoseVariables(aero::pose::reset_manip);
    robot->setLifter(0, 0);
    robot->sendModelAngles(3000);
    robot->waitInterpolation();
  }
  else ROS_INFO("failed");

  ROS_INFO("demo node finished");
  ros::shutdown();
  return 0;
}
