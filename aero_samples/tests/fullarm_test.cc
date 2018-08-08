#include <aero_std/AeroMoveitInterface.hh>

#define PJOINT(jname) {                                                 \
    ROS_DEBUG("a "#jname"_joint: %f", joint_angles[aero::joint::jname]); \
  }

#define ALLPJOINT() {                     \
    PJOINT(ankle);                        \
    PJOINT(knee);                         \
    PJOINT(r_shoulder_p);                 \
    PJOINT(r_shoulder_y);                 \
    PJOINT(r_elbow);                      \
    PJOINT(r_wrist_p);                    \
    PJOINT(r_wrist_r);                    \
    PJOINT(r_wrist_y);                    \
  }

#define ALLSJOINT() {                                           \
    robot_interface::joint_angle_map map;                       \
    robot->getRobotStateVariables(map);                         \
    for(auto it = map.begin(); it != map.end(); it++) {         \
      ROS_DEBUG("s %s: %f", it->first.c_str(), it->second);     \
    }                                                           \
  }

int main(int argc, char **argv)
{
  // init ros
  ros::init(argc, argv, "fullarm_test_sample_node");
  ros::NodeHandle nh;

  // init robot interface
  aero::interface::AeroMoveitInterface::Ptr robot(new aero::interface::AeroMoveitInterface(nh));
  ROS_INFO("--- RESET MANIP POSE ---");
  robot->setPoseVariables(aero::pose::reset_manip);
  robot->sendModelAngles(3000,aero::ikrange::arm_lifter);
  robot->waitInterpolation();
  sleep(1);

  robot->sendLifter(0.0, 0.0);
  robot->waitInterpolation();

  // set real robot's joint angles to the robot model in interface 
  robot->setRobotStateToCurrentState();

  aero::joint_angle_map joint_angles;

  // test elbow and shoulder joints
  double r_elbow_to = 2.0;

  double r_shoulder_p_to = 0.75;

  double r_shoulder_y_to = -0.7;

  robot->getRobotStateVariables(joint_angles);
  ALLPJOINT();
  ALLSJOINT();

  ROS_INFO("--- ELBOW JOINTS ---");
  joint_angles[aero::joint::r_elbow] = r_elbow_to;
  robot->setRobotStateVariables(joint_angles);
  ALLPJOINT();
  ALLSJOINT();
  robot->sendModelAngles(1400,aero::ikrange::arm_lifter);
  robot->waitInterpolation();

  ROS_INFO("--- SHOULDER PITCH JOINT ---");
  joint_angles[aero::joint::r_shoulder_p] = r_shoulder_p_to;
  robot->setRobotStateVariables(joint_angles);
  ALLPJOINT();
  ALLSJOINT();
  robot->sendModelAngles(1400,aero::ikrange::arm_lifter);
  robot->waitInterpolation();

  ROS_INFO("--- SHOULDER YAW JOINTS ---");
  joint_angles[aero::joint::r_shoulder_y] = r_shoulder_y_to;
  robot->setRobotStateVariables(joint_angles);
  ALLPJOINT();
  ALLSJOINT();
  robot->sendModelAngles(1400,aero::ikrange::arm_lifter);
  robot->waitInterpolation();

  ROS_INFO("--- RESET MANIP POSE ---");
  robot->setPoseVariables(aero::pose::reset_manip);
  ALLPJOINT();
  ALLSJOINT();
  robot->sendModelAngles(2000,aero::ikrange::arm_lifter);
  robot->waitInterpolation();
  sleep(1);

  // test wrist and neck joints and hand
  double r_wrist_r_to =  0.5;
  double r_wrist_y_to =  0.5;

  robot->getRobotStateVariables(joint_angles);

  ROS_INFO("--- WRIST ROLL JOINTS ---");
  joint_angles[aero::joint::r_wrist_r] = r_wrist_r_to;
  ALLPJOINT();
  ALLSJOINT();
  robot->sendModelAngles(2000,aero::ikrange::arm_lifter);
  robot->waitInterpolation();

  ROS_INFO("--- WRIST YAW JOINTS ---");
  joint_angles[aero::joint::r_wrist_y] = r_wrist_y_to;
  robot->setRobotStateVariables(joint_angles);
  ALLPJOINT();
  ALLSJOINT();
  robot->sendModelAngles(2000,aero::ikrange::arm_lifter);
  robot->waitInterpolation();

  ROS_INFO("--- testing right hand ---");
  robot->sendGrasp(aero::arm::rarm);
  robot->openHand(aero::arm::rarm);
  robot->sendHand(aero::arm::rarm, 0.0);

  ROS_INFO("--- reseting robot pose ---");
  robot->setPoseVariables(aero::pose::reset_manip);
  robot->sendModelAngles(1000,aero::ikrange::arm_lifter);
  robot->waitInterpolation();
  sleep(1);


#if 1
  // test lifter
  ROS_INFO("lifter x:0.0 z:-0.4");
  robot->sendLifter(0.0, -0.4, 3000);
  robot->waitInterpolation();
  usleep(500 * 1000);

  ROS_INFO("lifter x:0.0 z:-0.2");
  robot->sendLifter(0.0, -0.2, 3000);
  robot->waitInterpolation();
  usleep(500 * 1000);

  ROS_INFO("lifter x:-0.1 z:-0.2");
  robot->sendLifter(-0.1, -0.2, 3000);
  robot->waitInterpolation();
  usleep(500 * 1000);

  ROS_INFO("lifter x:0.1 z:-0.2");
  robot->sendLifter(0.1, -0.2, 3000);
  robot->waitInterpolation();
  usleep(500 * 1000);

  ROS_INFO("lifter x:0.0 z:0.0");
  robot->sendLifter(0.0, 0.0, 3000);
  robot->waitInterpolation();
  usleep(500 * 1000);
#endif

  ROS_INFO("demo node finished");
  ros::shutdown();
  return 0;
}
