#include <aero_std/AeroMoveitInterface.hh>

#define PJOINT(jname) {                                                 \
    ROS_DEBUG("a "#jname"_joint: %f", joint_angles[aero::joint::jname]); \
  }

#define ALLPJOINT() {                     \
    PJOINT(ankle);                        \
    PJOINT(knee);                         \
    PJOINT(l_elbow);                      \
    PJOINT(l_hand_y);                     \
    PJOINT(l_shoulder_p);                 \
    PJOINT(l_shoulder_r);                 \
    PJOINT(l_shoulder_y);                 \
    PJOINT(l_wrist_p);                    \
    PJOINT(l_wrist_r);                    \
    PJOINT(l_wrist_y);                    \
    PJOINT(r_elbow);                      \
    PJOINT(r_hand_y);                     \
    PJOINT(r_shoulder_p);                 \
    PJOINT(r_shoulder_r);                 \
    PJOINT(r_shoulder_y);                 \
    PJOINT(r_wrist_p);                    \
    PJOINT(r_wrist_r);                    \
    PJOINT(r_wrist_y);                    \
    PJOINT(waist_p);                      \
    PJOINT(waist_r);                      \
    PJOINT(waist_y);                      \
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
  ros::init(argc, argv, "set_lifter_node");
  ros::NodeHandle nh;

  // init robot interface
  aero::interface::AeroMoveitInterface::Ptr robot(new aero::interface::AeroMoveitInterface(nh));
  ROS_INFO("--- RESET MANIP POSE ---");
  robot->setPoseVariables(aero::pose::reset_manip);
  robot->sendModelAngles(3000);
  robot->waitInterpolation();
  sleep(1);

  robot->sendLifter(0.0, 0.0);
  robot->waitInterpolation();

  // set real robot's joint angles to the robot model in interface 
  robot->setRobotStateToCurrentState();

  aero::joint_angle_map joint_angles;
  robot->getRobotStateVariables(joint_angles);

#if 1
 
  ROS_INFO("lifter x:0.0 z:-0.2");
  robot->sendLifter(0.0, -0.2, 3000);
  robot->waitInterpolation();
  usleep(500 * 1000);

  ROS_INFO("demo node finished");
  ros::shutdown();
  return 0;
}
