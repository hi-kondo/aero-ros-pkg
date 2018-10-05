#ifndef AERO_PS_TELEOP_
#define AERO_PS_TELEOP_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>

#include <aero_std/AeroMoveitInterface.hh>
#include <aero_startup/GraspControl.h>
#include "aero_extra_controllers/AeroHandController.hh"

#include <map>
#include <string>

namespace aero {
  
  namespace teleop {

    static const std::string X = "x";
    static const std::string Y = "y";
    static const std::string Z = "z";
    static const std::string ROLL = "roll";
    static const std::string ROLL_PLUS = "rollp";
    static const std::string ROLL_MINUS = "rollm";
    static const std::string PITCH = "pitch";
    static const std::string PITCH_PLUS = "pitchp";
    static const std::string PITCH_MINUS = "pitchm";
    static const std::string YAW = "yaw";
    static const std::string YAW_PLUS = "yawp";
    static const std::string YAW_MINUS = "yawm";
    static const std::string LEFT = "L";
    static const std::string RIGHT = "R";
    static const std::string FLAG = "enable";
    static const std::string RIGHT_FLAG = "enable_right";
    static const std::string LEFT_FLAG = "enable_left";

    static const int BASIC_MODE = 0;
    static const int JOINT_MODE = 1;
    static const int IK_MODE = 2;


    class ps_teleop {
    public:
      typedef std::shared_ptr<ps_teleop> Ptr;
      ps_teleop(ros::NodeHandle &nh_param);
      ~ps_teleop();
      void loop();
      
    protected:
      virtual void joyCallback(const sensor_msgs::Joy::ConstPtr &joy_msg);
      void basicMode(const sensor_msgs::Joy::ConstPtr &joy_msg);
      void jointMode(const sensor_msgs::Joy::ConstPtr &joy_msg);
      void ikMode(const sensor_msgs::Joy::ConstPtr &joy_msg);
      
      int mode;
      bool during_reset;
      ros::Time reset_start_time;
      bool in_grasp_mode;
      // common
      int reset_pose_button;
      int switch_basic_mode;
      int switch_mode;
      // twist
      int enable_button;
      int enable_turbo_button;
      std::map<std::string, int> axis_linear_map;
      std::map<std::string, double> scale_linear_map;
      std::map<std::string, double> scale_linear_turbo_map;
      std::map<std::string, int> axis_angular_map;
      std::map<std::string, double> scale_angular_map;
      std::map<std::string, double> scale_angular_turbo_map;
      bool sent_disable_msg;
      // other basics
      int external_command_button;
      bool enable_lifter;
      int enable_grasp_angle_button;
      int enable_lifter_button;
      double lifter_dx, lifter_dz;
      std::map<std::string, int> lifter_axis_map;
      std::map<std::string, double> scale_lifter_map;
      std::map<std::string, int> grasp_angle_axis_map;
      std::map<std::string, double> range_grasp_angle_map;
      std::map<std::string, double> min_grasp_angle_map;
      int grasp_L_button;
      int grasp_R_button;
      // joint mode
      bool enable_right_shoulder;
      double right_shoulder_dr, right_shoulder_dp, right_shoulder_dy;
      std::map<std::string, int> shoulder_map;
      std::map<std::string, double> scale_shoulder_map;
      bool enable_right_elbow;
      double right_elbow_dp, right_elbow_dy;
      std::map<std::string, int> elbow_map;
      std::map<std::string, double> scale_elbow_map;
      bool enable_right_wrist;
      double right_wrist_dp, right_wrist_dy;
      std::map<std::string, int> wrist_map;
      std::map<std::string, double> scale_wrist_map;
      // ik mode
      bool enable_ik;
      std::map<std::string, int> ik_map;
      std::map<std::string, double> scale_ik_map;
      aero::Transform eef;
      aero::Translation eef_pos;
      aero::Translation pos, pos_dp;
      //aero::Quaternion init_rot;
      //aero::Quaternion rot_top;
      aero::Quaternion rot_left;
      
      ros::Publisher cmd_vel_pub;
      ros::Publisher ik_bool_pub;
      ros::Publisher eef_pos_pub;

      interface::AeroMoveitInterface::Ptr robot;
      std::map<aero::joint, double> min_bounds, max_bounds;
      ros::Subscriber joy_sub;
    };
  }
}
#endif