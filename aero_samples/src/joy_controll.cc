#include <ros/ros.h>
#include <fstream>
//#include <string>
//to get tf--
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
//---
//to get ps3 data---
#include <sensor_msgs/Joy.h>
//read yaml---
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <ros/package.h>
#include <aero_std/AeroMoveitInterface.hh>

//////////////////////////////
class Joy_Controll{
public:
  Joy_Controll(ros::NodeHandle _nh);
  void getPS3(const sensor_msgs::JoyPtr& _ps3);

private:
//to get tf--
  ros::Subscriber wheel_pos_sub_;
  tf::TransformListener listener_;
  aero::joint_angle_map joint_angles;

//to get ps3 data--
  ros::Subscriber ps3_sub_;
  int32_t ps3_start_;
  int32_t ps3_pre_start_;
  int32_t ps3_select_;
  int32_t ps3_triangle_;
  int32_t ps3_square_;
  int32_t ps3_circle_;
  int32_t ps3_cross_;

  int32_t ps3_left_;
  int32_t ps3_right_;
//----
  double lifter_position1;
  double lifter_position2;
  double pre_lifter_position1;

  double r_shoulder_p_to_deg;
  double r_shoulder_p_to_rad;

  double r_shoulder_y_to_deg;
  double r_shoulder_y_to_rad;

  ros::NodeHandle nh_;
  uint16_t counter_;

  std::string pkg_path_;
  aero::interface::AeroMoveitInterface::Ptr robot_;

};

Joy_Controll::Joy_Controll(const ros::NodeHandle _nh)
  : ps3_start_(0)
  , ps3_pre_start_(0)
  , ps3_select_(0)
  , ps3_circle_(0)
  , ps3_cross_(0)
  , ps3_square_(0)
  , ps3_triangle_(0)
  , ps3_left_(0)
  , ps3_right_(0)
  , counter_(0)
  , lifter_position1(0)
  , lifter_position2(0)
  , pre_lifter_position1(0)
  , nh_(_nh)
{ 	
//to get ps3 data---
  ps3_sub_ = nh_.subscribe("/joy",1, &Joy_Controll::getPS3,this);
//--
  pkg_path_ = ros::package::getPath("scenario");

  robot_.reset(new aero::interface::AeroMoveitInterface(nh_));

}

/*--------------------------------------------------------
***********         ps3_config     ************************
button[0]:select
button[1]:left_joy_stick
button[2]:right_joy_stick
button[3]:start
button[4]:up_key
button[5]:right_key
button[6]:down_key
button[7]:left_key
button[8]:L2
button[9]:R2
button[10]:L1
button[11]:R1
button[12]:triangle
button[13]:circle
button[14]:cross
button[15]:square
button[16]:PS
----------------------------------------------------------*/

void Joy_Controll::getPS3(const sensor_msgs::JoyPtr& _ps3){
  ps3_start_ = _ps3->buttons[3];
  ps3_select_ = _ps3->buttons[0];
  ps3_triangle_ = _ps3->buttons[12];
  ps3_circle_ = _ps3->buttons[13];
  ps3_cross_ = _ps3->buttons[14];
  ps3_square_ = _ps3->buttons[15];

  ps3_right_ = _ps3->buttons[11];
  ps3_left_ = _ps3->buttons[10];


  //hand_grasp
  if(ps3_right_==1){
    if(ps3_circle_==1){
      ROS_INFO("HandGrasp");
      robot_->sendGrasp(aero::arm::rarm, 100);// arm and grasping power(max 100[%])
      ROS_INFO("HandGrasp End");
      sleep(5);

    }else if(ps3_cross_==1){
      ROS_INFO("HandOpen");
      robot_->openHand(aero::arm::rarm);
      ROS_INFO("HandOpen End");
      sleep(3);
    }
  }

 //shoulder
  if(ps3_right_ == 1){
    int stickval = static_cast <int>(_ps3->axes[3]*100);
    ROS_INFO("R1_pushed");
    ROS_INFO("stickval:%d", stickval);
    if (r_shoulder_p_to_deg <= 80.0 && r_shoulder_p_to_deg >= 0.0){

        if(stickval != 0) {
          ROS_INFO("send");
          robot_->setRobotStateVariables(joint_angles);
          robot_->sendModelAngles(200,aero::ikrange::arm);// send to robot
        }
        if(stickval < 0){
          ROS_INFO("down");
          lifter_position1 += 0.005;
          lifter_position2 -= 0.005;
        }
        else if(stickval > 0){
          lifter_position1 -= 0.005;
          lifter_position2 += 0.005;       
          ROS_INFO("up");
 
        }
    }

    if(lifter_position2 > 0.0) {
    lifter_position1 = 0.0;
    lifter_position2 = 0.0;
    ROS_INFO("lifter_position > 0");

    }
    else if(lifter_position2 < -0.4){
      //lifter_position1 = -400;
      lifter_position2 = -0.4;
      ROS_INFO("lifter_position < -400");
    }
    ROS_INFO("lifter_pos:%f",lifter_position2);
   }

  //lifter
  if(ps3_left_ == 1){
    int stickval = static_cast <int>(_ps3->axes[3]*100);
    ROS_INFO("left_pushed");
    ROS_INFO("stickval:%d", stickval);
    if (lifter_position2 <= 0.0 && lifter_position2 >= -0.4){

        if(stickval != 0) {
          ROS_INFO("sendLifter");
          robot_->sendLifter(0.0,lifter_position2,10);
          robot_->waitInterpolation();
          pre_lifter_position1 = lifter_position1;
        }
        if(stickval < 0){
          ROS_INFO("down");
          lifter_position1 += 0.005;
          lifter_position2 -= 0.005;
        }
        else if(stickval > 0){
          lifter_position1 -= 0.005;
          lifter_position2 += 0.005;       
          ROS_INFO("up");
 
        }
    }

    if(lifter_position2 > 0.0) {
    lifter_position1 = 0.0;
    lifter_position2 = 0.0;
    ROS_INFO("lifter_position > 0");

    }
    else if(lifter_position2 < -0.4){
      //lifter_position1 = -400;
      lifter_position2 = -0.4;
      ROS_INFO("lifter_position < -0.4");
    }
    ROS_INFO("lifter_pos:%f",lifter_position2);
   }

}

int main(int argc, char **argv)
{
	ros::init(argc,argv,"joy_controll");
	ros::NodeHandle nh;

  Joy_Controll jc(nh);

	ros::spin();

	return 0;
}
