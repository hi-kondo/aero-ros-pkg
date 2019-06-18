/*
 * This file auto-generated from script. Do not Edit!
 * Original : aero_startup/.templates/aero_hardware_interface/AeroControllers.cc
 * Depend : aero_description/{my_robot}/headers/Constants.hh
*/
#include "AeroControllers.hh"

using namespace aero;
using namespace controller;

//////////////////////////////////////////////////
AeroUpperController::AeroUpperController(const std::string& _port) :
    AeroControllerProto(_port, ID_UPPER)
{
  stroke_vector_.resize(AERO_DOF_UPPER);
  stroke_ref_vector_.resize(AERO_DOF_UPPER);
  stroke_cur_vector_.resize(AERO_DOF_UPPER);
  status_vector_.resize(AERO_DOF_UPPER);

  stroke_joint_indices_.clear();
  stroke_joint_indices_.reserve(AERO_DOF_UPPER);

  // adding code
  stroke_joint_indices_.push_back(
      AJointIndex(1, 0, 0, std::string("can_neck_y")));
  stroke_joint_indices_.push_back(
      AJointIndex(1, 1, 1, std::string("can_neck_right")));
  stroke_joint_indices_.push_back(
      AJointIndex(1, 2, 2, std::string("can_neck_left")));
  stroke_joint_indices_.push_back(
      AJointIndex(1, 3, 3, std::string("can_r_shoulder_p")));
  stroke_joint_indices_.push_back(
      AJointIndex(1, 4, 4, std::string("can_r_shoulder_r")));
  stroke_ref_vector_[4] = 1119;
  stroke_joint_indices_.push_back(
      AJointIndex(1, 5, 5, std::string("can_r_elbow_y")));
  stroke_joint_indices_.push_back(
      AJointIndex(1, 6, 6, std::string("can_r_elbow_p")));
  stroke_joint_indices_.push_back(
      AJointIndex(1, 7, 7, std::string("can_r_wrist_y")));
  stroke_joint_indices_.push_back(
      AJointIndex(1, 8, 8, std::string("can_r_wrist_top")));
  stroke_joint_indices_.push_back(
      AJointIndex(1, 9, 9, std::string("can_r_wrist_bottom")));
  stroke_joint_indices_.push_back(
      AJointIndex(1, 10, 11, std::string("can_r_hand")));
  stroke_ref_vector_[10] = 900;
  stroke_joint_indices_.push_back(
      AJointIndex(1, 11, 18, std::string("can_l_shoulder_p")));
  stroke_joint_indices_.push_back(
      AJointIndex(1, 12, 19, std::string("can_l_shoulder_r")));
  stroke_ref_vector_[12] = 1119;
  stroke_joint_indices_.push_back(
      AJointIndex(1, 13, 20, std::string("can_l_elbow_y")));
  stroke_joint_indices_.push_back(
      AJointIndex(1, 14, 21, std::string("can_l_elbow_p")));
  stroke_joint_indices_.push_back(
      AJointIndex(1, 15, 22, std::string("can_l_wrist_y")));
  stroke_joint_indices_.push_back(
      AJointIndex(1, 16, 23, std::string("can_l_wrist_top")));
  stroke_joint_indices_.push_back(
      AJointIndex(1, 17, 24, std::string("can_l_wrist_bottom")));
  stroke_joint_indices_.push_back(
      AJointIndex(1, 18, 26, std::string("can_l_hand")));
  stroke_ref_vector_[18] = 900;
  stroke_joint_indices_.push_back(
      AJointIndex(1, 19, 10, std::string("can_waist_right")));
  stroke_joint_indices_.push_back(
      AJointIndex(1, 20, 25, std::string("can_waist_left")));
  stroke_joint_indices_.push_back(
      AJointIndex(1, 21, 15, std::string("can_waist_y")));

  angle_joint_indices_["waist_y_joint"] = 0;
  angle_joint_indices_["waist_p_joint"] = 1;
  angle_joint_indices_["waist_r_joint"] = 2;
  angle_joint_indices_["l_shoulder_p_joint"] = 3;
  angle_joint_indices_["l_shoulder_r_joint"] = 4;
  angle_joint_indices_["l_shoulder_y_joint"] = 5;
  angle_joint_indices_["l_elbow_joint"] = 6;
  angle_joint_indices_["l_wrist_y_joint"] = 7;
  angle_joint_indices_["l_wrist_p_joint"] = 8;
  angle_joint_indices_["l_wrist_r_joint"] = 9;
  angle_joint_indices_["l_indexbase_joint"] = 10;
  angle_joint_indices_["l_indexmid_joint"] = 11;
  angle_joint_indices_["l_indexend_joint"] = 12;
  angle_joint_indices_["l_thumb_joint"] = 13;
  angle_joint_indices_["neck_y_joint"] = 14;
  angle_joint_indices_["neck_p_joint"] = 15;
  angle_joint_indices_["neck_r_joint"] = 16;
  angle_joint_indices_["r_shoulder_p_joint"] = 17;
  angle_joint_indices_["r_shoulder_r_joint"] = 18;
  angle_joint_indices_["r_shoulder_y_joint"] = 19;
  angle_joint_indices_["r_elbow_joint"] = 20;
  angle_joint_indices_["r_wrist_y_joint"] = 21;
  angle_joint_indices_["r_wrist_p_joint"] = 22;
  angle_joint_indices_["r_wrist_r_joint"] = 23;
  angle_joint_indices_["r_indexbase_joint"] = 24;
  angle_joint_indices_["r_indexmid_joint"] = 25;
  angle_joint_indices_["r_indexend_joint"] = 26;
  angle_joint_indices_["r_thumb_joint"] = 27;


  get_command(CMD_GET_POS, stroke_cur_vector_);
  stroke_ref_vector_.assign(stroke_cur_vector_.begin(),
                            stroke_cur_vector_.end());
}

//////////////////////////////////////////////////
AeroUpperController::~AeroUpperController()
{
}

//////////////////////////////////////////////////
void AeroUpperController::util_servo_on()
{
  boost::mutex::scoped_lock lock(ctrl_mtx_);

  std::vector<uint8_t> dat;
  dat.resize(8);
  dat[0] = 0xfd;  // header
  dat[1] = 0xdf;  // header
  dat[2] = 0x04;  // data length
  dat[3] = 0x21;  // servo
  dat[4] = 0x1d;  // ID29
  dat[5] = 0x00;  // on
  dat[6] = 0x01;  // on
  dat[7] = 0xbc;  // checksum
  seed_.send_data(dat);
}

//////////////////////////////////////////////////
void AeroUpperController::util_servo_off()
{
  boost::mutex::scoped_lock lock(ctrl_mtx_);

  std::vector<uint8_t> dat;
  dat.resize(8);
  dat[0] = 0xfd;  // header
  dat[1] = 0xdf;  // header
  dat[2] = 0x04;  // data length
  dat[3] = 0x21;  // servo
  dat[4] = 0x1d;  // ID29
  dat[5] = 0x00;  // off
  dat[6] = 0x00;  // off
  dat[7] = 0xbd;  // checksum
  seed_.send_data(dat);
}

//////////////////////////////////////////////////
void AeroUpperController::Hand_Script(uint16_t _sendnum, uint16_t _script) {
  boost::mutex::scoped_lock lock(ctrl_mtx_);
  seed_.AERO_Snd_Script(_sendnum, _script);
}

//////////////////////////////////////////////////
AeroLowerController::AeroLowerController(const std::string& _port) :
    AeroControllerProto(_port, ID_LOWER)
{
  stroke_vector_.resize(AERO_DOF_LOWER);
  stroke_ref_vector_.resize(AERO_DOF_LOWER);
  stroke_cur_vector_.resize(AERO_DOF_LOWER);
  status_vector_.resize(AERO_DOF_LOWER);

  wheel_vector_.resize(AERO_DOF_WHEEL);
  wheel_ref_vector_.resize(AERO_DOF_WHEEL);
  wheel_cur_vector_.resize(AERO_DOF_WHEEL);

  stroke_joint_indices_.clear();
  stroke_joint_indices_.reserve(AERO_DOF_LOWER);

  wheel_indices_.clear();
  wheel_indices_.reserve(AERO_DOF_WHEEL);

  // adding code
  stroke_joint_indices_.push_back(
      AJointIndex(2, 0, 0, std::string("can_down")));
  stroke_joint_indices_.push_back(
      AJointIndex(2, 1, 1, std::string("can_up")));

  wheel_indices_.push_back(
      AJointIndex(2, 0, 2, std::string("can_front_r_wheel")));
  wheel_indices_.push_back(
      AJointIndex(2, 1, 3, std::string("can_rear_r_wheel")));
  wheel_indices_.push_back(
      AJointIndex(2, 2, 4, std::string("can_front_l_wheel")));
  wheel_indices_.push_back(
      AJointIndex(2, 3, 5, std::string("can_rear_l_wheel")));

  angle_joint_indices_["knee_joint"] = 28;
  angle_joint_indices_["ankle_joint"] = 29;


  get_command(CMD_GET_POS, stroke_cur_vector_);
  stroke_ref_vector_.assign(stroke_cur_vector_.begin(),
                            stroke_cur_vector_.end());
}

//////////////////////////////////////////////////
AeroLowerController::~AeroLowerController()
{
}

//////////////////////////////////////////////////
void AeroLowerController::servo_on()
{
  servo_command(1, 0);
}

//////////////////////////////////////////////////
void AeroLowerController::servo_off()
{
  servo_command(0, 0);
}

//////////////////////////////////////////////////
void AeroLowerController::wheel_on()
{
  servo_command(0x7fff, 1);
}

//////////////////////////////////////////////////
void AeroLowerController::wheel_only_off()
{
  servo_command(0x7fff, 0);
}

//////////////////////////////////////////////////
void AeroLowerController::servo_command(int16_t _d0, int16_t _d1)
{
  boost::mutex::scoped_lock lock(ctrl_mtx_);

  std::vector<int16_t> stroke_vector(stroke_joint_indices_.size(), _d0);
  std::vector<uint8_t> dat(RAW_DATA_LENGTH);

  stroke_to_raw_(stroke_vector, dat);

  // adding code
  encode_short_(_d1, &dat[RAW_HEADER_OFFSET + 2 * 2]);
  encode_short_(_d1, &dat[RAW_HEADER_OFFSET + 3 * 2]);
  encode_short_(_d1, &dat[RAW_HEADER_OFFSET + 4 * 2]);
  encode_short_(_d1, &dat[RAW_HEADER_OFFSET + 5 * 2]);


  seed_.send_command(CMD_MOTOR_SRV, 0, dat);
}

//////////////////////////////////////////////////
void AeroLowerController::set_wheel_velocity(
    std::vector<int16_t>& _wheel_vector, uint16_t _time)
{
  boost::mutex::scoped_lock lock(ctrl_mtx_);

  std::vector<uint8_t> dat(RAW_DATA_LENGTH);

  // use previous reference strokes to keep its positions
  stroke_to_raw_(stroke_ref_vector_, dat);

  // wheel to raw
  for (size_t i = 0; i < wheel_indices_.size(); ++i) {
    AJointIndex& aji = wheel_indices_[i];
    encode_short_(_wheel_vector[aji.stroke_index],
                  &dat[RAW_HEADER_OFFSET + aji.raw_index * 2]);
  }
  seed_.send_command(CMD_MOVE_SPD, _time, dat);

  // MoveAbs returns current stroke
  // MOVE_SPD also returns data, it is different behavior from Aero Command List ???
  std::vector<uint8_t> dummy;
  dummy.resize(RAW_DATA_LENGTH);
  seed_.read(dummy);
}

//////////////////////////////////////////////////
int32_t AeroLowerController::get_wheel_id(std::string& _name)
{
  for (size_t i = 0; i < wheel_indices_.size(); ++i) {
    if (wheel_indices_[i].joint_name == _name)
      return static_cast<int32_t>(wheel_indices_[i].stroke_index);
  }
  return -1;
}

//////////////////////////////////////////////////
std::string AeroLowerController::get_wheel_name(size_t _idx)
{
  return wheel_indices_[_idx].joint_name;
}

//////////////////////////////////////////////////
std::vector<int16_t>& AeroLowerController::get_reference_wheel_vector()
{
  return wheel_ref_vector_;
}
