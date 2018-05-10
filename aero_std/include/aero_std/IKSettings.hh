#ifndef _IK_SETTINGS_H_
#define _IK_SETTINGS_H_

#define ENUM_TO_STRING(var) #var

#include <unordered_map>

namespace aero
{
  typedef Eigen::Affine3d      Transform;
  typedef Eigen::Vector3d      Vector3;
  typedef Eigen::Matrix3d      Matrix3;
  typedef Eigen::Translation3d Translation;
  typedef Eigen::Quaterniond   Quaternion;
  typedef Eigen::AngleAxisd    AngleAxis;
  typedef Eigen::Matrix<double, 6, 1> Vector6;

  enum struct arm : int {rarm, larm, either, both_arms};

  enum struct ikrange : int {arm, upperbody, arm_lifter, wholebody}; // torso -> upperbody, lifter -> wholebody

  enum struct eef : int {hand, grasp, pick, index, thumb, none};

  enum struct pose : int {reset, reset_manip, move, initial};

  enum struct tracking : int {disable, rpy, map, base, map_static, base_static, topic, tf};
  /// robot dependant
  enum struct joint : int {r_shoulder_p,
      r_shoulder_r,
      r_shoulder_y,
      r_elbow,
      r_wrist_y,
      r_wrist_p,
      r_wrist_r,
      r_hand_y,
      l_shoulder_p,
      l_shoulder_r,
      l_shoulder_y,
      l_elbow,
      l_wrist_y,
      l_wrist_p,
      l_wrist_r,
      l_hand_y,
      waist_y,
      waist_p,
      waist_r,
      neck_y,
      neck_p,
      neck_r,
      ankle,
      knee,
      lifter_x, // backward compatibility
      lifter_z, // backward compatibility
      unknown
      };

  // typedef std::map<std::string, double> stringmap; // robot_interface::...
  typedef std::map<aero::joint, double> joint_angle_map;

  /// robot dependant
  const std::map<aero::joint, std::string> joint_map = {
    {aero::joint::r_shoulder_p,"r_shoulder_p_joint"},
    {aero::joint::r_shoulder_r,"r_shoulder_r_joint"},
    {aero::joint::r_shoulder_y,"r_shoulder_y_joint"},
    {aero::joint::r_elbow,"r_elbow_joint"},
    {aero::joint::r_wrist_y,"r_wrist_y_joint"},
    {aero::joint::r_wrist_p,"r_wrist_p_joint"},
    {aero::joint::r_wrist_r,"r_wrist_r_joint"},
    {aero::joint::r_hand_y,"r_hand_y_joint"},
    {aero::joint::l_shoulder_p,"l_shoulder_p_joint"},
    {aero::joint::l_shoulder_r,"l_shoulder_r_joint"},
    {aero::joint::l_shoulder_y,"l_shoulder_y_joint"},
    {aero::joint::l_elbow,"l_elbow_joint"},
    {aero::joint::l_wrist_y,"l_wrist_y_joint"},
    {aero::joint::l_wrist_p,"l_wrist_p_joint"},
    {aero::joint::l_wrist_r,"l_wrist_r_joint"},
    {aero::joint::l_hand_y,"l_hand_y_joint"},
    {aero::joint::waist_y,"waist_y_joint"},
    {aero::joint::waist_p,"waist_p_joint"},
    {aero::joint::waist_r,"waist_r_joint"},
    {aero::joint::neck_y,"neck_y_joint"},
    {aero::joint::neck_p,"neck_p_joint"},
    {aero::joint::neck_r,"neck_r_joint"},
    {aero::joint::knee,"knee_joint"},
    {aero::joint::ankle,"ankle_joint"},
    {aero::joint::lifter_x,"virtual_lifter_x_joint"},
    {aero::joint::lifter_z,"virtual_lifter_z_joint"}
  };

  /// can be generated by joint_map
  const std::map<std::string, aero::joint> string_map = {
    {"r_shoulder_p_joint", aero::joint::r_shoulder_p},
    {"r_shoulder_r_joint", aero::joint::r_shoulder_r},
    {"r_shoulder_y_joint" ,aero::joint::r_shoulder_y},
    {"r_elbow_joint" ,aero::joint::r_elbow},
    {"r_wrist_y_joint" ,aero::joint::r_wrist_y},
    {"r_wrist_p_joint" ,aero::joint::r_wrist_p},
    {"r_wrist_r_joint" ,aero::joint::r_wrist_r},
    {"r_hand_y_joint" ,aero::joint::r_hand_y},
    {"l_shoulder_p_joint" ,aero::joint::l_shoulder_p},
    {"l_shoulder_r_joint" ,aero::joint::l_shoulder_r},
    {"l_shoulder_y_joint" ,aero::joint::l_shoulder_y},
    {"l_elbow_joint" ,aero::joint::l_elbow},
    {"l_wrist_y_joint" ,aero::joint::l_wrist_y},
    {"l_wrist_p_joint" ,aero::joint::l_wrist_p},
    {"l_wrist_r_joint" ,aero::joint::l_wrist_r},
    {"l_hand_y_joint" ,aero::joint::l_hand_y},
    {"waist_y_joint" ,aero::joint::waist_y},
    {"waist_p_joint" ,aero::joint::waist_p},
    {"waist_r_joint" ,aero::joint::waist_r},
    {"neck_y_joint", aero::joint::neck_y},
    {"neck_p_joint", aero::joint::neck_p},
    {"neck_r_joint", aero::joint::neck_r},
    {"knee_joint" ,aero::joint::knee},
    {"ankle_joint" ,aero::joint::ankle},
    {"virtual_lifter_x_joint" ,aero::joint::lifter_x},
    {"virtual_lifter_z_joint" ,aero::joint::lifter_z}
  };

  inline const std::string arm2lr_(aero::arm _arm)
  {
    switch(_arm) {
    case aero::arm::rarm:
      return "r";
      break;
    case aero::arm::larm:
      return "l";
      break;
      //case aero::arm::either:
      //break;
      //case aero::arm::both:
      //break;
    default:
      return "";
      break;
    }
  }

  inline const std::string arm2leftright_(aero::arm _arm)
  {
    switch(_arm) {
    case aero::arm::rarm:
      return "right";
      break;
    case aero::arm::larm:
      return "left";
      break;
      //case aero::arm::either:
      //break;
      //case aero::arm::both:
      //break;
    default:
      return "";
      break;
    }
  }

  inline const std::string arm2string_(aero::arm _arm)
  {
    switch(_arm) {
    case aero::arm::rarm:
      return "rarm";
      break;
    case aero::arm::larm:
      return "larm";
      break;
    case aero::arm::either:
      return "either";
      break;
    case aero::arm::both_arms:
      return "both_arms";
      break;
    default:
      return "";
      break;
    }
  }

  inline const std::string moveGroup(aero::arm _arm, aero::ikrange _range)
  {
    std::string mg( arm2string_(_arm) );
    switch(_range) {
    case aero::ikrange::upperbody:
      mg = mg + "_with_waist";
      break;
    case aero::ikrange::wholebody:
      mg = mg + "_with_torso";
      break;
    case aero::ikrange::arm_lifter:
      mg = mg + "_with_lifter";
      break;
    default:
      break;
    }
    return mg;
  }

  inline void controllerGroup(std::vector<std::string > &_names, aero::ikrange _range, bool _head)
  {
    switch (_range) {
    case aero::ikrange::arm:
      _names.push_back("rarm");
      _names.push_back("larm");
      break;
    case aero::ikrange::upperbody:
      _names.push_back("rarm");
      _names.push_back("larm");
      _names.push_back("waist");
      break;
    case aero::ikrange::arm_lifter:
      _names.push_back("rarm");
      _names.push_back("larm");
      _names.push_back("lifter");
      break;
    case aero::ikrange::wholebody:
      _names.push_back("rarm");
      _names.push_back("larm");
      _names.push_back("waist");
      _names.push_back("lifter");
      break;
    }
    if(_head) {
      _names.push_back("head");
    }
  }

  inline const std::string eefLink(aero::arm _arm, aero::eef _eef)
  {
    std::string ln( arm2lr_(_arm) );
    switch (_eef) {
    case aero::eef::hand:
      ln = ln + "_hand_link";
      break;
    case aero::eef::grasp:
      ln = ln + "_eef_grasp_link";
      break;
    case aero::eef::pick:
      ln = ln + "_eef_pick_link";
      break;
    case aero::eef::index:
      ln = ln + "_index_tip_link";
      break;
    case aero::eef::thumb:
      ln = ln + "_thumb_tip_link";
      break;
    default:
      ln = "";
    }
    return ln;
  }

  //inline std::string joint2JointName(aero::joint _joint)
  inline const std::string &joint2str(aero::joint _joint)
  {
    // if joint_map is correct, error does not occur
    // add joint existing check by user, if ( _joint != aero::joint::unknown ) {
    return aero::joint_map.at(_joint);
  }

  //inline aero::joint jointName2Joint(std::string _joint_name)
  inline aero::joint str2joint(const std::string &_joint_name)
  {
    auto it = aero::string_map.find(_joint_name);
    if (it == aero::string_map.end()) {
      return aero::joint::unknown;
    }
    return it->second;
  }

  inline void jointMap2StringMap(const joint_angle_map &_j_map, std::map<std::string, double> &_s_map)
  {
    _s_map.clear();
    for(auto it = _j_map.begin(); it != _j_map.end(); ++it) {
      if ( it->first != aero::joint::unknown ) {
        _s_map[joint2str(it->first)] = it->second;
      }
    }
  }

  inline void stringMap2JointMap(const std::map<std::string, double> &_s_map, joint_angle_map &_j_map)
  {
    _j_map.clear();
    for(auto it = _s_map.begin(); it != _s_map.end(); ++it) {
      const aero::joint &j = str2joint(it->first);
      if ( j != aero::joint::unknown ) {
        _j_map[j] = it->second;
      }
    }
  }

  struct fullarm {
    joint_angle_map joints;
    double l_hand;
    double r_hand;
  };

  inline void mid_coords(double _rate, const aero::Transform &_t_a, const aero::Transform &_t_b,
                         aero::Transform &_res) {
    aero::Vector3 va(_t_a.translation());
    aero::Vector3 vb(_t_b.translation());
    aero::Quaternion qa(_t_a.linear());
    aero::Quaternion qb(_t_b.linear());

    aero::Translation tr(va * (1 - _rate) + vb * _rate);
    aero::Quaternion  q(qa.slerp(_rate, qb));

    _res = tr * q;
  }
}
static std::ostream& operator<<(std::ostream& os, const aero::Quaternion &qq)
{
  os << " #f("
     << qq.w() << " "
     << qq.x() << " "
     << qq.y() << " "
     << qq.z() << ")";
  return os;
}
static std::ostream& operator<<(std::ostream& os, const aero::Transform &tr)
{
  aero::Vector3 tt = tr.translation();
  aero::Quaternion qq(tr.linear());
  os << "(cons #f("
     << tt(0) << " "
     << tt(1) << " "
     << tt(2) << ")";
  os << qq << ")";
  return os;
}
#endif
