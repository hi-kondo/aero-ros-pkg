/*
 * This file auto-generated from script. Do not Edit!
 * Original : aero_description/arm_typeC/controllers/AeroHandController.hh
*/
/// robot dependant constant // for fcsc hand
#define POSITION_Right 12
// POSITION RIGHT change
#define POSITION_Left  27

const std::vector<std::string > rhand_joints = { "r_thumb_joint" };
const std::vector<std::string > lhand_joints = { "l_thumb_joint" };

const std::string l_grasp_check_joint = "l_thumb_joint";
const std::string r_grasp_check_joint = "r_thumb_joint";
const std::string l_grasp_fast_check_joint = "l_thumb_joint";
const std::string r_grasp_fast_check_joint = "r_thumb_joint";

#define L_GRASP_CHECK_bad(g_srv, req)  (g_srv.response.angles[0] <   req.thre_warn)
#define L_GRASP_CHECK_fail(g_srv, req) (g_srv.response.angles[0] >   req.thre_fail)
#define R_GRASP_CHECK_bad(g_srv, req)  (g_srv.response.angles[1] >  -req.thre_warn)
#define R_GRASP_CHECK_fail(g_srv, req) (g_srv.response.angles[1] <  -req.thre_fail)

#define L_GRASP_FAST_CHECK(g_srv, req) (g_srv.response.angles[0] >   req.thre_fail)
#define R_GRASP_FAST_CHECK(g_srv, req) (g_srv.response.angles[1] < - req.thre_fail)

#define L_OPEN() {                                   \
    map["l_thumb_joint"] = -15.0 * M_PI / 180.0;     \
  }
#define R_OPEN() {                                   \
    map["r_thumb_joint"] =  15.0 * M_PI / 180.0;     \
  }

#define L_GRASP() {                                             \
    map["l_thumb_joint"]      =  larm_angle * M_PI / 180/ 4.0;  \
  }
#define R_GRASP() {                                             \
    map["r_thumb_joint"]      =  rarm_angle * M_PI / 180 / 4.0; \
  }
/// end dependant