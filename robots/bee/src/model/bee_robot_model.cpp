#include <bee/model/bee_robot_model.h>
#include <urdf/model.h>
#include <cmath>
#include <stdexcept>

namespace bee::control
{
// Match RobotModel/KDL: the dummy root link's inertia is not supported by KDL.
// Fixed and moving child links both contribute, including any modelled feet.
double modelMass(const std::string& xml)
{
  urdf::Model model;
  if (!model.initString(xml)) throw std::invalid_argument("invalid robot_description");
  double mass = 0;
  for (const auto& entry : model.links_)
  {
    const auto& link = entry.second;
    if (link == model.getRoot() || !link->inertial) continue;
    const double m = link->inertial->mass;
    if (!std::isfinite(m) || m < 0) throw std::invalid_argument("invalid URDF link mass");
    mass += m;
  }
  if (!std::isfinite(mass) || mass <= 0) throw std::invalid_argument("URDF has no positive body mass");
  return mass;
}
}

BeeRobotModel::BeeRobotModel(bool init_with_rosparam, bool verbose, double fc_t_min_thre, double epsilon) :
  GimbalrotorRobotModel(init_with_rosparam, verbose, fc_t_min_thre, epsilon)
{
}

void BeeRobotModel::updateRobotModelImpl(const KDL::JntArray& joint_positions)
{
  GimbalrotorRobotModel::updateRobotModelImpl(joint_positions);

  const auto seg_tf_map = getSegmentsTf();
  KDL::Frame fix_cp_frame = seg_tf_map.at("contact_point");
  KDL::Frame variable_cp_frame;
  variable_cp_frame.M = fix_cp_frame.M * getCogDesireOrientation<KDL::Rotation>().Inverse();
  variable_cp_frame.p = fix_cp_frame.p;
  setContactFrame(variable_cp_frame);
  KDL::Frame Cog2Cp;
  setCog2Cp(getCog<KDL::Frame>().Inverse() * variable_cp_frame);

  // calcCenterOfMoving();
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(BeeRobotModel, aerial_robot_model::RobotModel);
