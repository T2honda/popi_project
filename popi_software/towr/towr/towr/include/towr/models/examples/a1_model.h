#ifndef TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_A1_MODEL_H_
#define TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_A1_MODEL_H_

#include <towr/models/kinematic_model.h>
#include <towr/models/single_rigid_body_dynamics.h>
#include <towr/models/endeffector_mappings.h>

namespace towr {

/**
 * @brief The Kinematics of the quadruped robot A1.
 */
class PopiKinematicModel : public KinematicModel {
public:
  PopiKinematicModel () : KinematicModel(4)
  {
    //valeurs nominales dans le repère aile
    const double x_nominal_b = -0.0402;
    const double y_nominal_b = 0.1077;
    const double z_nominal_b = -0.4875;

    const double x_ecart_base_aile = 0.3305;
    const double y_ecart_base_aile = 0.175;
    const double z_ecart_base_aile = 0.051;

    nominal_stance_.at(LF) <<  x_nominal_b + x_ecart_base_aile,   y_nominal_b + y_ecart_base_aile, z_nominal_b - z_ecart_base_aile;
    nominal_stance_.at(RF) <<  x_nominal_b + x_ecart_base_aile,  - y_nominal_b - y_ecart_base_aile, z_nominal_b - z_ecart_base_aile;
    nominal_stance_.at(LH) <<  x_nominal_b - x_ecart_base_aile,   y_nominal_b + y_ecart_base_aile, z_nominal_b - z_ecart_base_aile;
    nominal_stance_.at(RH) <<  x_nominal_b - x_ecart_base_aile,  - y_nominal_b - y_ecart_base_aile, z_nominal_b - z_ecart_base_aile;

    max_dev_from_nominal_ << 0.25, 0.20, 0.10;
  }
};

/**
 * @brief The Dynamics of the quadruped robot Popi.
 */
class PopiDynamicModel : public SingleRigidBodyDynamics {
public: // double mass, double Ixx, double Iyy, double Izz, double Ixy, double Ixz, double Iyz, int ee_count
  PopiDynamicModel() : SingleRigidBodyDynamics(50.033,
                      0.97314, 3.88974, 4.74716, 0.0, 0.0, 0.0,
                      4) {}
};

} /* namespace towr */

#endif /* TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_POPI_MODEL_H_ */
