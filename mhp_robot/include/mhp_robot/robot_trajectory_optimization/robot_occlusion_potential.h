/*********************************************************************
 *
 *  Software License Agreement
 *
 *  Copyright (c) 2025,
 *  TU Dortmund University, Institute of Control Theory and System Engineering
 *  All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 *  Authors: Maximilian Krämer, Heiko Renz
 *  Maintainer(s)/Modifier(s):
 *********************************************************************/

#ifndef ROBOT_OCCLUSION_POTENTIAL_H
#define ROBOT_OCCLUSION_POTENTIAL_H

#include <Eigen/Core>
#include <mhp_robot/MsgOcclusionDist.h>
#include <mhp_robot/robot_collision/robot_collision.h>
#include <mhp_robot/robot_misc/planning_delay.h>
#include <mhp_robot/robot_obstacle/obstacle_list.h>
#include <mhp_robot/robot_trajectory_optimization/robot_stage_preprocessor.h>
#include <tf_conversions/tf_eigen.h>
#include <memory>


namespace mhp_robot
{
namespace robot_trajectory_optimization
{
class RobotOcclusionPotential
{
 public:
  using Ptr = std::shared_ptr<RobotOcclusionPotential>;
  using UPtr = std::unique_ptr<RobotOcclusionPotential>;

  RobotOcclusionPotential() = default;

  RobotOcclusionPotential(const RobotOcclusionPotential&) = delete;
  RobotOcclusionPotential(RobotOcclusionPotential&&) = delete;
  RobotOcclusionPotential& operator=(const RobotOcclusionPotential&) = delete;
  RobotOcclusionPotential& operator=(RobotOcclusionPotential&&) = delete;
  virtual ~RobotOcclusionPotential()
  {
  }

  virtual double computePotentials(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k) = 0;

  virtual void computeGradient(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k, Eigen::Ref<Eigen::VectorXd> dx);

  virtual void computeHessian(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k, Eigen::Ref<Eigen::MatrixXd> dxdx);

  bool initialize(robot_kinematic::RobotKinematic::UPtr robot_kinematic);
  bool update(double dt);
  bool isInitialized() const;

  void setPlannerId(const int id)
  {
    _planner_id = id;
    if (_planner_id != 0)
      _ms_planner_mode = true;
  }
  int getPlannerId() const
  {
    return _planner_id;
  }
  bool isPlannerSet() const
  {
    return _ms_planner_mode;
  }

  virtual double getOptimalViewDistance() const
  {
    return 0.0;
  };

  double getCostsAllk() const
  {
    double total_cost = 0.0;
    for (const auto& [k, cost] : _costs_for_k)
    {
      total_cost += cost;
    }
    return total_cost;
  };

 protected:
  using RobotKinematic = robot_kinematic::RobotKinematic;

  void occlusionDistributionCallback(const mhp_robot::MsgOcclusionDist::ConstPtr& msg);

  RobotKinematic::UPtr _robot_kinematic;
  ros::Subscriber _occlusion_dist_sub;

  Eigen::MatrixXd _occlusion_distribution;            // [radius, theta, phi, occlusion]
  Eigen::MatrixXd _occlusion_distribution_cartesian;  // [x, y, z, occlusion]
  Eigen::MatrixXd _occlusion_predictions;             // [row: prediction step, col: occlusion]]
  Eigen::Vector3d _occlusion_sphere_center = Eigen::Vector3d::Zero();

  // Callback variables
  Eigen::MatrixXd _occlusion_distribution_cb;            // [radius, theta, phi, occlusion]
  Eigen::MatrixXd _occlusion_distribution_cartesian_cb;  // [x, y, z, occlusion]
  Eigen::MatrixXd _occlusion_predictions_cb;             // [row: prediction step, col: occlusion]]
  Eigen::Vector3d _occlusion_sphere_center_cb = Eigen::Vector3d::Zero();

  Eigen::Matrix4d _tf_cam_to_ee_link = Eigen::Matrix4d::Identity();

  std::mutex _occ_mutex;

  double _dt = 0.1;
  double _eps = 1e-7;
  std::string _camera_frame = "depth_camera_link";

  bool _initialized = false;

  double _theta_threshold = 0.2;
  double _phi_threshold = 0.2;
  bool _use_occlusion_prediction = false;

  // Variables for Multistage Planner
  int _planner_id = 0;
  bool _ms_planner_mode = false;

  Eigen::Vector3d sphericalToCartesian(const double& radius, const double& theta, const double& phi) const;
  Eigen::Vector3d sphericalToCartesian(Eigen::Ref<Eigen::Vector3d> spherical) const
  {
    return sphericalToCartesian(spherical(0), spherical(1), spherical(2));
  };
  Eigen::Vector3d cartesianToSpherical(const double x, const double y, const double z) const;
  Eigen::Vector3d cartesianToSpherical(const Eigen::Ref<Eigen::Vector3d> pos) const
  {
    return cartesianToSpherical(pos(0), pos(1), pos(2));
  };

  std::unordered_map<int, double> _costs_for_k;  // Store costs for each k to avoid recomputation

 private:
};

}  // namespace robot_trajectory_optimization
}  // namespace mhp_robot

#endif  // ROBOT_OCCLUSION_POTENTIAL_H
