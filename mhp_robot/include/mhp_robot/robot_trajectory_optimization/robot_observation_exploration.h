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
 *  Authors: Heiko Renz
 *  Maintainer(s)/Modifier(s):
 *********************************************************************/

#ifndef ROBOT_OBSERVATION_EXPLORATION_H
#define ROBOT_OBSERVATION_EXPLORATION_H

#include <Eigen/Core>
#include <mhp_robot/robot_kinematic/robot_kinematic.h>
#include <mhp_robot/robot_trajectory_optimization/robot_information_gain.h>
#include <mhp_robot/robot_trajectory_optimization/robot_occlusion_potential.h>
#include <memory>

namespace mhp_robot
{
namespace robot_trajectory_optimization
{

class RobotObservationExploration
{
 public:
  using Ptr = std::shared_ptr<RobotObservationExploration>;
  using UPtr = std::unique_ptr<RobotObservationExploration>;

  RobotObservationExploration() = default;

  RobotObservationExploration(const RobotObservationExploration&) = delete;
  RobotObservationExploration(RobotObservationExploration&&) = delete;
  RobotObservationExploration& operator=(const RobotObservationExploration&) = delete;
  RobotObservationExploration& operator=(RobotObservationExploration&&) = delete;
  virtual ~RobotObservationExploration()
  {
  }

  virtual double computePotentials(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k) = 0;

  virtual void computeGradient(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k, Eigen::Ref<Eigen::VectorXd> dx);

  virtual void computeHessian(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k, Eigen::Ref<Eigen::MatrixXd> dxdx);

  virtual bool initialize(robot_kinematic::RobotKinematic::UPtr robot_kinematic);
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

  bool isTrackingMode() const
  {
    return _tracking_mode;
  }

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
  using RobotInformationGain = robot_trajectory_optimization::RobotInformationGain;
  using RobotOcclusionPotential = robot_trajectory_optimization::RobotOcclusionPotential;
  using RobotKinematic = robot_kinematic::RobotKinematic;

  double _dt = 0.1;
  double _eps = 1e-7;
  bool _initialized = false;

  // Variables for Multistage Planner
  int _planner_id = 0;
  bool _ms_planner_mode = false;

  RobotInformationGain::Ptr _robot_information_gain;
  RobotOcclusionPotential::Ptr _robot_occlusion_potential;

  RobotKinematic::UPtr _robot_kinematic;

  bool _tracking_mode = true;  // true for observing the environment (occlusion potential), false for exploring it (info gain)

  std::unordered_map<int, double> _costs_for_k;  // stores costs for each k
  
 private:
};

}  // namespace robot_trajectory_optimization
}  // namespace mhp_robot

#endif  // ROBOT_OBSERVATION_EXPLORATION_H
