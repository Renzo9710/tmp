/*********************************************************************
 *
 *  Software License Agreement
 *
 *  Copyright (c) 2024,
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

#include <mhp_robot/robot_trajectory_optimization/robot_observation_exploration.h>

namespace mhp_robot
{
namespace robot_trajectory_optimization
{
void RobotObservationExploration::computeGradient(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k,
                                                  Eigen::Ref<Eigen::VectorXd> dx)
{
  Eigen::VectorXd diff = Eigen::VectorXd::Zero(x_k.size());
  for (int i = 0; i < x_k.size(); ++i)
  {
    diff(i) = _eps;
    dx(i) = (computePotentials(k, x_k + diff) - computePotentials(k, x_k - diff)) / (2 * _eps);
    diff(i) = 0.0;
  }
}

void RobotObservationExploration::computeHessian(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k,
                                                 Eigen::Ref<Eigen::MatrixXd> dxdx)
{
  int n = x_k.size();
  Eigen::VectorXd diff = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd ldx(n), rdx(n);
  for (int i = 0; i < n; ++i)
  {
    diff(i) = _eps;
    computeGradient(k, x_k + diff, rdx);
    computeGradient(k, x_k - diff, ldx);
    dxdx.col(i) = (rdx - ldx) / (2 * _eps);
    diff(i) = 0.0;
  }
}

bool RobotObservationExploration::initialize(robot_kinematic::RobotKinematic::UPtr robot_kinematic)
{
  if (_initialized)
    return true;

  _robot_kinematic = std::move(robot_kinematic);

  if (!_robot_information_gain)
  {
    ROS_WARN("RobotObservationExploration: No information gain or occlusion potential instantiated.");
  }
  else
  {
    if (!_robot_information_gain->initialize(_robot_kinematic->createUniqueInstance()))
    {
      ROS_ERROR("RobotObservationExploration: Failed to initialize information gain.");
      return false;
    }
  }

  if (!_robot_occlusion_potential)
  {
    ROS_WARN("RobotObservationExploration: No occlusion potential instantiated.");
  }
  else
  {
    if (!_robot_occlusion_potential->initialize(_robot_kinematic->createUniqueInstance()))
    {
      ROS_ERROR("RobotObservationExploration: Failed to initialize occlusion potential.");
      return false;
    }
  }

  _initialized = true;

  return true;
}

bool RobotObservationExploration::update(double dt)
{
  _dt = dt;
  if (_robot_information_gain)
  {
    if (!_robot_information_gain->update(dt))
    {
      ROS_ERROR("RobotObservationExploration: Failed to update information gain.");
      return false;
    }
  }
  if (_robot_occlusion_potential)
  {
    if (!_robot_occlusion_potential->update(dt))
    {
      ROS_ERROR("RobotObservationExploration: Failed to update occlusion potential.");
      return false;
    }
  }
  return true;
}

bool RobotObservationExploration::isInitialized() const
{
  return _initialized;
}

}  // namespace robot_trajectory_optimization
}  // namespace mhp_robot
