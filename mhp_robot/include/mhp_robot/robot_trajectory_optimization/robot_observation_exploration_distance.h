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

#ifndef ROBOT_OBSERVATION_EXPLORATION_DISTANCE_H
#define ROBOT_OBSERVATION_EXPLORATION_DISTANCE_H

#include <geometry_msgs/PoseStamped.h>
#include <mhp_robot/robot_trajectory_optimization/robot_observation_exploration.h>
#include <ros/ros.h>

namespace mhp_robot
{
namespace robot_trajectory_optimization
{
class RobotObservationExplorationDistance : virtual public RobotObservationExploration
{
 public:
  RobotObservationExplorationDistance() = default;

  using Ptr = std::shared_ptr<RobotObservationExplorationDistance>;
  using UPtr = std::unique_ptr<RobotObservationExplorationDistance>;

  bool initialize(robot_kinematic::RobotKinematic::UPtr robot_kinematic) override;

  double computePotentials(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k) override;

 protected:
  double _switch_distance_human_object =
      0.5;  // distance to switch from observation to exploration (from occlusion potential to information gain)
  
 private:
  Eigen::Vector3d _poi_info_gain;
  Eigen::Vector3d _hand_com;
  bool _tracking_mode_close_enough = false;
  Eigen::Vector3d _robot_base = Eigen::Vector3d{0.0, 0.0, 0.9273};  // shoulder link on pedestal
  double _robot_reach = 1.3;  // Robot reach of UR10
  double _overall_robot_view_distance;  // Robot reach + optimal view distance of occlusion potential
  ros::Subscriber _hand_com_sub;
  void handComCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  bool _com_initialized = false;
};

}  // namespace robot_trajectory_optimization
}  // namespace mhp_robot

#endif  // ROBOT_OBSERVATION_EXPLORATION_DISTANCE_H
