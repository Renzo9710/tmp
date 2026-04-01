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
 *  Authors: Maximilian Krämer, Heiko Renz
 *  Maintainer(s)/Modifier(s):
 *********************************************************************/

#ifndef UR_BASE_OBSERVATION_EXPLORATION_CORBO_H
#define UR_BASE_OBSERVATION_EXPLORATION_CORBO_H

#include <mhp_robot/robot_trajectory_optimization/robot_observation_exploration.h>
#include <ur10_planner/ocp/gains/ur_base_information_gain_corbo.h>
#include <ur10_planner/ocp/potentials/ur_base_occlusion_potential_corbo.h>

namespace mhp_planner
{
class URBaseObservationExploration
  : virtual public mhp_robot::robot_trajectory_optimization::RobotObservationExploration
{
 public:
  using Ptr = std::shared_ptr<URBaseObservationExploration>;
  using UPtr = std::unique_ptr<URBaseObservationExploration>;

  URBaseObservationExploration() = default;

  virtual Ptr getInstance() const = 0;

  URBaseInformationGain::Ptr _ur_information_gain = nullptr;
  URBaseOcclusionPotential::Ptr _ur_occlusion_potential = nullptr;

  virtual bool fromParameterServer(const std::string& ns) = 0;

};

}  // namespace mhp_planner

#endif  // UR_BASE_OBSERVATION_EXPLORATION_CORBO_H
