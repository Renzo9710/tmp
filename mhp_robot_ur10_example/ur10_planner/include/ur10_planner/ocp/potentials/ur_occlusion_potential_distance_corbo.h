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

#ifndef UR_OCCLUSION_POTENTIAL_DISTANCE_MHP_PLANNER_H
#define UR_OCCLUSION_POTENTIAL_DISTANCE_MHP_PLANNER_H

#include <mhp_planner/core/factory.h>
#include <mhp_robot/robot_trajectory_optimization/robot_occlusion_potential_distance.h>
#include <ur10_planner/ocp/potentials/ur_base_occlusion_potential_corbo.h>

namespace mhp_planner {

class UROcclusionPotentialDistance : public URBaseOcclusionPotential,
                                       public mhp_robot::robot_trajectory_optimization::RobotOcclusionPotentialDistance
{
 public:
    using Ptr  = std::shared_ptr<UROcclusionPotentialDistance>;
    using UPtr = std::unique_ptr<UROcclusionPotentialDistance>;

    UROcclusionPotentialDistance() = default;

    URBaseOcclusionPotential::Ptr getInstance() const override;

    bool fromParameterServer(const std::string& ns) override;

 private:
    bool _extended = false;
};
FACTORY_REGISTER_OBJECT(UROcclusionPotentialDistance, URBaseOcclusionPotential)

}  // namespace mhp_planner

#endif  // UR_OCCLUSION_POTENTIAL_DISTANCE_MHP_PLANNER_H
