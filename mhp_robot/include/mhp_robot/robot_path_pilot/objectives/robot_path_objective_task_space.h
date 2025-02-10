/*********************************************************************
 *
 *  Software License Agreement
 *
 *  Copyright (c) 2018,
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
*  Authors: Maximilian Krämer
*  Maintainer(s)/Modifier(s): Heiko Renz
 *********************************************************************/

#ifndef ROBOT_PATH_OBJECTIVE_TASK_SPACE_H
#define ROBOT_PATH_OBJECTIVE_TASK_SPACE_H

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <mhp_robot/robot_kinematic/robot_kinematic.h>

namespace mhp_robot {
namespace robot_path_pilot {

class RobotPathObjectiveTaskSpace : public ompl::base::StateCostIntegralObjective
{
 public:
    RobotPathObjectiveTaskSpace(ompl::base::SpaceInformationPtr si, robot_kinematic::RobotKinematic::UPtr robot_kinematic);

    ompl::base::Cost stateCost(const ompl::base::State* s) const override;
    ompl::base::Cost motionCost(const ompl::base::State* s1, const ompl::base::State* s2) const override;

 private:
    using RobotKinematic = robot_kinematic::RobotKinematic;

    RobotKinematic::UPtr _robot_kinematic;
    int _dim = 6;
};

}  // namespace robot_path_pilot
}  // namespace mhp_robot

#endif  // ROBOT_PATH_OBJECTIVE_TASK_SPACE_H
