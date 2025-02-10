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
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <mhp_robot/robot_path_pilot/objectives/robot_path_objective_task_space.h>

namespace mhp_robot {
namespace robot_path_pilot {

RobotPathObjectiveTaskSpace::RobotPathObjectiveTaskSpace(ompl::base::SpaceInformationPtr si, RobotKinematic::UPtr robot_kinematic)
    : ompl::base::StateCostIntegralObjective(si, true), _robot_kinematic(std::move(robot_kinematic))
{
    _dim = si->getStateDimension();
}

ompl::base::Cost RobotPathObjectiveTaskSpace::stateCost(const ompl::base::State* s) const
{
    return ompl::base::StateCostIntegralObjective::identityCost();
}

ompl::base::Cost RobotPathObjectiveTaskSpace::motionCost(const ompl::base::State* s1, const ompl::base::State* s2) const
{
    // Compute cost
    Eigen::VectorXd q1 = Eigen::Map<const Eigen::VectorXd>(s1->as<ompl::base::RealVectorStateSpace::StateType>()->values, _dim);
    Eigen::VectorXd q2 = Eigen::Map<const Eigen::VectorXd>(s2->as<ompl::base::RealVectorStateSpace::StateType>()->values, _dim);

    Eigen::Vector3d p1 = _robot_kinematic->getEndEffectorMatrix(q1).block<3, 1>(0, 3);
    Eigen::Vector3d p2 = _robot_kinematic->getEndEffectorMatrix(q2).block<3, 1>(0, 3);

    return ompl::base::Cost((p1 - p2).norm());
}

}  // namespace robot_path_pilot
}  // namespace mhp_robot
