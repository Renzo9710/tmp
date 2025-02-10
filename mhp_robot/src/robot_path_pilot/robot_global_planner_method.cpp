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
#include <mhp_robot/robot_path_pilot/robot_global_planner_method.h>
#include <ros/io.h>

namespace mhp_robot {
namespace robot_path_pilot {

void RobotGlobalPlannerMethod::setProblemDefinition(ompl::base::ProblemDefinitionPtr problem) { _planner->setProblemDefinition(problem); }

bool RobotGlobalPlannerMethod::isSetup() const { return _planner->isSetup(); }

void RobotGlobalPlannerMethod::setup() { _planner->setup(); }

void RobotGlobalPlannerMethod::clearPlan() { _planner->clear(); }

void RobotGlobalPlannerMethod::clearQuery()
{
    ROS_WARN("GlobalPlannerMethodInterface: Default clear query removes ALL planner data.");

    clearPlan();
}

ompl::base::PlannerStatus RobotGlobalPlannerMethod::solve(double solve_time) { return _planner->solve(solve_time); }

std::string RobotGlobalPlannerMethod::getName() const { return _planner->getName(); }

}  // namespace robot_path_pilot
}  // namespace mhp_robot
