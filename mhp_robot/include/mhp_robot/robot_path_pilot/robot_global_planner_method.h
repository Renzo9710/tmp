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

#ifndef ROBOT_GLOBAL_PLANNER_METHOD_H
#define ROBOT_GLOBAL_PLANNER_METHOD_H

#include <ompl/base/Planner.h>
#include <ompl/base/SpaceInformation.h>

namespace mhp_robot {
namespace robot_path_pilot {

class RobotGlobalPlannerMethod
{
 public:
    using Ptr  = std::shared_ptr<RobotGlobalPlannerMethod>;
    using UPtr = std::unique_ptr<RobotGlobalPlannerMethod>;

    RobotGlobalPlannerMethod() = default;

    RobotGlobalPlannerMethod(const RobotGlobalPlannerMethod&) = delete;
    RobotGlobalPlannerMethod(RobotGlobalPlannerMethod&&)      = default;
    RobotGlobalPlannerMethod& operator=(const RobotGlobalPlannerMethod&) = delete;
    RobotGlobalPlannerMethod& operator=(RobotGlobalPlannerMethod&&) = default;
    virtual ~RobotGlobalPlannerMethod() {}

    virtual bool initialize(ompl::base::SpaceInformationPtr space_information) = 0;

    virtual void clearQuery();

    void clearPlan();

    void setProblemDefinition(ompl::base::ProblemDefinitionPtr problem);

    bool isSetup() const;
    void setup();
    
    ompl::base::PlannerStatus solve(double solve_time);

    std::string getName() const;

 protected:
    ompl::base::PlannerPtr _planner;
    ompl::base::SpaceInformationPtr _space_information;
};

}  // namespace robot_path_pilot
}  // namespace mhp_robot

#endif  // ROBOT_GLOBAL_PLANNER_METHOD_H
