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
 *********************************************************************/
#ifndef ERGODIC_H
#define ERGODIC_H

#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <mhp_robot/robot_path_pilot/robot_global_planner_method.h>
#include <mhp_robot/robot_path_pilot/ompl_planners/ompl_ergodic.h>

namespace mhp_robot {
namespace robot_path_pilot {
namespace planner_methods {

class Ergodic : virtual public RobotGlobalPlannerMethod
{
 public:
    using Ptr  = std::shared_ptr<Ergodic>;
    using UPtr = std::unique_ptr<Ergodic>;

    using ergodicGlobalPlanner = mhp_robot::robot_path_pilot::ompl_planners::ergodicGlobalPlanner;
    
    Ergodic() = default;

    bool initialize(ompl::base::SpaceInformationPtr space_information) override;

 private:
    std::shared_ptr<ergodicGlobalPlanner> _ergodic_planner;
};

}  // namespace planner_methods
}  // namespace robot_path_pilot
}  // namespace mhp_robot

#endif  // ERGODIC_H
