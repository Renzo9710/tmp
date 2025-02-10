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


#include <mhp_robot/robot_path_pilot/planner_methods/ergodic.h>

namespace mhp_robot {
namespace robot_path_pilot {
namespace planner_methods {

bool Ergodic::initialize(ompl::base::SpaceInformationPtr space_information)
{
    _space_information = space_information;
    std::cout<<"Initializing Ergodic planner"<<std::endl;
    _planner           = std::make_shared<ergodicGlobalPlanner>(_space_information);

    // do configuration stuff here if any

    return true;
}

}  // namespace planner_methods
}  // namespace robot_path_pilot
}  // namespace mhp_robot
