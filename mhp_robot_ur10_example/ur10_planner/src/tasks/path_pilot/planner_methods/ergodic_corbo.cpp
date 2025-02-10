/*********************************************************************
 *
 *  Software License Agreement
 *
 *  Copyright (c) 2023,
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
#include <ur10_planner/tasks/path_pilot/planner_methods/ergodic_corbo.h>

namespace mhp_planner
{

    URBaseGlobalPlannerMethodCORBO::Ptr ErgodicCORBO::getInstance() const { return std::make_shared<ErgodicCORBO>(); }

    bool ErgodicCORBO::fromParameterServer(const std::string &ns)
    {
        // Nothing to do here just for the sake of completeness
        return true;
    }

} // namespace corbo
