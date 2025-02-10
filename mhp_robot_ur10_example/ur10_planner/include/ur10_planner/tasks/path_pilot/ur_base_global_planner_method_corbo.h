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

#ifndef UR_BASE_GLOBAL_PLANNER_METHOD_CORBO_H
#define UR_BASE_GLOBAL_PLANNER_METHOD_CORBO_H

#include <mhp_planner/core/factory.h>
#include <mhp_robot/robot_path_pilot/robot_global_planner_method.h>

namespace mhp_planner
{

    class URBaseGlobalPlannerMethodCORBO : virtual public mhp_robot::robot_path_pilot::RobotGlobalPlannerMethod
    {
    public:
        using Ptr = std::shared_ptr<URBaseGlobalPlannerMethodCORBO>;
        using UPtr = std::unique_ptr<URBaseGlobalPlannerMethodCORBO>;

        URBaseGlobalPlannerMethodCORBO() = default;

        virtual Ptr getInstance() const = 0;

        virtual bool fromParameterServer(const std::string &ns) = 0;
    };

} // namespace corbo

#endif // UR_BASE_GLOBAL_PLANNER_METHOD_CORBO_H
