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
#include <ur10_planner/tasks/path_pilot/ur_global_planner_corbo.h>
#include <ur10_planner/tasks/path_pilot/ur_path_pilot_corbo.h>

namespace mhp_planner
{
    URBasePathPilotCORBO::Ptr URPathPilotCORBO::getInstance() const { return std::make_shared<URPathPilotCORBO>(); }

    bool URPathPilotCORBO::fromParameterServer(const std::string &ns)
    {
        // get node handle
        ros::NodeHandle nh;

        // get global planner method
        URGlobalPlannerCORBO::Ptr tmp = std::make_shared<URGlobalPlannerCORBO>();
        if (!tmp->fromParameterServer(ns + "/global_planner"))
        {
            PRINT_ERROR("URTask: Could not read global planner method.");
            return false;
        }
        _global_planner = tmp;

        // get parameters
        if (!nh.getParam(ns + "/direct_motion_r", _direct_motion_r))
        {
            PRINT_ERROR("URTask: Could not read parameter direct_motion_r.");
            return false;
        };

        if (!nh.getParam(ns + "/replan_r", _replan_r))
        {
            PRINT_ERROR("URTask: Could not read parameter replan_r.");
            return false;
        };

        if (!nh.getParam(ns + "/tau", _tau))
        {
            PRINT_ERROR("URTask: Could not read parameter tau.");
            return false;
        };

        if (!nh.getParam(ns + "/lambda", _lambda))
        {
            PRINT_ERROR("URTask: Could not read parameter lambda.");
            return false;
        };

        if (!nh.getParam(ns + "/min_obstacle_collision", _min_obstacle_collision))
        {
            PRINT_ERROR("URTask: Could not read parameter min_obstacle_collision.");
            return false;
        };

        if (!nh.getParam(ns + "/backstepping_resolution", _backstepping_resolution))
        {
            PRINT_ERROR("URTask: Could not read parameter backstepping_resolution.");
            return false;
        };

        if (!nh.getParam(ns + "/K", _K))
        {
            PRINT_ERROR("URTask: Could not read parameter K.");
            return false;
        };

        if (!nh.getParam(ns + "/dt", _dt))
        {
            PRINT_ERROR("URTask: Could not read parameter dt.");
            return false;
        };

        if (!nh.getParam(ns + "/visualize", _visualize))
        {
            PRINT_ERROR("URTask: Could not read parameter visualize.");
            return false;
        };

        return true;
    }
} // namespace mhp_planner
