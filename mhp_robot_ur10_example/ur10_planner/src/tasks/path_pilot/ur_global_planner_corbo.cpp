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
#include <ur10_planner/tasks/path_pilot/ur_base_global_planner_method_corbo.h>
#include <ur10_planner/tasks/path_pilot/ur_global_planner_corbo.h>

namespace mhp_planner
{
    bool URGlobalPlannerCORBO::fromParameterServer(const std::string &ns)
    {
        // get node handle
        ros::NodeHandle nh;

        // global planner method
        // construct object
        std::string type;
        if (!nh.getParam(ns + "/global_planner_method", type))
        {
            PRINT_ERROR("URGlobalPlannerCORBO: Could not read parameter global_planner_method.");
            return false;
        }
        URBaseGlobalPlannerMethodCORBO::Ptr global_planner_method = Factory<URBaseGlobalPlannerMethodCORBO>::instance().create(type);
        global_planner_method->fromParameterServer(ns + "/global_planner_method");
        _global_planner_method = global_planner_method;

        // get parameters
        if (!nh.getParam(ns + "/min_self_collision", _min_self_collision))
        {
            PRINT_ERROR("URTask: Could not read parameter min_self_collision.");
            return false;
        };

        if (!nh.getParam(ns + "/min_obstacle_collision", _min_obstacle_collision))
        {
            PRINT_ERROR("URTask: Could not read parameter min_obstacle_collision.");
            return false;
        };

        if (!nh.getParam(ns + "/min_ground_collision", _min_ground_collision))
        {
            PRINT_ERROR("URTask: Could not read parameter min_ground_collision.");
            return false;
        };

        if (!nh.getParam(ns + "/min_roof_collision", _min_roof_collision))
        {
            PRINT_ERROR("URTask: Could not read parameter min_roof_collision.");
            return false;
        };

        if (!nh.getParam(ns + "/min_plane_collision", _min_plane_collision))
        {
            PRINT_ERROR("URTask: Could not read parameter min_plane_collision.");
            return false;
        };

        if (!nh.getParam(ns + "/validity_resolution", _validity_resolution))
        {
            PRINT_ERROR("URTask: Could not read parameter validity_resolution.");
            return false;
        };

        if (!nh.getParam(ns + "/solve_time", _solve_time))
        {
            PRINT_ERROR("URTask: Could not read parameter solve_time.");
            return false;
        };

        if (!nh.getParam(ns + "/max_plan_time", _max_plan_time))
        {
            PRINT_ERROR("URTask: Could not read parameter max_plan_time.");
            return false;
        };

        if (!nh.getParam(ns + "/max_path_length", _max_path_length))
        {
            PRINT_ERROR("URTask: Could not read parameter max_path_length.");
            return false;
        };

        if (!nh.getParam(ns + "/simplify", _simplify))
        {
            PRINT_ERROR("URTask: Could not read parameter simplify.");
            return false;
        };

        if (!nh.getParam(ns + "/simplify_time", _simplify_time))
        {
            PRINT_ERROR("URTask: Could not read parameter simplify_time.");
            return false;
        };

        std::string path_objective;
        if (!nh.getParam(ns + "/path_length_objective", path_objective))
        {
            if (path_objective == "TaskSpace")
            {
                setPathObjective(TASKSPACE);
            }
            else if (path_objective == "JointSpace")
            {
                setPathObjective(JOINTSPACE);
            }
            else
            {
                setPathObjective(NONE);
            }
        }

        return true;
    }

} // namespace mhp_planner
