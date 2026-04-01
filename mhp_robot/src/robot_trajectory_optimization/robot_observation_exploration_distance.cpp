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
 *  Maintainer(s)/Modifier(s):
 *********************************************************************/

#include <mhp_robot/robot_trajectory_optimization/robot_observation_exploration_distance.h>

namespace mhp_robot{
namespace robot_trajectory_optimization {
bool RobotObservationExplorationDistance::initialize(robot_kinematic::RobotKinematic::UPtr robot_kinematic)
{
    if (_initialized) return true;

    // call base class initialize
    if (!RobotObservationExploration::initialize(std::move(robot_kinematic))) return false;

    // get poi
    ros::NodeHandle nh;
    std::vector<double> poi = nh.param("/ufomap_server_node/poi_world", std::vector<double>{1.19, 0.16, 0.33});

    _poi_info_gain = Eigen::Vector3d(poi[0], poi[1], poi[2]);

    // get hand com
    _hand_com_sub = nh.subscribe("/hand_com", 1, &RobotObservationExplorationDistance::handComCallback, this);
    _overall_robot_view_distance = _robot_reach + _robot_occlusion_potential->getOptimalViewDistance();
    _initialized = true;
    return true;
}

double RobotObservationExplorationDistance::computePotentials(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k)
{
    double potential = 0.0;

    if (_com_initialized)
    {
        // get distance between hand com and poi
        if (k == 0)
        {
            double dist              = (_hand_com - _poi_info_gain).norm();
            double dist_hum_to_robot = (_hand_com - _robot_base).norm() - _overall_robot_view_distance;
            if (!isnan(dist) && !isnan(dist_hum_to_robot))
            {
                _tracking_mode_close_enough = dist <= _switch_distance_human_object && dist_hum_to_robot <= 0;
                // std::cout << "Distance between Hand Com and PoI: " << dist << " for hand com " << _hand_com.transpose() << std::endl;
            }
            else
            {
                ROS_WARN("RobotObservationExplorationDistance: Distance between hand com and poi is nan. Stay in tracking mode.");
                _tracking_mode_close_enough = true;
            }
        }

        // use _switch_distance to switch between exploration and observation
        if (_tracking_mode_close_enough)  // human close to object and in robot workspace
        {
            if (k == 0) _tracking_mode = true;
            if (_robot_occlusion_potential)
            {
                if (k == 0) ROS_WARN_THROTTLE(1,"RobotObservationExplorationDistance: Using occlusion potential.");
                potential += _robot_occlusion_potential->computePotentials(k, x_k);
            }
        }
        else
        {
            if (k == 0) _tracking_mode = false;
            if (_robot_information_gain)
            {
                if (k == 0) ROS_WARN_THROTTLE(1,"RobotObservationExplorationDistance: Using information gain.");
                potential += _robot_information_gain->computeCost(k, x_k);
            }
        }
    }
    else
    {
        if (k == 0) _tracking_mode = false;
        ROS_WARN_ONCE("RobotObservationExplorationDistance: Hand com not initialized.");
    }

    auto it = _costs_for_k.find(k);
    if (it != _costs_for_k.end())
    {
        it->second = potential;
    }
    else
    {
        _costs_for_k.insert(std::make_pair(k, potential));
    }

    return potential;
}

void RobotObservationExplorationDistance::handComCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    _hand_com = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

    if (!_com_initialized)
    {
        _com_initialized = true;
    }
}

}  // namespace robot_trajectory_optimization
}  // namespace mhp_robot
