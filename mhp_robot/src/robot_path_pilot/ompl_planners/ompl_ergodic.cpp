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

#include <mhp_robot/robot_path_pilot/ompl_planners/ompl_ergodic.h>
#include <cstdlib>
#include <thread>
namespace mhp_robot {
namespace robot_path_pilot {
namespace ompl_planners {
ergodicGlobalPlanner::ergodicGlobalPlanner(const ompl::base::SpaceInformationPtr& space_information)
    : ompl::base::Planner(space_information, "Ergodic global planner")
{
    // set the name of the planner
    // std::thread(&ergodicGlobalPlanner::startDistributionNode, this).detach();
    _erg_traj_sub = _nh.subscribe("/ergodic_trajectory_joint_space", 10, &ergodicGlobalPlanner::ergTrajCallback, this);
}
void ergodicGlobalPlanner::startDistributionNode()
{
    ROS_WARN("Start ergodic reference node manually.");
}
ergodicGlobalPlanner::~ergodicGlobalPlanner(void)
{
    ROS_WARN("Kill ergodic reference node manually.");
    // free any resources allocated here if required
}
void ergodicGlobalPlanner::ergTrajCallback(const trajectory_msgs::JointTrajectory::ConstPtr& msg)
{
    // process the ergodic trajectory (in joint space) here fro later "solving"
    _erg_traj          = *msg;
    _erg_traj_received_first = true;
    ROS_INFO_ONCE("Received ergodic trajectory");
}
ompl::base::PlannerStatus ergodicGlobalPlanner::solve(const ompl::base::PlannerTerminationCondition& ptc)
{   
    checkValidity();
    if (_erg_traj_received_first && !_erg_traj_set)
    {

        ROS_INFO("Collect ergodic trajectory (presolved!!!)");

        // Generate path    
        auto p(std::make_shared<ompl::geometric::PathGeometric>(si_));

        // Fill Path with states
        _time.resize(_erg_traj.points.size());
        for (int i = 0; i < _erg_traj.points.size(); i++)
        {
            ompl::base::State* state = si_->allocState();
            for (int j = 0; j < 6; j++)
            {
                state->as<ompl::base::RealVectorStateSpace::StateType>()->values[j] = _erg_traj.points[i].positions[j];
            }
            p->append(state);
            _time(i) = _erg_traj.points[i].time_from_start.toSec();
        }

        // Add path to solution (no further checks since already approved by ergodic planner)
        pdef_->addSolutionPath(p, true, _time(_erg_traj.points.size()-1)/ _time.size(), getName()); 
        //ATTENTION: Misuse the double value for the difference between path and reference for the delta t of the trajectory steps!!!!

        // p->print(std::cout);
        _erg_traj_received_first = false;
        _erg_traj_set            = true;
        return ompl::base::PlannerStatus::APPROXIMATE_SOLUTION;
    }
    else
    {
        ROS_WARN_ONCE("No ergodic trajectory received yet");
        return ompl::base::PlannerStatus::TIMEOUT;
    }
}
void ergodicGlobalPlanner::clear(void)
{
    // clear data structures here
}

void ergodicGlobalPlanner::setup(void)
{
    ROS_INFO_ONCE("Setting up ergodic global planner");
}

void ergodicGlobalPlanner::getPlannerData(ompl::base::PlannerData& data) const
{
    // fill data with the states and edges that were created
}

}  // namespace ompl_planners
}  // namespace robot_path_pilot
}  // namespace mhp_robot
