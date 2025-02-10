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

#include <ompl/base/Planner.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>

// often useful headers:
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/util/RandomNumbers.h>

#include <mhp_robot/robot_kinematic/robot_inverse_kinematic.h>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>

namespace mhp_robot {
namespace robot_path_pilot {
namespace ompl_planners {
class ergodicGlobalPlanner : public ompl::base::Planner
{
 public:
    using Ptr  = std::shared_ptr<ergodicGlobalPlanner>;
    using UPtr = std::unique_ptr<ergodicGlobalPlanner>;

    ergodicGlobalPlanner(const ompl::base::SpaceInformationPtr& space_information);

    virtual ~ergodicGlobalPlanner(void);

    virtual ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition& ptc);

    virtual void clear(void);
    void startDistributionNode();
    // optional, if additional setup/configuration is needed, the setup() method can be implemented
    virtual void setup(void);

    virtual void getPlannerData(ompl::base::PlannerData& data) const;

 private:
    ros::Subscriber _erg_traj_sub;
    ros::NodeHandle _nh;
    void ergTrajCallback(const trajectory_msgs::JointTrajectory::ConstPtr& msg);

    // your planner data goes here
    trajectory_msgs::JointTrajectory _erg_traj;
    bool _erg_traj_received_first = false;
    bool _erg_traj_set            = false;
    Eigen::VectorXd _time;
};
}  // namespace ompl_planners
}  // namespace robot_path_pilot
}  // namespace mhp_robot