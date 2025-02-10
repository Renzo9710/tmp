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

#ifndef ROBOT_PATH_PILOT_H
#define ROBOT_PATH_PILOT_H

#include <nav_msgs/Path.h>
#include <ompl/geometric/PathGeometric.h>
#include <mhp_robot/robot_collision/robot_collision.h>
#include <mhp_robot/robot_kinematic/robot_kinematic.h>
#include <mhp_robot/robot_misc/robot_utility.h>
#include <mhp_robot/robot_obstacle/obstacle_list.h>
#include <mhp_robot/robot_path_pilot/robot_global_planner.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Dense>
#include <memory>
#include <vector>


namespace mhp_robot {
namespace robot_path_pilot {

class RobotPathPilot
{
 public:
    using Ptr  = std::shared_ptr<RobotPathPilot>;
    using UPtr = std::unique_ptr<RobotPathPilot>;

    RobotPathPilot() = default;

    RobotPathPilot(const RobotPathPilot&) = delete;
    RobotPathPilot(RobotPathPilot&&)      = delete;
    RobotPathPilot& operator=(const RobotPathPilot&) = delete;
    RobotPathPilot& operator=(RobotPathPilot&&) = delete;
    ~RobotPathPilot()                           = default;

    struct SubReference
    {
        Eigen::VectorXd configuration;                    // Joint configuration
        Eigen::VectorXd pose = Eigen::VectorXd::Zero(7);  // Corresponding task space pose

        // Information about the linear path segment on which the center is currently located
        int start_idx = 0;
        int end_idx   = 1;
    };

    bool initialize(robot_collision::RobotCollision::UPtr robot_collision, robot_kinematic::RobotKinematic::UPtr robot_kinematic);

    virtual SubReference sampleSubReference(const Eigen::Ref<const Eigen::VectorXd>& q0, const Eigen::Ref<const Eigen::MatrixXd>& prediction,
                                            const Eigen::Ref<const Eigen::VectorXd>& xref);

    void publishPath();
    void publishSubReference(const SubReference& tr);
    void shutdown();

 protected:
    using ObstacleList   = robot_obstacle::ObstacleList;
    using RobotKinematic = robot_kinematic::RobotKinematic;
    using RobotCollision = robot_collision::RobotCollision;
    using RobotUtility   = robot_misc::RobotUtility;

    ObstacleList _obstacle_manager;
    RobotKinematic::UPtr _robot_kinematic;
    RobotCollision::UPtr _robot_collision;

    ros::Publisher _region_pub;
    ros::Publisher _prediction_pub;

    std::shared_ptr<ompl::geometric::PathGeometric> _path_ompl;
    std::vector<Eigen::VectorXd> _path_eigen;
    SubReference _tr;
    Eigen::VectorXd _xref_old;

    RobotGlobalPlanner::Ptr _global_planner;
    Eigen::VectorXd _path_goal;

    bool _initialized               = false;
    bool _visualize                 = true;
    bool _path_requested            = false;
    bool _first_run                 = true;
    double _direct_motion_r         = 0.1;
    double _replan_r                = 0.08;
    double _lambda                  = 0.4;
    double _tau                     = 0.02;
    double _dt                      = 0.1;
    double _backstepping_resolution = 0.01;
    double _min_obstacle_collision  = 0.05;
    int _dim                        = 0;
    int _n_temporary_static         = 0;
    int _n_static                   = 0;
    int _K                          = -5;

    Eigen::VectorXd _time;
    double _start_time = 0.0;
    double _slow_down_factor = 1.2;
    SubReference _tr_old;
    // TODO(renz): Transfer this options to proto to change them for each run individually
    bool _resend_last = false; 
    bool _ergodic_time_based = false;
    bool _ergodic_path_based = true;
    
    void updatePose(SubReference& tr);
    bool checkScene();
    bool checkGoal(const Eigen::Ref<const Eigen::VectorXd>& xref) const;
    void scaleAndCheckReference(double scale, SubReference& tr);
    bool advanceOnPath(double scale, SubReference& tr) const;

    virtual bool validateSubReference(const SubReference& tr);
};

}  // namespace robot_path_pilot
}  // namespace mhp_robot

#endif  // ROBOT_PATH_PILOT_H
