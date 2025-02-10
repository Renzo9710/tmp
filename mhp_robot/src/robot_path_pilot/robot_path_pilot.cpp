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

#include <mhp_robot/robot_path_pilot/robot_path_pilot.h>

namespace mhp_robot {
namespace robot_path_pilot {

bool RobotPathPilot::initialize(RobotCollision::UPtr robot_collision, robot_kinematic::RobotKinematic::UPtr robot_kinematic)
{
    if (_initialized) return true;

    _robot_collision = std::move(robot_collision);
    _robot_kinematic = std::move(robot_kinematic);

    // visualization
    ros::NodeHandle n("~");

    std::string region_marker_topic = "terminal_center";
    std::string path_marker_topic   = "global_path";

    _prediction_pub = n.advertise<nav_msgs::Path>(path_marker_topic, 1);
    _region_pub     = n.advertise<visualization_msgs::Marker>(region_marker_topic, 1);

    _dim = _robot_kinematic->getRobotUtility().getJointsCount();

    // Define dimensions of VectorXd
    _tr.configuration = Eigen::VectorXd::Zero(_dim);
    _tr.pose          = Eigen::VectorXd::Zero(7);

    if (!_global_planner)
    {
        ROS_ERROR("RobotPathPilot: No global planner instantiated.");
        return false;
    }

    if (!_global_planner->initialize(_robot_collision->createUniqueInstance(), _robot_kinematic->createUniqueInstance())) return false;

    _initialized = true;

    return true;
}

RobotPathPilot::SubReference RobotPathPilot::sampleSubReference(const Eigen::Ref<const Eigen::VectorXd>& q0,
                                                                const Eigen::Ref<const Eigen::MatrixXd>& prediction,
                                                                const Eigen::Ref<const Eigen::VectorXd>& xref)
{
    // std::cout<<"RobotPathPilot: Sample Sub Reference"<<std::endl;
    // Check if scene (clear) or xref (query) has changed
    _global_planner->update();

    bool scene_changed = checkScene();
    bool query_changed = checkGoal(xref);  //|| prediction->getTime().empty()
    double scale       = 0.0;
    int n              = prediction.cols();

    if (_first_run || query_changed || scene_changed || (_path_eigen.empty() && !_path_requested))
    {
        if (scene_changed)
        {
            // clear planner data
            _global_planner->clearPlan();
            scene_changed = false;
            // std::cout << "Scene Changed" << std::endl;
        }

        if (query_changed)
        {
            // clear/update query
            _global_planner->clearQuery();
            query_changed = false;
            // std::cout << "Query Changed" << std::endl;
        }
        if (_first_run) _first_run = false;

        // Request to solve P(q0 -> xref)
        _global_planner->setStartAndGoalState(q0, xref);
        _xref_old = xref;
        _global_planner->requestPath();

        std::cout << "Request Path" << std::endl;

        _path_requested = true;
    }

    std::shared_ptr<ompl::geometric::PathGeometric> path_ompl;
    std::vector<Eigen::VectorXd> path_eigen;

    bool new_path = _global_planner->getPath(path_eigen, path_ompl);
    if (new_path && path_ompl && !path_eigen.empty())
    {
        _path_eigen = path_eigen;
        _path_ompl  = std::make_shared<ompl::geometric::PathGeometric>(*path_ompl);
    }
    if (_global_planner->getName() != "Ergodic global planner")
    {

        if (_path_eigen.empty())
        {
            if (!_path_requested)
            {
                // This case should not occur
                ROS_ERROR("This should not occur");
            }
            else
            {
                // Already requested
                if (new_path)
                {
                    // new P now available -> start extract tr from P

                    std::cout << "new P already available -> start extract tr from P" << std::endl;

                    _path_requested = false;

                    // Reset Reference
                    _tr.configuration = q0;
                    _tr.start_idx     = 0;
                    _tr.end_idx       = 1;

                    scale = std::max((_lambda * n * _dt), 0.0);
                    // scale = (_lambda * n * _dt);
                    scaleAndCheckReference(scale, _tr);
                }
                else
                {
                    std::cout << "tr = q0" << std::endl;

                    // tr = q0
                    _tr.configuration = q0;
                    _tr.start_idx     = 0;
                    _tr.end_idx       = 1;
                }
            }
        }
        else
        {
            if (_path_requested)
            {
                // A new path was requested
                if (new_path)
                {
                    std::cout << "new P now available -> extract tr from P" << std::endl;

                    // new P now available -> extract tr from P
                    _path_requested = false;

                    // Reset Reference
                    _tr.configuration = q0;
                    _tr.start_idx     = 0;
                    _tr.end_idx       = 1;

                    scale = std::max((_lambda * n * _dt), 0.0);
                    std::cout << "Scale: " << scale << std::endl;
                    // scale = (_lambda * n * _dt);
                    scaleAndCheckReference(scale, _tr);
                }
                else
                {
                    std::cout << "continue to extract tr from current (old) P" << std::endl;

                    // continue to extract tr from current (old) P
                    int i = 0;
                    for (i = 0; i < n; ++i)
                    {
                        double e = (prediction.col(n - i - 1) - _tr.configuration).norm();
                        if (e > _tau) break;
                    }
                    int K = _K + i;  // Maybe use more defensive (bigger) K during "blind-mode"

                    scale = std::max((_lambda * K * _dt), 0.0);
                    // scale = (_lambda * K * _dt);
                    scaleAndCheckReference(scale, _tr);
                }
            }
            else
            {
                std::cout << "continue to extract tr from current P" << std::endl;

                // continue to extract tr from current P
                int i = 0;
                for (i = 0; i < n; ++i)
                {
                    double e = (prediction.col(n - i - 1) - _tr.configuration).norm();
                    if (e > _tau) break;
                }
                int K = _K + i;

                scale = std::max((_lambda * K * _dt), 0.0);
                // scale = (_lambda * K * _dt);

                // std::cout << "Scale: " << scale << std::endl;

                scaleAndCheckReference(scale, _tr);
            }
        }
        updatePose(_tr);
        std::cout << "Scale " << scale << std::endl;

        // Check if q0 and xref are close to each other
        if ((q0 - xref).norm() <= _direct_motion_r)
        {
            SubReference tr_direct_motion;

            // Direct motion
            tr_direct_motion.configuration = xref;
            updatePose(tr_direct_motion);

            std::cout << "Direct Motion" << std::endl;

            return tr_direct_motion;
        }

        return _tr;
    }
    else  // Ergodic global planner
    {
        if (_time.size() == 0 && !_path_eigen.empty())
        {
            double dt = 0.0;
            if (!_global_planner->getDeltaTime(dt)) ROS_ERROR("RobotPathPilot: Could not get delta time from ergodic global planner");
            // _time.resize(_path_eigen.size());
            _time       = Eigen::VectorXd::LinSpaced(_path_eigen.size(), 0.0, dt * _path_eigen.size() * _slow_down_factor);
            _start_time = ros::Time::now().toSec();
        }

        if (_path_eigen.empty())
        {
            // No path available
            if (_resend_last)
            {
                if ((ros::Time::now().toSec() - _start_time) >= 3.0 || ((q0 - _path_eigen.front()).norm() < 0.3))
                {
                    _tr.configuration = xref;
                    updatePose(_tr);
                    _tr_old      = _tr;
                }
                _tr = _tr_old;
                updatePose(_tr);
            }
            else
            {
                _tr.configuration = q0;
                updatePose(_tr);
            }
        }
        else
        {
            // Trajectory available
            if (_ergodic_time_based && !_ergodic_path_based)
            {
                // Get current time
                double t = ros::Time::now().toSec() - _start_time;
                // Find closest time in path
                if (t > _time(0))
                {
                    _tr.configuration = _path_eigen.front();
                    updatePose(_tr);

                    _path_eigen.erase(_path_eigen.begin());

                    Eigen::VectorXd time_tmp = _time.tail(_time.size() - 1);
                    _time.resize(time_tmp.size());
                    _time = time_tmp;
                    if (_path_eigen.empty())
                    {
                        _tr_old      = _tr;
                        _resend_last = true;
                    }
                }
            }
            else if (_ergodic_path_based && !_ergodic_time_based)
            {
                // Check if we reached current goal with a tolerance
                if ((q0 - _path_eigen.front()).norm() < 0.3)
                {
                    _path_eigen.erase(_path_eigen.begin());
                    _start_time = ros::Time::now().toSec();

                    _tr.configuration = _path_eigen.front();
                    updatePose(_tr);
                    if (_path_eigen.empty())
                    {
                        _tr_old      = _tr;
                        _resend_last = true;
                    }
                }
                else if ((ros::Time::now().toSec() - _start_time) >= 1.0 && !((q0 - _path_eigen.front()).norm() < 0.3))
                {
                    _path_eigen.erase(_path_eigen.begin());
                    _start_time = ros::Time::now().toSec();

                    _tr.configuration = _path_eigen.front();
                    updatePose(_tr);
                    if (_path_eigen.empty())
                    {
                        _tr_old      = _tr;
                        _resend_last = true;
                    }
                }
                else
                {
                    _tr.configuration = _path_eigen.front();
                    updatePose(_tr);
                }
            }
            else
            {
                ROS_WARN("No Ergodic trajectory/path sampling mode selected");
            }
        }
        return _tr;
    }
}

void RobotPathPilot::publishPath()
{
    if (_path_eigen.empty() || !_visualize) return;

    nav_msgs::Path path_msg;
    Eigen::Matrix<double, 4, 4> ee_transformation;

    // Copy path to safely modify it for visualization
    ompl::geometric::PathGeometric path = *_path_ompl;
    // ROS_WARN_STREAM("Path size: " << path.getStateCount());

    if (_global_planner->getName() != "Ergodic global planner")
    {
        path.interpolate(20);
    }
    // ROS_WARN_STREAM("Path size: " << path.getStateCount());

    int n = path.getStateCount();

    for (int i = 0; i < n; ++i)
    {
        Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>()->values, _dim);
        ee_transformation = _robot_kinematic->getEndEffectorMatrix(q);

        geometry_msgs::PoseStamped pose_stamped_msg;
        robot_misc::Common::poseEigenToMsg(ee_transformation, pose_stamped_msg.pose);
        path_msg.poses.push_back(pose_stamped_msg);
    }

    path_msg.header.stamp    = ros::Time::now();
    path_msg.header.frame_id = "world";

    _prediction_pub.publish(path_msg);
}

void RobotPathPilot::publishSubReference(const SubReference& tr)
{
    if (!_visualize) return;

    visualization_msgs::Marker center;

    center.header.frame_id = "world";
    center.action          = visualization_msgs::Marker::ADD;
    center.lifetime        = ros::Duration(10.1);

    center.scale.x = 0.05;
    center.scale.y = 0.05;
    center.scale.z = 0.05;

    center.color.r = 0.0f;
    center.color.g = 0.0f;
    center.color.b = 1.0f;
    center.color.a = 1.0;

    center.ns = "Sub Reference";

    center.header.stamp    = ros::Time::now();
    center.type            = visualization_msgs::Marker::SPHERE;
    center.id              = 0;
    center.pose.position.x = tr.pose(0);
    center.pose.position.y = tr.pose(1);
    center.pose.position.z = tr.pose(2);

    center.pose.orientation.w = tr.pose(3);
    center.pose.orientation.x = tr.pose(4);
    center.pose.orientation.y = tr.pose(5);
    center.pose.orientation.z = tr.pose(6);

    _region_pub.publish(center);
}

void RobotPathPilot::shutdown() { _global_planner->shutdown(); }

void RobotPathPilot::updatePose(SubReference& tr)
{
    const Eigen::Ref<const Eigen::Matrix<double, 4, 4>> ee_transformation = _robot_kinematic->getEndEffectorMatrix(tr.configuration);

    Eigen::Quaterniond q;
    q = ee_transformation.block<3, 3>(0, 0);
    q.normalize();

    tr.pose.head<3>() = ee_transformation.block<3, 1>(0, 3);
    tr.pose.tail<4>() << q.w(), q.x(), q.y(), q.z();
}

bool RobotPathPilot::checkScene()
{
    // TODO(kraemer) check state of the scene based on obstacle ids and not on pure number

    bool changed = false;
    if (_n_temporary_static != _obstacle_manager.temporaryStaticObstacleCount())
    {
        changed             = true;
        _n_temporary_static = _obstacle_manager.temporaryStaticObstacleCount();
    }

    if (_n_static != _obstacle_manager.staticObstacleCount())
    {
        changed   = true;
        _n_static = _obstacle_manager.staticObstacleCount();
    }
    return changed;
}

bool RobotPathPilot::checkGoal(const Eigen::Ref<const Eigen::VectorXd>& xref) const
{
    return !_first_run ? (xref - _xref_old).norm() > _replan_r : false;
}

void RobotPathPilot::scaleAndCheckReference(double scale, SubReference& tr)
{
    if (scale >= 0.0) advanceOnPath(scale, tr);

    // Check validity
    while (!validateSubReference(tr))
    {
        // Sub reference was not valid, we have to reduce the step
        if (advanceOnPath(-_backstepping_resolution, tr))
        {
            ROS_WARN("We are at the end/front and cannot go any further");
        }
    }
}

bool RobotPathPilot::advanceOnPath(double scale, SubReference& tr) const
{
    Eigen::VectorXd p1 = _path_eigen[tr.start_idx];
    Eigen::VectorXd p2 = _path_eigen[tr.end_idx];
    Eigen::VectorXd s  = p2 - p1;
    double l           = s.norm();

    if (scale > 0)
    {
        // We go in positive direction

        // Progress on the current segment
        double d = (tr.configuration - p1).norm();
        while (d + scale > l)
        {
            std::cout << "d: " << d << " and l: " << l << " and scale: " << scale << " at start idx: " << tr.start_idx
                      << " and end idex: " << tr.end_idx << std::endl;

            if (tr.end_idx == _path_eigen.size() - 1)
            {
                // We are already on the last segment
                tr.configuration = _path_eigen.back();
                return true;
            }

            tr.start_idx++;
            tr.end_idx++;

            p1 = _path_eigen[tr.start_idx];
            p2 = _path_eigen[tr.end_idx];

            scale = d + scale - l;

            s = p2 - p1;
            l = s.norm();
            d = 0.0;  // We start at the beginning of the new segment
        }
        tr.configuration = p1 + s * (d + scale) / l;
    }
    else
    {
        // We go in negative direction

        // Progress on the current segment
        double d = (tr.configuration - p1).norm();

        while (d + scale < 0.0)
        {
            if (tr.start_idx == 0)
            {
                // We are already on the first segment
                tr.configuration = _path_eigen.front();
                return true;
            }

            tr.start_idx--;
            tr.end_idx--;

            p1 = _path_eigen[tr.start_idx];
            p2 = _path_eigen[tr.end_idx];

            scale = d + scale;

            s = p2 - p1;
            l = s.norm();
            d = l;  // We start at the end of the new segment
        }
        tr.configuration = p1 + s * (d + scale) / l;
    }

    return false;
}

bool RobotPathPilot::validateSubReference(const SubReference& tr)
{
    _robot_collision->setJointState(tr.configuration);

    for (int i = 0; i < (int)_obstacle_manager._dynamic_obstacles.size(); ++i)
    {
        if (_robot_collision->getMinObstacleDistance(_obstacle_manager._dynamic_obstacles[i]) < _min_obstacle_collision) return false;
    }

    for (int i = 0; i < (int)_obstacle_manager._humans.size(); ++i)
    {
        Eigen::VectorXd d(robot_misc::Human::_body_parts_size);
        _robot_collision->getMinHumanDistance(_obstacle_manager._humans[i], d);

        if (d.minCoeff() < _min_obstacle_collision) return false;
    }

    return true;
}

}  // namespace robot_path_pilot
}  // namespace mhp_robot
