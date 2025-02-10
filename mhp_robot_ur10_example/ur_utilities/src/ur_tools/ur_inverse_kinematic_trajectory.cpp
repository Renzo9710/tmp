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
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
// UR
#include <mhp_robot/robot_setpoint_manager/objectives/objectives.h>
#include <mhp_robot/robot_setpoint_manager/robot_setpoint_manager.h>
#include <ur_utilities/ur_collision/ur_collision.h>
#include <ur_utilities/ur_kinematic/ur_inverse_kinematic.h>
#include <ur_utilities/ur_kinematic/ur_kinematic.h>
#include <ur_utilities/ur_misc/ur_utility.h>

bool first_joint_state = true;
std::vector<double> joint_states(6);
Eigen::VectorXd joint_states_eigen(6);
bool received_trajectory = false;
bool prepared_trajectory = false;
Eigen::MatrixXd ergodic_trajectory;
std::vector<Eigen::AngleAxisd> orientation_vector_ergodic_trajectory;
Eigen::Vector4d _poi_world;

void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
    if (first_joint_state)
    {
        first_joint_state = false;
        joint_states      = msg->position;
        joint_states_eigen.resize(joint_states.size());
        joint_states_eigen[0] = joint_states[2];
        joint_states_eigen[1] = joint_states[1];
        joint_states_eigen[2] = joint_states[0];
        joint_states_eigen[3] = joint_states[3];
        joint_states_eigen[4] = joint_states[4];
        joint_states_eigen[5] = joint_states[5];
    }
    else
    {
        joint_states = msg->position;
        joint_states_eigen.resize(joint_states.size());
        joint_states_eigen[0] = joint_states[2];
        joint_states_eigen[1] = joint_states[1];
        joint_states_eigen[2] = joint_states[0];
        joint_states_eigen[3] = joint_states[3];
        joint_states_eigen[4] = joint_states[4];
        joint_states_eigen[5] = joint_states[5];
    }
}

void traj_callback(const trajectory_msgs::JointTrajectory::ConstPtr& msg)
{
    if (!prepared_trajectory)
    {
        // Get the trajectory
        ergodic_trajectory = Eigen::MatrixXd::Zero(4, msg->points.size());
        orientation_vector_ergodic_trajectory.clear();
        orientation_vector_ergodic_trajectory.reserve(msg->points.size());
        for (int i = 0; i < msg->points.size(); i++)
        {
            for (int j = 0; j < 3; j++)
            {
                ergodic_trajectory(j, i) = msg->points[i].positions[j];
            }
            ergodic_trajectory(3, i) = msg->points[i].time_from_start.toSec();

            Eigen::Vector3d orientation_vector = _poi_world.head<3>() - ergodic_trajectory.block<3, 1>(0, i);

            Eigen::Vector3d newOrientation = Eigen::Vector3d::UnitX().cross(orientation_vector).normalized();
            double angle                   = std::acos(Eigen::Vector3d::UnitX().dot(orientation_vector.normalized()));

            Eigen::AngleAxisd orientation = Eigen::AngleAxisd(angle, newOrientation);
            orientation_vector_ergodic_trajectory.push_back(orientation);
        }
        received_trajectory = true;
    }
}

int main(int argc, char* argv[])
{
#ifndef NDEBUG
    sleep(5);
#endif
    ros::init(argc, argv, "ur_transform_erg_traj");

    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    std::vector<double> poi = nh.param("/ufomap_server_node/poi_world", std::vector<double>{1.19, 0.16, 0.33});
    _poi_world = Eigen::Vector4d(poi[0], poi[1], poi[2], 1.0);

    ros::Publisher ergodic_trajectory_joint_space_pub;
    ergodic_trajectory_joint_space_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/ergodic_trajectory_joint_space", 1);
    ros::Publisher ergodic_path_pub;
    ergodic_path_pub = nh.advertise<nav_msgs::Path>("/ergodic_path", 1);

    ros::Subscriber ergodic_trajectory_sub;
    ergodic_trajectory_sub = nh.subscribe("/ergodic_trajectory", 1, traj_callback);
    ros::Subscriber joint_state_sub;
    joint_state_sub = nh.subscribe("/ur_driver/joint_states", 1, jointStateCallback);

    // Utilities and Kinematics
    mhp_robot::robot_misc::URUtility ur_utility;
    mhp_robot::robot_kinematic::URKinematic ur_kinematic;
    mhp_robot::robot_kinematic::URInverseKinematic::UPtr ur_inverse_kinematic =
        std::make_unique<mhp_robot::robot_kinematic::URInverseKinematic>();

    // Setpoint manager --> Check validity and continuity of the trajectory
    mhp_robot::robot_set_point_manager::RobotSetPointManager setpoint_manager(
        std::make_unique<mhp_robot::robot_collision::URCollision>());

    // Set the velocity limits of the setpoint manager for continuity check
    std::vector<double> joint_limits = ur_utility.getJointVelocityLimits();
    setpoint_manager.setVelocityLimits(-std::min_element(joint_limits.begin(), joint_limits.end()).operator*(),
                                       std::min_element(joint_limits.begin(), joint_limits.end()).operator*());

    // Inv. Kinematic objective for redundant solutions
    mhp_robot::robot_set_point_manager::objectives::BaseSetpointObjective::UPtr redundancy_objective =
        std::make_unique<mhp_robot::robot_set_point_manager::objectives::LengthSetpointObjective>();

    // Init elements for the IK
    std::vector<Eigen::VectorXd> validated_targets(1);
    int n_solutions;
    std::vector<Eigen::MatrixXd> q(1);
    std::vector<double> times(1);
    validated_targets[0] = Eigen::VectorXd::Zero(6);

    // Set up rate loop
    ros::Rate loop_rate(20);
    bool first_run = true;
    trajectory_msgs::JointTrajectory target;

    // X due to rotation between camaera z and ee x
    while (ros::ok())
    {
        // Set the task point into Ik element
        if (prepared_trajectory)
        {
            // std::cout << "Resend waypoints" << std::endl;
            trajectory_msgs::JointTrajectory msg;
            msg.header.stamp = ros::Time::now();
            msg.joint_names.push_back("shoulder_pan_joint");
            msg.joint_names.push_back("shoulder_lift_joint");
            msg.joint_names.push_back("elbow_joint");
            msg.joint_names.push_back("wrist_1_joint");
            msg.joint_names.push_back("wrist_2_joint");
            msg.joint_names.push_back("wrist_3_joint");
            for (int i = 0; i < validated_targets.size(); i++)
            {
                trajectory_msgs::JointTrajectoryPoint point;
                point.positions.resize(6);
                point.positions[0]    = validated_targets[i](0);
                point.positions[1]    = validated_targets[i](1);
                point.positions[2]    = validated_targets[i](2);
                point.positions[3]    = validated_targets[i](3);
                point.positions[4]    = validated_targets[i](4);
                point.positions[5]    = validated_targets[i](5);
                point.time_from_start = ros::Duration(times[i]);
                msg.points.push_back(point);
                // std::cout<< "point.positions[0]: " << point.positions[0] << std::endl;
            }
            ergodic_trajectory_joint_space_pub.publish(msg);

            nav_msgs::Path path;
            path.header.stamp    = ros::Time::now();
            path.header.frame_id = "world";
            for (int i = 0; i < validated_targets.size(); i++)
            {
                geometry_msgs::PoseStamped pose;
                pose.pose.position.x = ergodic_trajectory(0, i);
                pose.pose.position.y = ergodic_trajectory(1, i);
                pose.pose.position.z = ergodic_trajectory(2, i);
                Eigen::Quaterniond q(orientation_vector_ergodic_trajectory[i]);
                pose.pose.orientation.x = q.x();
                pose.pose.orientation.y = q.y();
                pose.pose.orientation.z = q.z();
                pose.pose.orientation.w = q.w();
                path.poses.push_back(pose);
            }
            ergodic_path_pub.publish(path);
        }
        else
        {
            if (received_trajectory)
            {
                q                 = std::vector<Eigen::MatrixXd>(orientation_vector_ergodic_trajectory.size());
                times             = std::vector<double>(orientation_vector_ergodic_trajectory.size());
                validated_targets = std::vector<Eigen::VectorXd>(orientation_vector_ergodic_trajectory.size());

                for (int i = 0; i < orientation_vector_ergodic_trajectory.size(); i++)
                {
            
                    ur_inverse_kinematic->setTaskPoint(ergodic_trajectory.block<3, 1>(0, i), orientation_vector_ergodic_trajectory[i]);
                    q[i] = ur_inverse_kinematic->getSolutions();
                    times[i] = ergodic_trajectory(3, i);
                }
                // Set the joint space waypoints
                setpoint_manager.setJointSpaceWaypoints(q, times);
                setpoint_manager.validate();

                // Get the optimal waypoints
                if (setpoint_manager.getOptimalWaypoints(*redundancy_objective, validated_targets))
                {
                    std::cout << "Optimal waypoints found" << std::endl;
                    trajectory_msgs::JointTrajectory msg;
                    msg.header.stamp = ros::Time::now();
                    msg.joint_names.push_back("shoulder_pan_joint");
                    msg.joint_names.push_back("shoulder_lift_joint");
                    msg.joint_names.push_back("elbow_joint");
                    msg.joint_names.push_back("wrist_1_joint");
                    msg.joint_names.push_back("wrist_2_joint");
                    msg.joint_names.push_back("wrist_3_joint");
                    for (int i = 0; i < validated_targets.size(); i++)
                    {
                        trajectory_msgs::JointTrajectoryPoint point;
                        point.positions.resize(6);
                        point.positions[0]    = validated_targets[i](0);
                        point.positions[1]    = validated_targets[i](1);
                        point.positions[2]    = validated_targets[i](2);
                        point.positions[3]    = validated_targets[i](3);
                        point.positions[4]    = validated_targets[i](4);
                        point.positions[5]    = validated_targets[i](5);
                        point.time_from_start = ros::Duration(times[i]);
                        msg.points.push_back(point);
                        // std::cout<< "point.positions[0]: " << point.positions[0] << std::endl;
                    }
                    ergodic_trajectory_joint_space_pub.publish(msg);

                    nav_msgs::Path path;
                    path.header.stamp    = ros::Time::now();
                    path.header.frame_id = "world";
                    for (int i = 0; i < validated_targets.size(); i++)
                    {
                        geometry_msgs::PoseStamped pose;
                        pose.pose.position.x = ergodic_trajectory(0, i);
                        pose.pose.position.y = ergodic_trajectory(1, i);
                        pose.pose.position.z = ergodic_trajectory(2, i);
                        Eigen::Quaterniond q(orientation_vector_ergodic_trajectory[i]);
                        pose.pose.orientation.x = q.x();
                        pose.pose.orientation.y = q.y();
                        pose.pose.orientation.z = q.z();
                        pose.pose.orientation.w = q.w();
                        path.poses.push_back(pose);
                    }
                    ergodic_path_pub.publish(path);
                    received_trajectory = false;
                    prepared_trajectory = true;
                }
                else
                {
                    ROS_WARN("Optimal waypoints not found");
                    received_trajectory = false;
                }
            }
        }
        ros::spinOnce();

        loop_rate.sleep();
    }
    return 0;
}
