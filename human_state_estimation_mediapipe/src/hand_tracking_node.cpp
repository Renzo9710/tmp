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

#include <ros/ros.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

void LookAtQuat(Eigen::Ref<Eigen::Vector3d> start_pos, Eigen::Ref<Eigen::Vector3d> target_pos, Eigen::Ref<Eigen::Vector3d> up_vector,
                Eigen::Quaterniond& quaternion)
{
    // get normalized direction vector from camera to center of mass
    Eigen::Vector3d direction_vector = (target_pos - start_pos).normalized();

    // check if parallel with direction vector and change up vector
    if (up_vector.dot(direction_vector) == 1)
    {
        // get x axis
        Eigen::Vector3d x_axis = Eigen::Vector3d(1, 0, 0);
        // get up vector
        up_vector = x_axis.cross(direction_vector);
        ROS_ERROR("LookAtQuat: Parallel");
    }

    // get rotation basis in Eigen matrix
    Eigen::Matrix3d rotation_basis = Eigen::Matrix3d::Zero();
    rotation_basis << -direction_vector, up_vector.cross(-direction_vector).normalized(),
        (-direction_vector).cross(up_vector.cross(-direction_vector).normalized()).normalized();

    // get quaternion from lookat
    quaternion = Eigen::Quaterniond(rotation_basis);
}

int main(int argc, char** argv)
{
#ifndef NDEBUG
    sleep(5);
#endif
    // Initialize the ROS node
    ros::init(argc, argv, "hand_tracking_node");
    ros::NodeHandle nh;

    std::string _frame_name_camera;
    nh.param("/depth_camera_frame", _frame_name_camera, std::string("camera_3d_depth_camera_link"));

    std::cout << "Frame name camera Tracking Node: " << _frame_name_camera << std::endl;
    tf::TransformListener listener;
    tf::StampedTransform transform;
    ros::Publisher com_pub    = nh.advertise<geometry_msgs::PoseStamped>("com", 1);
    ros::Publisher target_pub = nh.advertise<geometry_msgs::PoseStamped>("target", 1);

    const int number_of_keypoints = 21;
    const int number_of_hands     = 2;
    bool send_new_target          = false;

    Eigen::Matrix<double, 4, number_of_keypoints * number_of_hands> coordinates;  // x,y,z,weight
    std::string frame;

    double time_difference;
    const double max_time_difference = 0.2;

    geometry_msgs::PoseStamped com_hand;
    com_hand.header.frame_id = "world";

    geometry_msgs::PoseStamped target;
    target.header.frame_id = "world";

    if (!listener.waitForTransform("world", _frame_name_camera, ros::Time(0), ros::Duration(10.0)))
    {
        ROS_ERROR("HandTracking Node: Could not get transform from world to camera frame");
        return -1;
    }

    ros::Rate loop_rate(30);

    while (ros::ok())
    {
        // Loop through each hand
        for (int j = 0; j < number_of_hands; j++)
        {
            // Loop through each keypoint of the hand
            for (int i = 0; i < number_of_keypoints; i++)
            {
                try
                {
                    // Determine the frame name based on the hand and keypoint index
                    if (j == 0)
                    {
                        frame = "right_hand_" + std::to_string(i);
                    }
                    else
                    {
                        frame = "left_hand_" + std::to_string(i);
                    }

                    // Check if the transform is available
                    if (listener.canTransform("world", frame, ros::Time(0)))
                    {
                        // Lookup the transform from the world frame to the hand keypoint frame
                        listener.lookupTransform("world", frame, ros::Time(0), transform);

                        // Check how long ago the transform was
                        if ((ros::Time::now() - transform.stamp_).toSec() > max_time_difference)
                            send_new_target = false;
                        else
                            send_new_target = true;
                    }

                    // Calculate the time difference between the current time and the transform's timestamp
                    time_difference = ros::Time::now().toSec() - transform.stamp_.toSec();

                    // Store the coordinates of the keypoint
                    coordinates(0, i + j * number_of_keypoints) = transform.getOrigin().x();
                    coordinates(1, i + j * number_of_keypoints) = transform.getOrigin().y();
                    coordinates(2, i + j * number_of_keypoints) = transform.getOrigin().z();

                    // Set the weight of the keypoint based on the time difference
                    if (time_difference >= max_time_difference)
                    {
                        coordinates(3, i + j * number_of_keypoints) = 0.0;
                    }
                    else
                    {
                        coordinates(3, i + j * number_of_keypoints) = 1.0 - time_difference;
                    }
                }
                catch (tf::TransformException& ex)
                {
                    // If there is an exception, set the coordinates and weight to zero
                    coordinates(0, i + j * number_of_keypoints) = 0.0;
                    coordinates(1, i + j * number_of_keypoints) = 0.0;
                    coordinates(2, i + j * number_of_keypoints) = 0.0;
                    coordinates(3, i + j * number_of_keypoints) = 0.0;
                    ROS_ERROR("%s", ex.what());
                }
            }
        }

        // Calculate center of mass of hands (vectorized)
        Eigen::Vector3d center_of_mass = Eigen::Vector3d::Zero();

        // Initialize sum of weights to zero
        double sum_of_weights = 0.0;
        // Initialize center of mass to zero vector
        center_of_mass = Eigen::Vector3d::Zero();

        // Loop through each keypoint of both hands
        for (int i = 0; i < number_of_keypoints * number_of_hands; i++)
        {
            // Accumulate the weighted coordinates of the keypoints
            center_of_mass += coordinates.block(0, i, 3, 1) * coordinates(3, i);
            // Accumulate the weights
            sum_of_weights += coordinates(3, i);
        }

        // Divide by the sum of weights to get the center of mass
        center_of_mass /= sum_of_weights;
        // Publish center of mass as new target
        if (send_new_target)
        {
            com_hand.header.stamp    = ros::Time::now();
            com_hand.pose.position.x = center_of_mass(0);
            com_hand.pose.position.y = center_of_mass(1);
            com_hand.pose.position.z = center_of_mass(2);

            // com_pub.publish(com_hand);

            send_new_target = false;

            // Get position of end effector camera from TF
            tf::StampedTransform camera_transform;
            listener.lookupTransform("world", _frame_name_camera, ros::Time(0), camera_transform);

            // Get camera position
            Eigen::Vector3d camera_position =
                Eigen::Vector3d(camera_transform.getOrigin().x(), camera_transform.getOrigin().y(), camera_transform.getOrigin().z());
            // get rotation basis in Eigen matrix
            Eigen::Matrix3d rotation_basis;
            tf::matrixTFToEigen(camera_transform.getBasis(), rotation_basis);
            // get up vector in camera frame
            Eigen::Vector3d up_vector = rotation_basis * Eigen::Vector3d(0, 0, 1);

            // Get quaternion from LookAtQuat function
            Eigen::Quaterniond quaternion;
            LookAtQuat(camera_position, center_of_mass, up_vector, quaternion);

            // Publish target position as new target
            com_hand.pose.orientation.x = quaternion.x();
            com_hand.pose.orientation.y = quaternion.y();
            com_hand.pose.orientation.z = quaternion.z();
            com_hand.pose.orientation.w = quaternion.w();

            if (std::isnan(com_hand.pose.orientation.x) || std::isnan(com_hand.pose.orientation.y) || std::isnan(com_hand.pose.orientation.z) ||
                std::isnan(com_hand.pose.orientation.w) || std::isnan(center_of_mass(0)) || std::isnan(center_of_mass(1)) ||
                std::isnan(center_of_mass(2)))
            {
                ROS_WARN("HandTracking Node: Quaternion is NaN, not publishing target");
                send_new_target = false;
                continue;
            }
            com_pub.publish(com_hand);

            // get line of sight vector
            Eigen::Vector3d line_of_sight = center_of_mass - Eigen::Vector3d(camera_transform.getOrigin().x(), camera_transform.getOrigin().y(),
                                                                             camera_transform.getOrigin().z());

        }
        loop_rate.sleep();
    }  // end while(ros::ok())
}  // end main