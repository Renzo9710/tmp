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
 *  Authors: Maximilian Krämer, Heiko Renz
 *  Maintainer(s)/Modifier(s):
 *********************************************************************/

#include <mhp_robot/robot_trajectory_optimization/robot_information_gain.h>

namespace mhp_robot
{
    namespace robot_trajectory_optimization
    {

        void RobotInformationGain::computeGradient(int k, const Eigen::Ref<const Eigen::VectorXd> &x_k, Eigen::Ref<Eigen::VectorXd> dx)
        {
            Eigen::VectorXd diff = Eigen::VectorXd::Zero(x_k.size());
            for (int i = 0; i < x_k.size(); ++i)
            {
                diff(i) = _eps;
                dx(i) = (computeGain(k, x_k + diff) - computeGain(k, x_k - diff)) / (2 * _eps);
                diff(i) = 0.0;
            }
        }

        void RobotInformationGain::computeHessian(int k, const Eigen::Ref<const Eigen::VectorXd> &x_k, Eigen::Ref<Eigen::MatrixXd> dxdx)
        {
            int n = x_k.size();
            Eigen::VectorXd diff = Eigen::VectorXd::Zero(n);
            Eigen::VectorXd ldx(n), rdx(n);
            for (int i = 0; i < n; ++i)
            {
                diff(i) = _eps;
                computeGradient(k, x_k + diff, rdx);
                computeGradient(k, x_k - diff, ldx);
                dxdx.col(i) = (rdx - ldx) / (2 * _eps);
                diff(i) = 0.0;
            }
        }

        bool RobotInformationGain::initialize(robot_kinematic::RobotKinematic::UPtr robot_kinematic)
        {

            _robot_kinematic = std::move(robot_kinematic);

            ros::NodeHandle nh;

            // Should we buffer the point cloud?
            _buffer_pcl = nh.param("ufomap_server_node/buffer_pcl", true);
            _buffer_size = nh.param("ufomap_server_node/buffer_size", 10);
            _camera_frame = nh.param("depth_camera_frame", std::string("camera_3d_depth_camera_link"));

            // Point of interest in world frame
            std::vector<double> poi = nh.param("/ufomap_server_node/poi_world", std::vector<double>{1.19, 0.16, 0.33});
            _poi_world = Eigen::Vector4d(poi[0], poi[1], poi[2], 1.0);

            std::string _information_gain_topic_name = "/ufomap_server_node/info_dist_cloud";

            if (_buffer_pcl)
            {
                _information_gain_sub = nh.subscribe(_information_gain_topic_name, 1, &RobotInformationGain::informationGainCallbackBuffer, this);
            }
            else
            {
                _information_gain_sub = nh.subscribe(_information_gain_topic_name, 1, &RobotInformationGain::informationGainCallback, this);
            }

            tf::StampedTransform transform;
            tf::TransformListener tf_listener;
            try
            {
                ros::Time now = ros::Time::now();
                Eigen::Affine3d tmp;
                tf_listener.waitForTransform("/ee_link", "/camera_3d_depth_camera_link", ros::Time(0), ros::Duration(10.0));
                tf_listener.lookupTransform("/ee_link", "/camera_3d_depth_camera_link", ros::Time(0), transform);
                tf::transformTFToEigen(transform, tmp);
                _tf_cam_to_ee_link = tmp.matrix();
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s", ex.what());
                ros::Duration(1.0).sleep();
            }

            _initialized = true;

            return true;
        }

        bool RobotInformationGain::update(double dt)
        {
            _dt = dt;

            return false;
        }

        bool RobotInformationGain::isInitialized() const { return _initialized; }

        void RobotInformationGain::informationGainCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) { pcl::fromROSMsg(*msg, _information_pcl); }
        void RobotInformationGain::informationGainCallbackBuffer(const mhp_robot::MsgInfoPCLS::ConstPtr &msg)
        {
            _information_pcl.clear();

            pcl::PointCloud<pcl::PointXYZI> pcl_all;
            for (int i = 0; i < msg->pcls.size(); ++i)
            {
                pcl::PointCloud<pcl::PointXYZI> pcl;
                pcl::fromROSMsg(msg->pcls[i], pcl);
                pcl_all += pcl;
            }

            _pcl_mutex.lock();
            _information_pcl = pcl_all;
            _pcl_mutex.unlock();
            _pcl_num_mutex.lock();
            _pcl_num = _information_pcl.size();
            _pcl_num_mutex.unlock();
            _new_pcl_mutex.lock();
            _new_pcl = true;
            _new_pcl_mutex.unlock();
        }

    } // namespace robot_trajectory_optimization
} // namespace mhp_robot
