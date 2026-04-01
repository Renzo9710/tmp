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

#include <mhp_robot/robot_trajectory_optimization/robot_information_gain_time_decrease.h>

namespace mhp_robot {
namespace robot_trajectory_optimization {

double RobotInformationGainTimeDecrease::computeCost(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k)
{
  double factor = 1;
  double gain = 0;

  if (_information_pcl.size() > 0)  // be sure we already received a point cloud
  {
    // get the transformation from depth camera to world for joint configuration x_k
    Eigen::Matrix4d T = _robot_kinematic->getEndEffectorMatrix(x_k) * _tf_cam_to_ee_link;

    // Transform POI to world frame
    Eigen::Vector4d z_axis{ 0, 0, 1, 1 };
    Eigen::Vector4d z_axis_world = T * z_axis;
    // get the scalar product between the camera and the POI axis
    double scalar_product =
        Eigen::Vector3d{ _poi_world[0] - T(0, 3), _poi_world[1] - T(1, 3), _poi_world[2] - T(2, 3) }.dot(
            Eigen::Vector3d{ z_axis_world[0] - T(0, 3), z_axis_world[1] - T(1, 3), z_axis_world[2] - T(2, 3) }) /
        (Eigen::Vector3d{ _poi_world[0] - T(0, 3), _poi_world[1] - T(1, 3), _poi_world[2] - T(2, 3) }.norm() *
         Eigen::Vector3d{ z_axis_world[0] - T(0, 3), z_axis_world[1] - T(1, 3), z_axis_world[2] - T(2, 3) }.norm());
    // set multiplication factor to 0 if the scalar product relates to an angle outside of the FOV (horizontal -->
    // Azure camera 37.5° in each direction) ~ 0.79
    if (scalar_product < 0.79)
    {
      factor = 0;
    }
    else
    {
      factor = scalar_product;
    }
    // if ((T.block<3, 1>(0, 3) - _poi_world.block<3, 1>(0, 0)).norm() > 0.5)
    // {
    //     gain = 0;
    // }
    // else
    // {
    inverseDistanceWeigthing(T.block<3, 1>(0, 3), gain);
    // }
    // std::cout << "Factor: " << factor << std::endl;
    // std::cout << "Gain: " << gain << std::endl;
    // std::cout << "Gain with factor: " << 1 / (factor * gain + _eps) << std::endl;
    return _w_gain / (factor * gain + _eps);  // add small epsilon to avoid division by zero
  }
  else
  {
    ROS_WARN_THROTTLE(5,"RobotInformationGainTimeDecrease: No point cloud received yet");
    return 0;
  }
}

void RobotInformationGainTimeDecrease::inverseDistanceWeigthing(const Eigen::Ref<const Eigen::Vector3d>& point, double& gain)
{
    // get the distance to the point cloud points (with Eigen Matrix for vectorized operations)
    auto start_time = std::chrono::system_clock::now();

    int one_pcl_size = _pcl_num / _buffer_size;
    if (_first_pcl)
    {
        _weighted_dists = Eigen::VectorXd::Zero(_pcl_num);
        _time_weights   = Eigen::VectorXd::Zero(_pcl_num);
        // Linear decrease for time weights --> current pcl has weight of 1 and the first pcl has weight of 1/_buffer_size
        for (int i = 0; i < _buffer_size; ++i)
        {
            _time_weights.segment(i * one_pcl_size, one_pcl_size) = Eigen::VectorXd::Ones(one_pcl_size) * (1.0 / (_buffer_size - i));
        }
        _first_pcl = false;
    }
    if (_pcl_num > 0 && _new_pcl)
    {
        if (_pcl_num != _weighted_dists.size())
        {
            ROS_ERROR_ONCE("Buffer size is not correct");
            std::cout << "Information pcl size: " << _pcl_num << " Weighted dists size: " << _weighted_dists.size() << std::endl;
        }

        Eigen::MatrixXd pcl_points = Eigen::MatrixXd::Zero(3, one_pcl_size);
        _pcl_mutex.lock();
        for (int i = 0; i < one_pcl_size; ++i)
        {
            pcl_points.col(i) = Eigen::Vector3d{_information_pcl.points[_information_pcl.size() - one_pcl_size + i].x,
                                                _information_pcl.points[_information_pcl.size() - one_pcl_size + i].y,
                                                _information_pcl.points[_information_pcl.size() - one_pcl_size + i].z};
        }
        _pcl_mutex.unlock();

        Eigen::VectorXd dists = (pcl_points.colwise() - point).colwise().norm();

        int p         = 2;  // power of the inverse distance weighting
        dists.array() = dists.array().pow(-p);

        _weighted_dists.head(_pcl_num - one_pcl_size) = _weighted_dists.tail(_pcl_num - one_pcl_size);
        _weighted_dists.tail(one_pcl_size)            = dists.array() / dists.sum();

        _new_pcl = false;  // flag is reseted by PCL callback
    }

    // get the inverse distance weighting
    gain = 0;

    _pcl_mutex.lock();
    for (int i = 0; i < _pcl_num; ++i)
    {
        gain += (_weighted_dists(i)) * _time_weights(i) *_information_pcl.points[i].intensity;
    }
    _pcl_mutex.unlock();
    auto end_time                                 = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end_time - start_time;
}

}  // namespace robot_trajectory_optimization
}  // namespace mhp_robot