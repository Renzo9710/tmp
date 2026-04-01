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

#include <mhp_robot/robot_trajectory_optimization/robot_information_gain_potential.h>

namespace mhp_robot
{
namespace robot_trajectory_optimization
{
double RobotInformationGainPotential::computeCost(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k)
{
  double factor = 1;
  double gain = 0;
  if (_information_pcl.size() > 0)  // be sure we already received a point cloud
  {
    double potential = 0;

    // get the transformation from depth camera to world for joint configuration x_k
    Eigen::Matrix4d T = _robot_kinematic->getEndEffectorMatrix(x_k) * _tf_cam_to_ee_link;
    double distance_poi_to_cam = (_poi_world.head(3) - T.block<3, 1>(0, 3)).norm();
    // Distance towards the POI
    // Add potential to get closer to the occlusion sphere center
    if (distance_poi_to_cam > _optimal_view_distance)
    {
      potential += _weight_distance * std::pow((_optimal_view_distance - distance_poi_to_cam), 2);
    }
    else if (distance_poi_to_cam < _minimal_optimal_view_distance)
    {
      potential += _weight_distance * std::pow((distance_poi_to_cam - _minimal_optimal_view_distance), 2);
    }
    else
    {
      potential += 0;
    }

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
      potential += _weight_orientation *
                   (scalar_product * ((_y_val_at_zero_for_orientation - 1) / -0.79) + _y_val_at_zero_for_orientation);
    }
    else
    {
      potential += _weight_orientation * std::pow((((scalar_product - 0.79) / (1 - 0.79)) - 1), 2);
    }

    // Get inverse distance weighting
    std::vector<double> gain_vector{ T.block<3, 1>(0, 3)[0], T.block<3, 1>(0, 3)[1], T.block<3, 1>(0, 3)[2] };
    if (auto it = _buffer_gains.find(gain_vector); it != _buffer_gains.end())
    {
      // std::cout << "Found gain in buffer" << std::endl;
      gain = _buffer_gains[gain_vector];
    }
    else
    {
      inverseDistanceWeighting(T.block<3, 1>(0, 3), gain);
      _buffer_gains.insert({ gain_vector, gain });
    }
    if (_buffer_gains.size() > 90)
    {
      _buffer_gains.erase(_buffer_gains.begin(), std::next(_buffer_gains.begin(), 1));
    }

    double y_val_at_zero_for_gain = 2.0;
    if (gain < _gain_margin)
    {
      potential += _weight_gain * (gain * ((y_val_at_zero_for_gain - 1) / -_gain_margin) + y_val_at_zero_for_gain);
    }
    else
    {
      potential += _weight_gain * std::pow((((gain - _gain_margin) / (1 - _gain_margin)) - 1), 2);
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
  else
  {
    ROS_WARN_THROTTLE(5, "RobotInformationGainPotential: No point cloud received yet");
    return 0;
  }
}

void RobotInformationGainPotential::inverseDistanceWeighting(const Eigen::Ref<const Eigen::Vector3d>& point,
                                                             double& gain)
{
  // get the distance to the point cloud points (with Eigen Matrix for vectorized operations)
  auto start_time = std::chrono::system_clock::now();

  int one_pcl_size = _pcl_num / _buffer_size;
  if (_first_pcl)
  {
    _weighted_dists = Eigen::VectorXd::Zero(_pcl_num);
    _time_weights = Eigen::VectorXd::Zero(_pcl_num);
    // Linear decrease for time weights --> current pcl has weight of 1 and the first pcl has weight of 1/_buffer_size
    for (int i = 0; i < _buffer_size; ++i)
    {
      _time_weights.segment(i * one_pcl_size, one_pcl_size) =
          Eigen::VectorXd::Ones(one_pcl_size) * (1.0 / (_buffer_size - i));
    }
    _first_pcl = false;
  }
  Eigen::VectorXd intensities(_pcl_num);

  if (_pcl_num > 0)
  {
    if (_pcl_num != _weighted_dists.size())
    {
      ROS_ERROR_ONCE("Buffer size is not correct");
      std::cout << "Information pcl size: " << _pcl_num << " Weighted dists size: " << _weighted_dists.size()
                << std::endl;
    }

    Eigen::MatrixXd pcl_points = Eigen::MatrixXd::Zero(3, _pcl_num);

    for (int i = 0; i < _pcl_num; ++i)
    {
      const auto& pt = _information_pcl.points[i];
      pcl_points(0, i) = pt.x;
      pcl_points(1, i) = pt.y;
      pcl_points(2, i) = pt.z;
      intensities(i) = pt.intensity;
    }

    int p = 2;  // power of the inverse distance weighting
    Eigen::VectorXd tmp = (((pcl_points.colwise() - point).colwise().norm()).array().pow(-p)).matrix();

    _weighted_dists = tmp / tmp.sum();

    _new_pcl = false;  // flag is reseted by PCL callback
  }

  // compare time of loop and vectorized operations --> loop is slower
  // gain = 0;
  // for (int i = 0; i < _pcl_num; ++i)
  // {
  //   gain += (_weighted_dists(i)) * _time_weights(i) * _information_pcl.points[i].intensity;
  // }

  gain = 0;
  gain = (_weighted_dists.array() * _time_weights.array() * intensities.array()).sum();
  auto end_time = std::chrono::system_clock::now();
}

}  // namespace robot_trajectory_optimization
}  // namespace mhp_robot
