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

#include <mhp_robot/robot_trajectory_optimization/robot_occlusion_potential_distance.h>

namespace mhp_robot
{
namespace robot_trajectory_optimization
{
double RobotOcclusionPotentialDistance::computePotentials(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k)
{
  // check if occlusion distribution is available
  if (_occlusion_distribution.size() == 0)
  {
    ROS_WARN_THROTTLE(5, "RobotOcclusionPotentialDistance: Occlusion distribution is not available");
    return 0.0;
  }

  // TODO(renz): Check if Orthodrome instead of point to line distance would be beneficial
  Eigen::Matrix4d T = _robot_kinematic->getEndEffectorMatrix(x_k) * _tf_cam_to_ee_link;
  Eigen::Vector3d ee_position = T.block<3, 1>(0, 3);

  // Get row indices of occluded points that have 0 in the last regarding row of the occlusion distribution (note that
  // k=0 is the current time step which is included in the prediction matrix)
  Eigen::VectorXi indices;
  if (_use_occlusion_prediction)
  {
    if (k < _occlusion_predictions.rows())
    {
      indices = (_occlusion_predictions.row(k).array() == 0).cast<int>();
    }
    else
    {
      indices = (_occlusion_predictions.row(_occlusion_predictions.rows() - 1).array() == 0).cast<int>();
    }
  }
  else
  {
    indices = (_occlusion_distribution.col(3).array() == 0).cast<int>();
  }
  // Count how many rows satisfy the condition
  int count = indices.sum();

  // Select rows where last column is 1
  // Calculate the end point of an ray from the sphere center with a fixed length of _distance_threshold
  Eigen::ArrayXd distanceEEtoRays = Eigen::ArrayXd::Zero(count);
  Eigen::Vector3d ee_position_sphere = ee_position - _occlusion_sphere_center;
  Eigen::Vector3d sphere_coords = cartesianToSpherical(ee_position_sphere);

  // Prepare values for loop
  int j = 0;
  double theta;
  double phi;

  for (int i = 0; i < _occlusion_distribution.rows(); ++i)
  {
    theta = _occlusion_distribution(i, 1);
    phi = _occlusion_distribution(i, 2);

    if (indices(i) && (std::abs(theta - sphere_coords(1)) < _theta_threshold &&
                       std::abs(phi - sphere_coords(2)) <
                           _phi_threshold))  // Only take occupied values in the correct direction, else we get
                                             // runtime problems for high resolution occlusion distribution
    {
      distanceEEtoRays(j) = getPointToLineDistance(
          ee_position, _occlusion_sphere_center,
          _occlusion_sphere_center + sphericalToCartesian(_optimal_view_distance * 10, _occlusion_distribution(i, 1),
                                                          _occlusion_distribution(i, 2)));
      // Issue: distances at the end of the rays are bigger since the rays are not normalized to the sphere radius
      // Normalize the distance based on the distance to the sphere center
      distanceEEtoRays(j) /= ee_position_sphere.norm();      
      j++;
    }
  }

  // Calculate potential
  double potential = 0.0;

  // Add potential to get closer to the occlusion sphere center
  if (ee_position_sphere.norm() > _optimal_view_distance)
  {
    // if (k == 0)
    //   std::cout << "Potential add at dee>dopt: "
    //             << _weight_position_distance * std::pow((_optimal_view_distance - ee_position_sphere.norm()), 2)
    //             << std::endl;
    potential += _weight_position_distance * std::pow((_optimal_view_distance - ee_position_sphere.norm()), 2);
  }
  else if (ee_position_sphere.norm() < _minimal_optimal_view_distance)
  {
    // if (k == 0)
    //   std::cout << "Potential add at dee<dmin: "
    //             << _weight_position_distance * std::pow((ee_position_sphere.norm() - _minimal_optimal_view_distance), 2)
    //             << std::endl;
    potential += _weight_position_distance * std::pow((ee_position_sphere.norm() - _minimal_optimal_view_distance), 2);
  }
  else
  {
    // if (k == 0)
    //   std::cout << "Potential Distance else" << std::endl;
    potential += 0;
  }

  // Orientation potential for positions inside the view threshold (outside it does not matter how the camera is
  // oriented)
  if (true)
  {
    Eigen::Vector4d z_axis_world = T * _z_axis;
    // get the scalar product between the camera and the POI axis
    _scalar_product =
        Eigen::Vector3d{ _occlusion_sphere_center[0] - T(0, 3), _occlusion_sphere_center[1] - T(1, 3),
                         _occlusion_sphere_center[2] - T(2, 3) }
            .dot(Eigen::Vector3d{ z_axis_world[0] - T(0, 3), z_axis_world[1] - T(1, 3), z_axis_world[2] - T(2, 3) }) /
        (Eigen::Vector3d{ _occlusion_sphere_center[0] - T(0, 3), _occlusion_sphere_center[1] - T(1, 3),
                          _occlusion_sphere_center[2] - T(2, 3) }
             .norm() *
         Eigen::Vector3d{ z_axis_world[0] - T(0, 3), z_axis_world[1] - T(1, 3), z_axis_world[2] - T(2, 3) }.norm());

    // set potential to 0 if the scalar product relates to an angle inside of the FOV (horizontal -->
    // Azure camera 37.5° in each direction) ~ 0.79; Else increase potential
    if (_scalar_product < 0.79)
    {
      // if (k == 0)
      //   std::cout << "Potential add at s<0.79: "
      //             << _weight_orientation * (_scalar_product * ((_y_val_at_zero_for_orientation - 1) / -0.79) +
      //                                       _y_val_at_zero_for_orientation)
      //             << std::endl;
      potential += _weight_orientation *
                   (_scalar_product * ((_y_val_at_zero_for_orientation - 1) / -0.79) + _y_val_at_zero_for_orientation);
    }
    else
    {
      // if (k == 0)
      //   std::cout << "Potential add at s>0.79: "
      //             << _weight_orientation * std::pow((((_scalar_product - 0.79) / (1 - 0.79)) - 1), 2) << std::endl;
      potential += _weight_orientation * std::pow((((_scalar_product - 0.79) / (1 - 0.79)) - 1), 2);
    }
  }
  else
  {
    // if (k == 0)
    //   std::cout << "Potential alignment else" << std::endl;
    potential += 0;
  }

  // if ((ee_position - _occlusion_sphere_center).norm() <= _optimal_view_distance)
  if (true)
  {
    // Resizing distanceEEtoRays to the correct size
    if (distanceEEtoRays.size() == 0)
    {
      ROS_WARN_ONCE("No points close to Endeffector");
      potential += 0;
    }
    else
    {
      distanceEEtoRays.conservativeResize(j);
    }

    // Calculate potential for distance to occlusion
    distanceEEtoRays = distanceEEtoRays.cwiseMin(_distance_margin_to_occlusion);
    if (_min_distance)
    {
      potential += _weight_position * std::pow((distanceEEtoRays.minCoeff() / _distance_margin_to_occlusion - 1), 2);
    }
    else
    {
      // if (k == 0)
      //   std::cout << "Potential Distance occ no min distance: "
      //             << _weight_position * ((distanceEEtoRays / _distance_margin_to_occlusion - 1).square()).sum()
      //             << std::endl;
      potential += _weight_position * ((distanceEEtoRays / _distance_margin_to_occlusion - 1).square()).sum();
    }
  }
  else
  {
    // if (k == 0)
    //   std::cout << "Potential Distance occ else" << std::endl;
    potential += 0;
  }

  // Safe for plotting in Cobra GUI
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

double
RobotOcclusionPotentialDistance::getPointToPointDistance(const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& p,
                                                         const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& q) const
{
  return (p - q).norm();
}

double RobotOcclusionPotentialDistance::getPointToLineDistance(
    const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& p, const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& q1,
    const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& q2) const
{
  Eigen::Vector3d v, w;
  v = q2 - q1;
  w = p - q1;

  double c1 = w.dot(v);
  if (c1 <= 0)
  {
    return getPointToPointDistance(p, q1);
  }

  double c2 = v.dot(v);
  if (c2 <= c1)
  {
    return getPointToPointDistance(p, q2);
  }

  return (p - (q1 + (c1 / c2) * v)).norm();
}
}  // namespace robot_trajectory_optimization
}  // namespace mhp_robot
