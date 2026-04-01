/*********************************************************************
 *
 *  Software License Agreement
 *
 *  Copyright (c) 2025,
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

#ifndef ROBOT_OCCLUSION_POTENTIAL_DISTANCE_H
#define ROBOT_OCCLUSION_POTENTIAL_DISTANCE_H

#include <mhp_robot/robot_trajectory_optimization/robot_occlusion_potential.h>

namespace mhp_robot 
{
namespace robot_trajectory_optimization
{
class RobotOcclusionPotentialDistance : virtual public RobotOcclusionPotential
{
 public:
  RobotOcclusionPotentialDistance() = default;

  using Ptr = std::shared_ptr<RobotOcclusionPotentialDistance>;
  using UPtr = std::unique_ptr<RobotOcclusionPotentialDistance>;

  double computePotentials(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k) override;

  double getOptimalViewDistance() const override
  {
    return _optimal_view_distance;
  }

 protected:
  ros::Publisher _marker_ray_ends_pub;

  double _weight_orientation = 1.0;
  double _weight_position = 1.0;
  double _weight_position_distance = 1.0;

  double _optimal_view_distance = 1.0;
  double _minimal_optimal_view_distance = 0.5;  // based on camera specs
  double _distance_margin_to_occlusion = 1.0;
  bool _min_distance = false;

 private:
  double getPointToPointDistance(const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& p,
                                 const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& q) const;

  double getPointToLineDistance(const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& p,
                                const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& q1,
                                const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& q2) const;

  Eigen::Vector4d _z_axis{ 0, 0, 1, 1 };
  double _scalar_product;
  double _y_val_at_zero_for_orientation = 2.0;
};

}  // namespace robot_trajectory_optimization
}  // namespace mhp_robot

#endif  // ROBOT_OCCLUSION_POTENTIAL_DISTANCE_H
