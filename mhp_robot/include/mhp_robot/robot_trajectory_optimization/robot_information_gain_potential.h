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
#ifndef ROBOT_INFORMATION_GAIN_POTENTIAL_H
#define ROBOT_INFORMATION_GAIN_POTENTIAL_H

#include <mhp_robot/robot_trajectory_optimization/robot_information_gain.h>

namespace mhp_robot
{
namespace robot_trajectory_optimization
{
class RobotInformationGainPotential : virtual public RobotInformationGain
{
 public:
  RobotInformationGainPotential() = default;

  using Ptr = std::shared_ptr<RobotInformationGainPotential>;
  using UPtr = std::unique_ptr<RobotInformationGainPotential>;

  double computeCost(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k) override;
  void inverseDistanceWeighting(const Eigen::Ref<const Eigen::Vector3d>& point, double& gain);

 protected:
  double _weight_gain;
  double _weight_orientation;
  double _weight_distance;
  double _optimal_view_distance;
  double _minimal_optimal_view_distance = 0.5;
  double _gain_margin;

 private:
  bool _first_pcl = true;
  Eigen::VectorXd _weighted_dists;
  Eigen::VectorXd _time_weights;

  pcl::PointCloud<pcl::PointXYZI> _tmp_information_pcl;
  bool _tmp_new_pcl = _new_pcl;
  int _tmp_pcl_num = _pcl_num;

  double _y_val_at_zero_for_orientation = 2.0;

  std::map<std::vector<double>, double> _buffer_gains;
};

}  // namespace robot_trajectory_optimization
}  // namespace mhp_robot
#endif  // ROBOT_INFORMATION_GAIN_POTENTIAL_H
