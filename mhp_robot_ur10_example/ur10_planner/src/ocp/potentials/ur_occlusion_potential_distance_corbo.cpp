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

#include <ur10_planner/ocp/potentials/ur_occlusion_potential_distance_corbo.h>

namespace mhp_planner
{
URBaseOcclusionPotential::Ptr UROcclusionPotentialDistance::getInstance() const
{
  return std::make_shared<UROcclusionPotentialDistance>();
}
bool UROcclusionPotentialDistance::fromParameterServer(const std::string& ns)
{
  ros::NodeHandle nh;
  nh.getParam(ns + "/w_position", _weight_position);
  nh.getParam(ns + "/w_orientation", _weight_orientation);
  nh.getParam(ns + "/w_position_distance", _weight_position_distance);
  nh.getParam(ns + "/optimal_view_distance", _optimal_view_distance);
  nh.getParam(ns + "/distance_margin_to_occlusion", _distance_margin_to_occlusion);
  nh.getParam(ns + "/theta_margin_to_occlusion", _theta_threshold);
  nh.getParam(ns + "/phi_margin_to_occlusion", _phi_threshold);
  nh.getParam(ns + "/min_distance", _min_distance);
  nh.getParam(ns + "/use_occlusion_prediction", _use_occlusion_prediction);
  std::cout<<"Params: w_position: " << _weight_position << ", w_orientation: " << _weight_orientation << ", w_position_distance: " << _weight_position_distance
           << ", optimal_view_distance: " << _optimal_view_distance << ", distance_margin_to_occlusion: " << _distance_margin_to_occlusion
           << ", theta_margin_to_occlusion: " << _theta_threshold << ", phi_margin_to_occlusion: " << _phi_threshold
           << ", min_distance: " << _min_distance << ", use_occlusion_prediction: " << _use_occlusion_prediction << std::endl;
  return true;
}

}  // namespace mhp_planner
