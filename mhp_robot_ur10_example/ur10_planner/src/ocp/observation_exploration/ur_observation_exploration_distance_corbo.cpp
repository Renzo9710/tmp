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

#include <ur10_planner/ocp/observation_exploration/ur_observation_exploration_distance_corbo.h>

namespace mhp_planner
{
  URBaseObservationExploration::Ptr URObservationExplorationDistance::getInstance() const
  {
    return std::make_shared<URObservationExplorationDistance>();
  }
  bool URObservationExplorationDistance::fromParameterServer(const std::string &ns)
  {
    ros::NodeHandle nh;

    nh.getParam(ns + "/switch_distance_human_object", _switch_distance_human_object);

    // Get information gain
    std::string information_gain_type;
    nh.getParam(ns + "/information_gain/information_gain_type", information_gain_type);
    if (information_gain_type == "None")
    {
      _ur_information_gain = {};
    }
    else if (information_gain_type == "URInformationGainSimple")
    {
      _ur_information_gain = Factory<URBaseInformationGain>::instance().create(information_gain_type);
      // import parameters
      if (_ur_information_gain)
      {
        if (!_ur_information_gain->fromParameterServer(ns + "/information_gain"))
          return false;
      }
      else
      {
        ROS_ERROR("URStageCosts: unknown information gain specified.");
        return false;
      }
    }
    else if (information_gain_type == "URInformationGainTimeDecrease")
    {
      _ur_information_gain = Factory<URBaseInformationGain>::instance().create(information_gain_type);
      // import parameters
      if (_ur_information_gain)
      {
        if (!_ur_information_gain->fromParameterServer(ns + "/information_gain"))
          return false;
      }
      else
      {
        ROS_ERROR("URStageCosts: unknown information gain specified.");
        return false;
      }
    }
    else if (information_gain_type == "URInformationGainPotential")
    {
      _ur_information_gain = Factory<URBaseInformationGain>::instance().create(information_gain_type);
      // import parameters
      if (_ur_information_gain)
      {
        if (!_ur_information_gain->fromParameterServer(ns + "/information_gain"))
          return false;
      }
      else
      {
        ROS_ERROR("URStageCosts: unknown information gain specified.");
        return false;
      }
    }
    else
    {
      _ur_information_gain = {};
    }

    // Get occlusion potential
     std::string occlusion_potential_type;
    nh.getParam(ns + "/occlusion_potential/occlusion_potential_type", occlusion_potential_type);
    if (occlusion_potential_type == "None")
    {      _ur_occlusion_potential = {};
    }
    else if (occlusion_potential_type == "UROcclusionPotentialDistance")
    { 
      _ur_occlusion_potential = Factory<URBaseOcclusionPotential>::instance().create(occlusion_potential_type);
      // import parameters
      if (_ur_occlusion_potential)
      {
        if (!_ur_occlusion_potential->fromParameterServer(ns + "/occlusion_potential"))
          return false;
      }
      else
      {
        ROS_ERROR("URStageCosts: unknown occlusion potential specified.");
        return false;
      }
    }
    else
    {
      _ur_occlusion_potential = {};
    }
    std::cout<< "URObservationExplorationDistance parameters: switch_distance_human_object: " << _switch_distance_human_object
             << ", information_gain_type: " << information_gain_type
             << ", occlusion_potential_type: " << occlusion_potential_type << std::endl;
     return true;
  }
  

} // namespace mhp_planner
