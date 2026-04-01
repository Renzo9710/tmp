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

#include <ur10_planner/ocp/gains/ur_information_gain_potential_corbo.h>

namespace mhp_planner
{
URBaseInformationGain::Ptr URInformationGainPotential::getInstance() const
{
  return std::make_shared<URInformationGainPotential>();
}

bool URInformationGainPotential::fromParameterServer(const std::string &ns)
    {
        ros::NodeHandle nh;

        // Set weight for the information gain
        nh.getParam(ns + "/weight", _weight_gain);
        nh.getParam(ns + "/weight_distance", _weight_distance);
        nh.getParam(ns + "/weight_orientation", _weight_orientation);
        nh.getParam(ns + "/optimal_view_distance", _optimal_view_distance);
        nh.getParam(ns + "/gain_margin", _gain_margin);
        std::cout<< "URInformationGainPotential parameters: weight_gain: " << _weight_gain << ", weight_distance: " << _weight_distance
                 << ", weight_orientation: " << _weight_orientation << ", optimal_view_distance: " << _optimal_view_distance
                 << ", gain_margin: " << _gain_margin << std::endl;
        return true;
    } 

// #ifdef MESSAGE_SUPPORT
// bool URInformationGainPotential::fromMessage(const corbocustom::messages::URInformationGain& message,
//                                                   std::stringstream* issues)
// {
//   return fromMessage(message.ur_information_gain_potential(), issues);
// }

// void URInformationGainPotential::toMessage(corbocustom::messages::URInformationGain& message) const
// {
//   toMessage(*message.mutable_ur_information_gain_potential());
// }

// bool URInformationGainPotential::fromMessage(const corbocustom::messages::URInformationGainPotential& message,
//                                                   std::stringstream* issues)
// {
//   _weight_gain = message.w_gain();
//   _weight_orientation = message.w_orientation();
//   _weight_distance = message.w_distance();
//   _optimal_view_distance = message.optimal_view_distance();
//   _gain_margin = message.gain_margin();
//   return true;
// }

// void URInformationGainPotential::toMessage(corbocustom::messages::URInformationGainPotential& message) const
// {
//   message.set_w_gain(_weight_gain);
//   message.set_w_orientation(_weight_orientation);
//   message.set_w_distance(_weight_distance);
//   message.set_optimal_view_distance(_optimal_view_distance);
//   message.set_gain_margin(_gain_margin);
// }
// #endif
}  // namespace mhp_planner
