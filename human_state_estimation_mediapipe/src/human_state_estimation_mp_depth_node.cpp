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

#include <human_state_esitmation_mediapipe/human_state_estimation_mp_depth.h>


int main(int argc, char** argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "human_state_estimation_mp_depth_node");
  ros::NodeHandle nh;

  // Your code here
  human_state_estimation_mediapipe::HumanStateEstimationMPDepth human_state_estimation_mp_depth(true, true);

  ros::Rate loop_rate(30);
  while (ros::ok())
  {
    human_state_estimation_mp_depth.publish();

    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}