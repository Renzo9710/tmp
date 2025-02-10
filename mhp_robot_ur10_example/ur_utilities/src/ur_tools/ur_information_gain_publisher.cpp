/*********************************************************************
 *
 *  Software License Agreement
 *
 *  Copyright (c) 2023,
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
*  Authors: Maximilian Krämer
*  Maintainer(s)/Modifier(s): Heiko Renz
 *********************************************************************/

#include <mhp_robot/MsgDistances.h>
#include <mhp_robot/robot_kinematic/robot_kinematic.h>
#include <ros/ros.h>
#include <ur_utilities/ur_misc/ur_information_gain.h>
#include <ur_utilities/ur_kinematic/ur_kinematic.h>
#include <ur_utilities/ur_misc/ur_utility.h>
#include <Eigen/Eigen>

using URKinematic = mhp_robot::robot_kinematic::URKinematic;
using URUtility   = mhp_robot::robot_misc::URUtility;

int main(int argc, char** argv)
{
#ifndef NDEBUG
    sleep(5);
#endif
    ros::init(argc, argv, "ur_danger_index");
    ros::NodeHandle nh;
    ROS_INFO_STREAM("UR Danger Index Node Started...");

    URKinematic kinematic = URKinematic();
    URUtility   utility   = URUtility();
    URInformationGain gain = URInformationGain(&nh, &kinematic,&utility);

    gain.publish();

    ros::waitForShutdown();
    return 0;
}
