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
 *********************************************************************/

#ifndef ERGODIC_CORBO_H
#define ERGODIC_CORBO_H

#include <mhp_robot/robot_path_pilot/planner_methods/ergodic.h>
#include <ur10_planner/tasks/path_pilot/ur_base_global_planner_method_corbo.h>

namespace mhp_planner {

class ErgodicCORBO : public URBaseGlobalPlannerMethodCORBO, public mhp_robot::robot_path_pilot::planner_methods::Ergodic
{
 public:
    using Ptr  = std::shared_ptr<ErgodicCORBO>;
    using UPtr = std::unique_ptr<ErgodicCORBO>;

    ErgodicCORBO() = default;

    URBaseGlobalPlannerMethodCORBO::Ptr getInstance() const override;

    bool fromParameterServer(const std::string &ns) override;

 private:
};

FACTORY_REGISTER_OBJECT(ErgodicCORBO, URBaseGlobalPlannerMethodCORBO)

}  // namespace corbo

#endif  // ERGODIC_CORBO_H
