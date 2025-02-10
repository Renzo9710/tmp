/*********************************************************************
 *
 *  Software License Agreement
 *
 *  Copyright (c) 2018,
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

#ifndef ROBOT_GLOBAL_PLANNER_H
#define ROBOT_GLOBAL_PLANNER_H

#include <ompl/base/Planner.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/PathSimplifier.h>
#include <mhp_robot/robot_collision/robot_collision.h>
#include <mhp_robot/robot_obstacle/obstacle_list.h>
#include <mhp_robot/robot_path_pilot/robot_global_planner_method.h>
#include <mutex>
#include <thread>

namespace mhp_robot {
namespace robot_path_pilot {

class RobotGlobalPlanner
{
 public:
    using Ptr  = std::shared_ptr<RobotGlobalPlanner>;
    using UPtr = std::unique_ptr<RobotGlobalPlanner>;

    RobotGlobalPlanner() = default;

    RobotGlobalPlanner(const RobotGlobalPlanner&) = delete;
    RobotGlobalPlanner(RobotGlobalPlanner&&)      = delete;
    RobotGlobalPlanner& operator=(const RobotGlobalPlanner&) = delete;
    RobotGlobalPlanner& operator=(RobotGlobalPlanner&&) = delete;
    virtual ~RobotGlobalPlanner() {}

    void requestPath();
    void setStartAndGoalState(const Eigen::Ref<const Eigen::VectorXd>& start, const Eigen::Ref<const Eigen::VectorXd>& goal);
    bool getPath(std::vector<Eigen::VectorXd>& path_eigen, std::shared_ptr<ompl::geometric::PathGeometric>& path_ompl);

    bool initialize(robot_collision::RobotCollision::UPtr robot_collision, robot_kinematic::RobotKinematic::UPtr robot_kinematic);

    void clearPlan();
    void clearQuery();
    void shutdown();

    void update();

    enum PathObjective { NONE, JOINTSPACE, TASKSPACE } _path_objective = NONE;

    void setPathObjective(const PathObjective& objective);
    const PathObjective& getPathObjective() const;

   std::string getName() const;
   bool getDeltaTime(double& deltat) const;

   virtual bool fromParameterServer(const std::string& ns) = 0;

 protected:
    using ObstacleList   = robot_obstacle::ObstacleList;
    using RobotKinematic = robot_kinematic::RobotKinematic;
    using RobotCollision = robot_collision::RobotCollision;
    using RobotUtility   = robot_misc::RobotUtility;

    virtual bool isStateValid(const ompl::base::State* state);

    void plan();
    void stopPlanning();
    void updatePathEigen();

    double _min_self_collision     = 0.05;
    double _min_obstacle_collision = 0.1;
    double _min_ground_collision   = 0.1;
    double _min_roof_collision     = 0.1;
    double _min_plane_collision    = 0.1;
    double _validity_resolution    = 0.01;
    double _solve_time             = 0.01;
    double _simplify_time          = 0.01;
    double _max_plan_time          = 0.5;
    double _max_path_length        = 5;
    int _dim                       = 0;

    bool _initialized = false;
    bool _simplify    = true;
    bool _plan        = true;
    bool _new_plan    = false;

    std::thread _planning;
    std::mutex _mutex;

    ObstacleList _obstacle_manager;
    RobotCollision::UPtr _robot_collision;
    std::vector<robot_misc::Obstacle> _static_obstacles, _dynamic_obstacles;
    std::vector<robot_misc::Plane> _planes;

    std::shared_ptr<ompl::base::RealVectorStateSpace> _joint_space;
    ompl::base::SpaceInformationPtr _joint_space_info;
    ompl::base::ProblemDefinitionPtr _planning_problem;
    ompl::geometric::PathSimplifierPtr _simplifier;
    ompl::base::OptimizationObjectivePtr _objective;
    RobotGlobalPlannerMethod::Ptr _global_planner_method;

    std::shared_ptr<ompl::geometric::PathGeometric> _path_ompl_share;
    std::vector<Eigen::VectorXd> _path_eigen, _path_eigen_share;
};

}  // namespace robot_path_pilot
}  // namespace mhp_robot

#endif  // ROBOT_GLOBAL_PLANNER_H
