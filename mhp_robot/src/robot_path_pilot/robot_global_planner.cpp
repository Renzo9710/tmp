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
#include <mhp_robot/robot_path_pilot/objectives/robot_path_objective_task_space.h>
#include <mhp_robot/robot_path_pilot/robot_global_planner.h>

namespace mhp_robot {
namespace robot_path_pilot {

void RobotGlobalPlanner::plan()
{
    if (!_initialized) return;

    _new_plan       = false;
    double duration = 0.0;
    ompl::base::PlannerStatus status;

    ompl::time::point start_t = ompl::time::now();
    while (_plan)
    {
        // Plan for a small time
        status = _global_planner_method->solve(_solve_time);

        duration = ompl::time::seconds(ompl::time::now() - start_t);

        // Check if we reached the total planning timeout
        if (duration > _max_plan_time) break;
    }

    if (_plan)
    {
        // MUTEX
        _mutex.lock();

        // Get path
        std::shared_ptr<ompl::geometric::PathGeometric> path =
            std::static_pointer_cast<ompl::geometric::PathGeometric>(_planning_problem->getSolutionPath());
        // Copy because mutex does not cover planning and getPath may interfere
        if (path)
        {
            ROS_INFO_STREAM("Global plan found in " << duration << "s");

            _path_ompl_share = std::make_shared<ompl::geometric::PathGeometric>(*path);

            // Simplify for a small time --> don't do with ergodic trajectory!!
            if (_simplify && _global_planner_method->getName() != "Ergodic global planner") _simplifier->simplify(*_path_ompl_share, _simplify_time);

            _new_plan = true;
        }
        else
        {
            _path_ompl_share = nullptr;
            _new_plan        = false;
            ROS_ERROR_STREAM("No Plan found: " << status.asString());
        }
        updatePathEigen();

        _mutex.unlock();
        // MUTEX
    }
    else
    {
        ROS_INFO("Planning canceled");
    }
    _plan = false;
}

void RobotGlobalPlanner::stopPlanning()
{
    _plan     = false;
    _new_plan = false;
    if (_planning.joinable()) _planning.join();
}

void RobotGlobalPlanner::updatePathEigen()
{
    if (!_initialized) return;

    if (_path_ompl_share)
    {
        int n = _path_ompl_share->getStateCount();
        _path_eigen.clear();
        _path_eigen.resize(n, Eigen::VectorXd::Zero(_dim));

        for (int i = 0; i < n; ++i)
        {
            _path_eigen[i] =
                Eigen::Map<const Eigen::VectorXd>(_path_ompl_share->getState(i)->as<ompl::base::RealVectorStateSpace::StateType>()->values, _dim);
        }
    }
    else
    {
        _path_eigen_share = {};
    }
    _path_eigen_share = _path_eigen;
}

void RobotGlobalPlanner::requestPath()
{
    if (!_initialized) return;

    // Stop current planning thread
    stopPlanning();

    // Start new planning thread
    _plan     = true;
    _new_plan = false;
    _planning = std::thread(&RobotGlobalPlanner::plan, this);
}

bool RobotGlobalPlanner::getPath(std::vector<Eigen::VectorXd>& path_eigen, std::shared_ptr<ompl::geometric::PathGeometric>& path_ompl)
{
    bool tmp_new_plan = false;

    // MUTEX
    _mutex.lock();

    if (_path_ompl_share)
    {
        path_ompl  = std::make_shared<ompl::geometric::PathGeometric>(*_path_ompl_share);
        path_eigen = _path_eigen_share;

        tmp_new_plan = _new_plan;
        _new_plan    = false;
    }
    else
    {
        path_ompl  = nullptr;
        path_eigen = {};
    }
    _mutex.unlock();
    // MUTEX

    return tmp_new_plan;
}

bool RobotGlobalPlanner::initialize(robot_collision::RobotCollision::UPtr robot_collision, robot_kinematic::RobotKinematic::UPtr robot_kinematic)
{
    if (_initialized) return true;

    _robot_collision                  = std::move(robot_collision);
    const RobotUtility& robot_utility = _robot_collision->getRobotKinematic()->getRobotUtility();

    ROS_INFO("GlobalPlannerInterface: Initializing...");

    _dim         = robot_utility.getJointsCount();
    _joint_space = std::make_shared<ompl::base::RealVectorStateSpace>(_dim);

    ompl::base::RealVectorBounds bounds(_dim);
    for (int i = 0; i < _dim; ++i)
    {
        bounds.setLow(i, robot_utility.getJointMinLimit(i));
        bounds.setHigh(i, robot_utility.getJointMaxLimit(i));
    }

    _joint_space->setBounds(bounds);

    ompl::msg::setLogLevel(ompl::msg::LOG_NONE);

    _joint_space_info = std::make_shared<ompl::base::SpaceInformation>(_joint_space);
    _joint_space_info->setStateValidityChecker(std::bind(&RobotGlobalPlanner::isStateValid, this, std::placeholders::_1));
    _joint_space_info->setStateValidityCheckingResolution(_validity_resolution);  //  less or equal 0.05

    _planning_problem = std::make_shared<ompl::base::ProblemDefinition>(_joint_space_info);

    if (_path_objective == TASKSPACE)
    {
        _objective = std::make_shared<RobotPathObjectiveTaskSpace>(_joint_space_info, std::move(robot_kinematic));
        _objective->setCostThreshold(ompl::base::Cost(_max_path_length));
        _planning_problem->setOptimizationObjective(_objective);
    }
    else if (_path_objective == JOINTSPACE)
    {
        _objective = std::make_shared<ompl::base::PathLengthOptimizationObjective>(_joint_space_info);
        _objective->setCostThreshold(ompl::base::Cost(_max_path_length));
        _planning_problem->setOptimizationObjective(_objective);
    }
    else
    {
        _path_objective = NONE;
        std::cout << "No Objective" << std::endl;
    }

    if (!_global_planner_method)
    {
        ROS_ERROR("RobotGlobalPlanner: No global planner method instantiated.");
        return false;
    }

    _global_planner_method->initialize(_joint_space_info);

    if (!_joint_space_info->isSetup()) _joint_space_info->setup();

    _global_planner_method->setProblemDefinition(_planning_problem);

    if (!_global_planner_method->isSetup()) _global_planner_method->setup();

    _initialized = true;

    return true;
}

void RobotGlobalPlanner::clearPlan()
{
    if (!_initialized) return;

    // Stop current planning thread
    stopPlanning();

    _global_planner_method->clearPlan();
    _planning_problem->clearSolutionPaths();

    //_planning_problem->clearStartStates();
    //_planning_problem->clearGoal();
}

void RobotGlobalPlanner::clearQuery()
{
    if (!_initialized) return;

    // Stop current planning thread
    stopPlanning();

    _global_planner_method->clearQuery();

    _planning_problem->clearSolutionPaths();
    _planning_problem->clearStartStates();
    _planning_problem->clearGoal();
}

void RobotGlobalPlanner::shutdown() { stopPlanning(); }

void RobotGlobalPlanner::update()
{
    _obstacle_manager._mutex.lock();
    _static_obstacles  = _obstacle_manager._static_obstacles;
    _dynamic_obstacles = _obstacle_manager._dynamic_obstacles;
    _planes            = _obstacle_manager._planes;
    _obstacle_manager._mutex.unlock();
}

void RobotGlobalPlanner::setStartAndGoalState(const Eigen::Ref<const Eigen::VectorXd>& start, const Eigen::Ref<const Eigen::VectorXd>& goal)
{
    if (!_initialized) return;

    stopPlanning();

    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> start_state(_joint_space);
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> goal_state(_joint_space);

    for (int i = 0; i < _dim; ++i)
    {
        start_state[i] = start(i);
        goal_state[i]  = goal(i);
    }

    _planning_problem->setStartAndGoalStates(start_state, goal_state);

    _simplifier = std::make_shared<ompl::geometric::PathSimplifier>(_joint_space_info, _planning_problem->getGoal());
}

void RobotGlobalPlanner::setPathObjective(const RobotGlobalPlanner::PathObjective& objective) { _path_objective = objective; }

const RobotGlobalPlanner::PathObjective& RobotGlobalPlanner::getPathObjective() const { return _path_objective; }

bool RobotGlobalPlanner::isStateValid(const ompl::base::State* state)
{
    Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(state->as<ompl::base::RealVectorStateSpace::StateType>()->values, _dim);

    _robot_collision->setJointState(q);

    if (_robot_collision->getMinSelfCollisionDistance() < _min_self_collision) return false;

    if (_robot_collision->getMinGroundCollisionDistance() < _min_ground_collision) return false;

    if (_robot_collision->getMinRoofCollisionDistance() < _min_roof_collision) return false;

    for (int i = 0; i < (int)_static_obstacles.size(); ++i)
    {
        if (_robot_collision->getMinObstacleDistance(_static_obstacles[i]) < _min_obstacle_collision) return false;
    }

    for (int i = 0; i < (int)_planes.size(); ++i)
    {
        if (_robot_collision->getMinPlaneCollisionDistance(_planes[i]) < _min_plane_collision) return false;
    }

    for (int i = 0; i < (int)_dynamic_obstacles.size(); ++i)
    {
        if (_dynamic_obstacles[i].temporary_static)
        {
            if (_robot_collision->getMinObstacleDistance(_dynamic_obstacles[i]) < _min_obstacle_collision) return false;
        }
    }

    return true;
}

std::string RobotGlobalPlanner::getName() const { return _global_planner_method->getName(); }
bool RobotGlobalPlanner::getDeltaTime(double& deltat) const {
    deltat =  _planning_problem->getSolutionDifference();
    return true;
}
}  // namespace robot_path_pilot
}  // namespace mhp_robot
