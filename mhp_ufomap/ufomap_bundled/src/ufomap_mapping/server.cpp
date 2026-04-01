/**
 * UFOMap Mapping
 *
 * @author D. Duberg, KTH Royal Institute of Technology, Copyright (c) 2020.
 * @see https://github.com/UnknownFreeOccupied/ufomap_mapping
 * License: BSD 3
 *
 * Modifier Heiko Renz, TU Dortmund University, Germany
 *
 */

/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2020, D. Duberg, KTH Royal Institute of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// UFO
#include <ufomap_bundled/UFOMapStamped.h>

#include <ufomap_bundled/ufomap_mapping/server.h>
#include <ufomap_bundled/ufomap_msgs/conversions.h>
#include <ufomap_bundled/ufomap_ros/conversions.h>

// STD
#include <chrono>
#include <future>
#include <numeric>

namespace ufomap_mapping
{
Server::Server(ros::NodeHandle& nh, ros::NodeHandle& nh_priv, int num_startpoints, int num_scaling_gridpoints,
               int buffer_size)
  : _nh(nh)
  , _nh_priv(nh_priv)
  , _tf_listener(_tf_buffer)
  , _cs(nh_priv)
  , _num_startpoints_random(num_startpoints)
  , _buffer_size(buffer_size)  // NOLINT
{
  // Set up map
  double resolution = _nh_priv.param("resolution", 0.05);
  _resolution = resolution;
  ufo::map::DepthType depth_levels = _nh_priv.param("depth_levels", 16);

  // Information distribution extension
  _information_distribution = _nh_priv.param("information_distribution", false);
  _information_distribution_gpu = _nh_priv.param("information_distribution_gpu", false);
  _buffer_pcl = _nh_priv.param("buffer_pcl", false);
  std::vector<double> poi_world = _nh_priv.param("poi_world", std::vector<double>{ 1.37, 0.16, 0.2 });
  _poi_world = ufo::map::Point3(poi_world[0], poi_world[1], poi_world[2]);
  std::cout << "POI World: " << _poi_world.x() << " " << _poi_world.y() << " " << _poi_world.z() << std::endl;

  _poi_pub = _nh_priv.advertise<visualization_msgs::Marker>("poi", 100, false);
  _max_occ_point_pub = _nh_priv.advertise<visualization_msgs::Marker>("target", 100, false);
  _max_occ_task_space_target_pub = _nh_priv.advertise<geometry_msgs::PoseStamped>("occlusion_target", 1, false);

  _error_pub = _nh_priv.advertise<mhp_robot::MsgErrorTracking>("errors_tracking", 100, false);

  plotPoi();

  // Occlusions extension
  _occlusion_distribution = _nh_priv.param("occlusion_distribution", false);
  _occlusion_distribution_gpu = _nh_priv.param("occlusion_distribution_gpu", false);

  _pred_steps_occ = _nh_priv.param("/state_estimator/extrapolation_steps", 30) + 1;

  if (_occlusion_distribution && _video_target && !_skeleton_target && !_target_switch_mode)
  {
    ROS_WARN_ONCE("Occlusion distribution is enabled; Get target from video");
    _target_sub = _nh_priv.subscribe("/hand_com_mp", 1, &Server::targetCallback, this);
  }
  else if (_occlusion_distribution && _skeleton_target && !_video_target && !_target_switch_mode)
  {
    _hand_com_pub = _nh_priv.advertise<geometry_msgs::PoseStamped>("/hand_com", 10, false);
    ROS_WARN_ONCE("Occlusion distribution is enabled; Get target from skeleton");
  }
  else if (_occlusion_distribution && _target_switch_mode)
  {
    _hand_com_pub = _nh_priv.advertise<geometry_msgs::PoseStamped>("/hand_com", 10, false);
    _target_sub = _nh_priv.subscribe("/hand_com_mp", 1, &Server::targetCallback, this);
    ROS_WARN_ONCE("Occlusion distribution is enabled; Get target from video and skeleton (switch mode active)");
  }
  else
  {
    ROS_WARN_ONCE("Occlusion distribution is disabled");
  }

  // Subscribe for obstacles
  _virtual_obstacle_sub = _nh_priv.subscribe("/robot_workspace_monitor/obstacles", 1, &Server::obstacleCallback, this,
                                             ros::TransportHints().tcpNoDelay());

  // Automatic pruning is disabled so we can work in multiple threads for subscribers,
  // services and publishers
  // Copy to GPU if desired
  if (_nh_priv.param("color_map", false))
  {
    ROS_WARN_ONCE("Color map is enabled");
    if (_information_distribution_gpu || _occlusion_distribution_gpu)
    {
      ROS_WARN_ONCE("GPU support activated for information distribution and/or occlusion calculation");
      _map.emplace<ufo::map::OccupancyMapColor>(resolution, depth_levels, false);
      std::visit(
          [this](auto& map) {
            if constexpr (std::is_same_v<std::decay_t<decltype(map)>, ufo::map::OccupancyMapColor>)
            {
              ROS_WARN_ONCE("Copy map to GPU");
              _map_gpu.copyMapCPUtoGPU(&map);
            }
          },
          _map);
    }
    else
    {
      ROS_WARN_ONCE("No GPU support activated");
      _map.emplace<ufo::map::OccupancyMapColor>(resolution, depth_levels, false);
    }
  }
  else
  {
    ROS_WARN_ONCE("Color map is disabled");
    ROS_WARN_ONCE("No GPU support without color Map");
    _map.emplace<ufo::map::OccupancyMap>(resolution, depth_levels, false);
  }
  // Enable min/max change detection
  std::visit(
      [this](auto& map) {
        if constexpr (!std::is_same_v<std::decay_t<decltype(map)>, std::monostate>)
        {
          map.enableMinMaxChangeDetection(true);
        }
      },
      _map);

  // Set up dynamic reconfigure server
  _cs.setCallback(boost::bind(&Server::configCallback, this, _1, _2));

  // Set up publisher
  _info_pub = _nh_priv.advertise<diagnostic_msgs::DiagnosticStatus>("info", 10, false);
  _info_arrow_pub =
      _nh_priv.advertise<visualization_msgs::Marker>("info_dist_points", 1, false);  // Plotting arrows for each ray
  _occ_arrow_pub =
      _nh_priv.advertise<visualization_msgs::Marker>("occ_dist_points", 1, false);  // Plotting arrows for each ray
  _start_pub = _nh_priv.advertise<visualization_msgs::Marker>("info_dist_points_start", 1,
                                                              false);  // Plotting markers for each start point
  _info_dist_pub = _nh_priv.advertise<ufomap_bundled::MsgInfoDist>("info_dist", 10, false);  //  Publishing information
                                                                                             //  gains for start points
  _max_info_point_pub = _nh_priv.advertise<geometry_msgs::Pose>("info_dist_points_max", 1,
                                                                false);  // Send point for max information point
  _max_info_point_pub_vis = _nh_priv.advertise<visualization_msgs::Marker>(
      "info_dist_points_max_vis", 1, false);  // Send point for max information point
  _occlusion_distribution_pub = _nh_priv.advertise<mhp_robot::MsgOcclusionDist>("occlusion_dist", 10, false);

  //  Publishing information gains for start points
  // Buffering information distribution
  if (_buffer_pcl)
  {
    _info_dist_pub_all = _nh_priv.advertise<mhp_robot::MsgInfoPCLS>("info_dist_cloud", 1, false);

    for (int i = 0; i < _buffer_size; i++)
    {
      std::string name = "info_dist_cloud_" + std::to_string(i);
      _info_dist_pub_cloud[i] = _nh_priv.advertise<sensor_msgs::PointCloud2>(
          name, 10, false);  //  Publishing information gains for start points
    }
  }
  else  // Publish only the last information distribution
  {
    _info_dist_pub_cloud[0] = _nh_priv.advertise<sensor_msgs::PointCloud2>(
        "info_dist_cloud", 10, false);  //  Publishing information gains for start points
  }
  _gpu_time_pub = _nh_priv.advertise<diagnostic_msgs::DiagnosticStatus>(
      "info_gpu_times", 10, false);  //  Publishing information gains for start points

  // Enable services
  _get_map_server = _nh_priv.advertiseService("get_map", &Server::getMapCallback, this);
  _clear_volume_server = _nh_priv.advertiseService("clear_volume", &Server::clearVolumeCallback, this);
  _set_volume_server = _nh_priv.advertiseService("set_volume", &Server::setVolumeCallback, this);
  _reset_server = _nh_priv.advertiseService("reset", &Server::resetCallback, this);
  _save_map_server = _nh_priv.advertiseService("save_map", &Server::saveMapCallback, this);

  _scaling_factor_gridpoints = num_scaling_gridpoints;

  // Update human obstacle list:
  _obstacle_manager._mutex.lock();
  _human_obstacle_list = _obstacle_manager._humans;
  _static_obstacle_list = _obstacle_manager._static_obstacles;
  _dynamic_obstacle_list = _obstacle_manager._dynamic_obstacles;
  _obstacle_manager._mutex.unlock();
}

Server::~Server()
{
  _map_gpu.clearMapGPU();
  // _map_gpu_occ.clearMapGPU();
}

void Server::cloudCallback(sensor_msgs::PointCloud2::ConstPtr const& msg)
{
  ufo::math::Pose6 transform;
  try
  {
    transform = ufomap_ros::rosToUfo(
        _tf_buffer.lookupTransform(_frame_id, msg->header.frame_id, msg->header.stamp, _transform_timeout).transform);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN_THROTTLE(1, "%s", ex.what());
    return;
  }

  // Update human obstacle list:
  _obstacle_manager._mutex.lock();
  _human_obstacle_list = _obstacle_manager._humans;
  _static_obstacle_list = _obstacle_manager._static_obstacles;
  _dynamic_obstacle_list = _obstacle_manager._dynamic_obstacles;
  _obstacle_manager._mutex.unlock();

  // Get target from skeleton (video has a callback)
  std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
  std::chrono::steady_clock::time_point start_all = std::chrono::steady_clock::now();

  if (_skeleton_target && !_target_switch_mode)
  {
    ROS_INFO_THROTTLE(10, "Server: Get target from skeleton");
    calculateTargetFromPoint(msg->header);
  }
  else if (_skeleton_target && _target_switch_mode)
  {
    if (!_video_target)
    {
      ROS_INFO_THROTTLE(10, "Server: Get target from skeleton (switch mode active)");
      calculateTargetFromPoint(msg->header);
    }
    else
    {
      ROS_ERROR("Server: Both video and skeleton target are false, no target available --> This should not happen");
    }
  }
  else if (_video_target && _target_switch_mode)
  {
    if ((ros::Time::now() - _video_target_start_time).toSec() > _video_target_duration)
    {
      _video_target = false;
      _skeleton_target = true;
      ROS_WARN("Switch to skeleton target since MP solution is more than %f sec old",
               (ros::Time::now() - _video_target_start_time).toSec());
      calculateTargetFromPoint(msg->header);
    }
    else
    {
      ROS_WARN_THROTTLE(1, "Using video target since it is only %f sec old",
                        (ros::Time::now() - _video_target_start_time).toSec());
    }
  }

  // Time consumption target calculation
  double target_time =
      std::chrono::duration<float, std::chrono::seconds::period>(std::chrono::steady_clock::now() - start).count();

  if (0 == _num_targets || target_time < _min_target_time)
  {
    _min_target_time = target_time;
  }
  if (target_time > _max_target_time)
  {
    _max_target_time = target_time;
  }
  _accumulated_target_time += target_time;
  ++_num_targets;

  start = std::chrono::steady_clock::now();
  // Set static obstacles
  if (!_static_obstacles_added && !_static_obstacle_list.empty())
  {
    if (!setVolumeStaticObstacle())
    {
      ROS_WARN(
          "Ufomap Server: Could not set static obstacle model; No occlusion avoidance for future static obstacles");
    }
  }

  // Set dynamic obstacles (each cycle) TODO(renz): Add flag that this is only executed if the obstacle in the list
  // change
  if (!_dynamic_obstacle_list.empty())
  {
    if (!setVolumeDynamicObstacle())
    {
      ROS_WARN(
          "Ufomap Server: Could not set dynamic obstacle model; No occlusion avoidance for future dynamic obstacles");
    }
  }

  // Add human obstacle model to map
  if (!_human_obstacle_list.empty())
  {
    if (!setVolumeHuman())
    {
      ROS_WARN("Ufomap Server: Could not set human obstacle model; No occlusion avoidance for future human poses");
    }
  }

  double obstacle_time =
      std::chrono::duration<float, std::chrono::seconds::period>(std::chrono::steady_clock::now() - start).count();
  if (0 == _num_obstacles || obstacle_time < _min_obstacle_time)
  {
    _min_obstacle_time = obstacle_time;
  }
  if (obstacle_time > _max_obstacle_time)
  {
    _max_obstacle_time = obstacle_time;
  }
  _accumulated_obstacle_time += obstacle_time;
  ++_num_obstacles;

  // _map_mutex.lock();
  std::visit(
      [this, &msg, &transform](auto& map) {
        if constexpr (std::is_same_v<std::decay_t<decltype(map)>, ufo::map::OccupancyMapColor>)
        // colorOccupancyMap or
        // OccupancyMap
        {
          std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

          // Update map
          ufo::map::PointCloud cloud;  // Use black PCL since colored values are used for obstacles in the near future
          // ufo::map::PointCloudColor cloud;
          // ufomap_ros::rosToUfo(*msg, cloud);
          // cloud.push_back(ufo::map::Point3(0.0, 0.0, -10.0));
          cloud.transform(transform, true);

          map.insertPointCloudDiscrete(transform.translation(), cloud, _max_range, _insert_depth, _simple_ray_casting,
                                       _early_stopping, _async);

          double integration_time =
              std::chrono::duration<float, std::chrono::seconds::period>(std::chrono::steady_clock::now() - start)
                  .count();

          if (0 == _num_integrations || integration_time < _min_integration_time)
          {
            _min_integration_time = integration_time;
          }
          if (integration_time > _max_integration_time)
          {
            _max_integration_time = integration_time;
          }
          _accumulated_integration_time += integration_time;
          ++_num_integrations;

          // Clear robot
          if (_clear_robot)
          {
            start = std::chrono::steady_clock::now();

            try
            {
              transform = ufomap_ros::rosToUfo(
                  _tf_buffer.lookupTransform(_frame_id, _robot_frame_id, msg->header.stamp, _transform_timeout)
                      .transform);
            }
            catch (tf2::TransformException& ex)
            {
              ROS_WARN_THROTTLE(1, "%s", ex.what());
              return;
            }

            ufo::map::Point3 r(_robot_radius, _robot_radius, _robot_height / 2.0);
            ufo::geometry::AABB aabb(transform.translation() - r, transform.translation() + r);
            map.setValueVolume(aabb, map.getClampingThresMin(), _clearing_depth);

            double clear_time =
                std::chrono::duration<float, std::chrono::seconds::period>(std::chrono::steady_clock::now() - start)
                    .count();
            if (0 == _num_clears || clear_time < _min_clear_time)
            {
              _min_clear_time = clear_time;
            }
            if (clear_time > _max_clear_time)
            {
              _max_clear_time = clear_time;
            }
            _accumulated_clear_time += clear_time;
            ++_num_clears;
          }

          // Publish update
          if (!_map_pub.empty() && _update_part_of_map && (map.validMinMaxChange() || validMinMaxChange()) &&
              (!_last_update_time.isValid() || (msg->header.stamp - _last_update_time) >= _update_rate))
          {
            bool can_update = true;
            if (_update_async_handler.valid())
            {
              can_update = std::future_status::ready == _update_async_handler.wait_for(std::chrono::seconds(0));
            }

            if (can_update)
            {
              _last_update_time = msg->header.stamp;
              start = std::chrono::steady_clock::now();
              ufo::map::Point3 min_change = map.minChange();
              ufo::map::Point3 max_change = map.maxChange();

              min_change.x() = std::min(min_change.x(), _min_change_for_obs.x());
              min_change.y() = std::min(min_change.y(), _min_change_for_obs.y());
              min_change.z() = std::min(min_change.z(), _min_change_for_obs.z());

              max_change.x() = std::max(max_change.x(), _max_change_for_obs.x());
              max_change.y() = std::max(max_change.y(), _max_change_for_obs.y());
              max_change.z() = std::max(max_change.z(), _max_change_for_obs.z());

              // ufo::map::Point3 min_change = ufo::map::Point3(-10.0, -10.0, -10.0);
              // ufo::map::Point3 max_change = ufo::map::Point3(10.0, 10.0, 10.0);
              // ufo::geometry::AABB aabb(map.minChange(), map.maxChange());
              ufo::geometry::AABB aabb(min_change, max_change);
              // TODO(UNKNOWN): should this be here?
              map.resetMinMaxChangeDetection();

              _update_async_handler = std::async(std::launch::async, [this, aabb, stamp = msg->header.stamp]() {
                std::visit(
                    [this, &aabb, stamp](auto& map) {
                      if constexpr (!std::is_same_v<std::decay_t<decltype(map)>, std::monostate>)
                      {
                        for (int i = 0; i < _map_pub.size(); ++i)
                        {
                          if (_map_pub[i] && (0 < _map_pub[i].getNumSubscribers() || _map_pub[i].isLatched()))
                          {
                            ufomap_bundled::UFOMapStamped::Ptr msg(new ufomap_bundled::UFOMapStamped);
                            if (ufomap_bundled::ufoToMsg(map, msg->map, aabb, _compress, i))
                            {
                              msg->header.stamp = stamp;
                              msg->header.frame_id = _frame_id;
                              _map_pub[i].publish(msg);
                            }
                          }
                        }
                      }
                    },
                    _map);
              });

              double update_time =
                  std::chrono::duration<float, std::chrono::seconds::period>(std::chrono::steady_clock::now() - start)
                      .count();
              if (0 == _num_updates || update_time < _min_update_time)
              {
                _min_update_time = update_time;
              }
              if (update_time > _max_update_time)
              {
                _max_update_time = update_time;
              }
              _accumulated_update_time += update_time;
              ++_num_updates;
            }
          }
          // Calculate information and/or occlusion distribution if enabled
          start = std::chrono::steady_clock::now();

          calculateInformationAndOcclusionDistributions(map, msg);
          double occlusion_time =
              std::chrono::duration<float, std::chrono::seconds::period>(std::chrono::steady_clock::now() - start)
                  .count();
          if (0 == _num_occlusions || occlusion_time < _min_occlusion_time)
          {
            _min_occlusion_time = occlusion_time;
          }
          if (occlusion_time > _max_occlusion_time)
          {
            _max_occlusion_time = occlusion_time;
          }
          _accumulated_occlusion_time += occlusion_time;
          ++_num_occlusions;


          // Calculate errors for evaluations
          start = std::chrono::steady_clock::now();

          if (_error_calculation)
          {
            // get camera pose from tf if available and start point is set
            if (_tf_buffer.canTransform(_frame_id, "camera_3d_depth_camera_link", ros::Time::now(),
                                        ros::Duration(0.05)) &&
                _start_points_occlusion.size() > 0)
            {
              tf::transformMsgToEigen(
                  _tf_buffer
                      .lookupTransform(_frame_id, "camera_3d_depth_camera_link", ros::Time::now(), ros::Duration(0.05))
                      .transform,
                  _camera_pose);

              // Get tracking point of interest
              Eigen::Vector3d tracking_poi = Eigen::Vector3d{ std::get<1>(_start_points_occlusion[0]).x(),
                                                              std::get<1>(_start_points_occlusion[0]).y(),
                                                              std::get<1>(_start_points_occlusion[0]).z() };

              // get distance camera to startpoint occlusion
              _distance_cam_tracking = (_camera_pose.translation() - tracking_poi).norm();

              // get orientation deviation from ideal orientation toward tracking_poi
              // First get the z-axis in world coordinates
              _z_axis_world = _camera_pose.matrix() * Eigen::Vector4d{ 0, 0, 1, 1 };

              // // get the scalar product between the camera and the POI axis
              double scalar_product = Eigen::Vector3d{ tracking_poi[0] - _camera_pose.matrix()(0, 3),
                                                       tracking_poi[1] - _camera_pose.matrix()(1, 3),
                                                       tracking_poi[2] - _camera_pose.matrix()(2, 3) }
                                          .dot(Eigen::Vector3d{ _z_axis_world[0] - _camera_pose.matrix()(0, 3),
                                                                _z_axis_world[1] - _camera_pose.matrix()(1, 3),
                                                                _z_axis_world[2] - _camera_pose.matrix()(2, 3) }) /
                                      (Eigen::Vector3d{ tracking_poi[0] - _camera_pose.matrix()(0, 3),
                                                        tracking_poi[1] - _camera_pose.matrix()(1, 3),
                                                        tracking_poi[2] - _camera_pose.matrix()(2, 3) }
                                           .norm() *
                                       Eigen::Vector3d{ _z_axis_world[0] - _camera_pose.matrix()(0, 3),
                                                        _z_axis_world[1] - _camera_pose.matrix()(1, 3),
                                                        _z_axis_world[2] - _camera_pose.matrix()(2, 3) }
                                           .norm());
              _angle_cam_tracking = acos(scalar_product) * 180.0 / M_PI;

              // Check if the line of sight from camera to tracking PoI is clear in UFOmap
              std::vector<ufo::map::Code> rayCodes;
              rayCodes = map.computeRay(ufo::map::Point3(_camera_pose.translation().x(), _camera_pose.translation().y(),
                                                         _camera_pose.translation().z()),
                                        ufo::map::Point3(tracking_poi.x(), tracking_poi.y(), tracking_poi.z()));
              if (rayCodes.size() > 0)
              {
                // Check if the ray codes are all free
                bool line_of_sight_occupied = false;
                ufo::map::Color color;
                for (const auto& code : rayCodes)
                {
                  color = map.getColor(code);

                  line_of_sight_occupied = (map.isOccupied(code) && color.r != 255);

                  if (line_of_sight_occupied)
                  {
                    break;
                  }
                }
                _line_of_sight_occluded = line_of_sight_occupied;
                _distance_poi_to_tracking =
                    (Eigen::Vector3d{ _poi_world.x() - tracking_poi.x(), _poi_world.y() - tracking_poi.y(),
                                      _poi_world.z() - tracking_poi.z() })
                        .norm();
                // Send errors to topic
                mhp_robot::MsgErrorTracking error_msg;
                error_msg.header.stamp = msg->header.stamp;
                error_msg.header.frame_id = _frame_id;
                error_msg.distance_cam_to_tracking = _distance_cam_tracking;
                error_msg.angle_cam_to_tracking = _angle_cam_tracking;
                error_msg.line_of_sight_occluded = _line_of_sight_occluded;
                error_msg.distance_poi_to_tracking = _distance_poi_to_tracking;
                _error_pub.publish(error_msg);
              }
            }
          }
         
          double error_time =
              std::chrono::duration<float, std::chrono::seconds::period>(std::chrono::steady_clock::now() - start)
                  .count();
          if (0 == _num_errors || error_time < _min_error_time)
          {
            _min_error_time = error_time;
          }
          if (error_time > _max_error_time)
          {
            _max_error_time = error_time;
          }
          _accumulated_error_time += error_time;
          ++_num_errors;


        }
      },
      _map);
  // _map_mutex.unlock();
  // Clear human obstacle model
  start = std::chrono::steady_clock::now();

  if (!_dynamic_obstacle_list.empty())
  {
    if (!resetVolumeDynamicObstacle())
    {
      ROS_WARN("Ufomap Server: Could not reset dynamic obstacle model");
    }
  }

  if (!_human_obstacle_list.empty())
  {
    if (!resetVolumeHuman())
    {
      ROS_WARN("Ufomap Server: Could not reset human obstacle model");
    }
  }

  double delete_time =
      std::chrono::duration<float, std::chrono::seconds::period>(std::chrono::steady_clock::now() - start)
          .count();
  if (0 == _num_deletes || delete_time < _min_delete_time)
  {
    _min_delete_time = delete_time;
  }
  if (delete_time > _max_delete_time)
  {
    _max_delete_time = delete_time;
  }
  _accumulated_delete_time += delete_time;
  ++_num_deletes;  

  double all_time =
      std::chrono::duration<float, std::chrono::seconds::period>(std::chrono::steady_clock::now() - start_all)
          .count();
  if (0 == _num_alls || all_time < _min_all_time)
  {
    _min_all_time = all_time;
  }
  if (all_time > _max_all_time)
  {
    _max_all_time = all_time;
  }
  _accumulated_all_time += all_time;
  ++_num_alls;  

  // Publish all time information
  publishInfo();

}

void Server::publishInfo()
{
  if (_verbose)
  {
    printf("\nTimings:\n");
    if (0 != _num_integrations)
    {
      printf("\tIntegration time (s): %5d %09.6f\t(%09.6f +- %09.6f)\n", _num_integrations,
             _accumulated_integration_time, _accumulated_integration_time / _num_integrations, _max_integration_time);
    }
    if (0 != _num_clears)
    {
      printf("\tClear time (s):       %5d %09.6f\t(%09.6f +- %09.6f)\n", _num_clears, _accumulated_clear_time,
             _accumulated_clear_time / _num_clears, _max_clear_time);
    }
    if (0 != _num_updates)
    {
      printf("\tUpdate time (s):      %5d %09.6f\t(%09.6f +- %09.6f)\n", _num_updates, _accumulated_update_time,
             _accumulated_update_time / _num_updates, _max_update_time);
    }
    if (0 != _num_wholes)
    {
      printf("\tWhole time (s):       %5d %09.6f\t(%09.6f +- %09.6f)\n", _num_wholes, _accumulated_whole_time,
             _accumulated_whole_time / _num_wholes, _max_whole_time);
    }

  }

  if (_info_pub && 0 < _info_pub.getNumSubscribers())
  {
    diagnostic_msgs::DiagnosticStatus msg;
    msg.level = diagnostic_msgs::DiagnosticStatus::OK;
    msg.name = "UFOMap mapping timings";
    msg.values.resize(13);
    msg.values[0].key = "Min integration time (s); Max integration time (s); Average integration time (s);";
    msg.values[0].value = std::to_string(_min_integration_time) + "; " + std::to_string(_max_integration_time) + "; " +
                        std::to_string(_accumulated_integration_time / _num_integrations);
    // msg.values[1].key = "Max integration time (s)";
    // msg.values[1].value = std::to_string(_max_integration_time);
    // msg.values[2].key = "Average integration time (s)";
    // msg.values[2].value = std::to_string(_accumulated_integration_time / _num_integrations);
    msg.values[1].key = "Min clear time (s); Max clear time (s); Average clear time (s);";
    msg.values[1].value = std::to_string(_min_clear_time) + "; " + std::to_string(_max_clear_time) + "; " +
                        std::to_string(_accumulated_clear_time / _num_clears);
    msg.values[2].key = "Min update time (s); Max update time (s); Average update time (s);";
    msg.values[2].value = std::to_string(_min_update_time) + "; " + std::to_string(_max_update_time) + "; " +
                        std::to_string(_accumulated_update_time / _num_updates);
    msg.values[3].key = "Min distribution (no postprocessing) time (s); Max distribution time (s); Average distribution time (s);";
    msg.values[3].value = std::to_string(_min_distribution_time) + "; " + std::to_string(_max_distribution_time) + "; " +
                        std::to_string(_accumulated_distribution_time / _num_distributions);
    msg.values[4].key = "Min whole time (s); Max whole time (s); Average whole time (s);";
    msg.values[4].value = std::to_string(_min_whole_time) + "; " + std::to_string(_max_whole_time) + "; " +
                        std::to_string(_accumulated_whole_time / _num_wholes);
    msg.values[5].key = "Min perspective time (s); Max perspective time (s); Average perspective time (s);";
    msg.values[5].value = std::to_string(_min_perspective_time) + "; " + std::to_string(_max_perspective_time) + "; " +
                        std::to_string(_accumulated_perspective_time / _num_perspectives);
    msg.values[6].key = "Min distribution GPU time (s); Max distribution GPU time (s); Average distribution GPU time (s);";
    msg.values[6].value = std::to_string(_min_distribution_gpu_time) + "; " + std::to_string(_max_distribution_gpu_time) + "; " +
                        std::to_string(_accumulated_distribution_gpu_time / _num_distributions_gpu);
    msg.values[7].key = "Min target time (s); Max target time (s); Average target time (s);";
    msg.values[7].value = std::to_string(_min_target_time) + "; " + std::to_string(_max_target_time) + "; " +
                        std::to_string(_accumulated_target_time / _num_targets);
    msg.values[8].key = "Min obstacle time (s); Max obstacle time (s); Average obstacle time (s);";
    msg.values[8].value = std::to_string(_min_obstacle_time) + "; " + std::to_string(_max_obstacle_time) + "; " +
                        std::to_string(_accumulated_obstacle_time / _num_obstacles);
    msg.values[9].key = "Min occlusion time (s); Max occlusion time (s); Average occlusion time (s);";
    msg.values[9].value = std::to_string(_min_occlusion_time) + "; " + std::to_string(_max_occlusion_time) + "; " +
                        std::to_string(_accumulated_occlusion_time / _num_occlusions);
    msg.values[10].key = "Min error calculation time (s); Max error calculation time (s); Average error calculation time (s);";
    msg.values[10].value = std::to_string(_min_error_time) + "; " + std::to_string(_max_error_time) + "; " +
                        std::to_string(_accumulated_error_time / _num_errors);
    msg.values[11].key = "Min delete time (s); Max delete time (s); Average delete time (s);";
    msg.values[11].value = std::to_string(_min_delete_time) + "; " + std::to_string(_max_delete_time) + "; " +
                        std::to_string(_accumulated_delete_time / _num_deletes);

    msg.values[12].key = "Min whole time (s); Max whole time (s); Average whole time (s);";
    msg.values[12].value = std::to_string(_min_all_time) + "; " + std::to_string(_max_all_time) + "; " +
                        std::to_string(_accumulated_all_time / _num_alls);
    _info_pub.publish(msg);
  }
}

void Server::publishInfoGPU()
{
  if (_verbose)
  {
    printf("\nTimings:\n");
    if (!_time_metric_vec[0] == 0.0)
    {
      printf("\tCopy map host to device (averaged over all start points) [s]: %5f \n", _time_metric_vec[0]);
      printf("\tCopy information distribution variables host to device [s]: %5f \n", _time_metric_vec[1]);
      printf("\tKernel time to process all rays [s]: %5f \n", _time_metric_vec[2]);
      printf("\tCopy device results to host time [s]: %5f \n", _time_metric_vec[3]);
      printf("\tData transfer time (Host to device and device to host) [s]: %5f \n", _time_metric_vec[4]);
      printf("\tTotal distribution time [s]: %5f \n", _time_metric_vec[5]);
    }
  }
  if (_gpu_time_pub && 0 < _gpu_time_pub.getNumSubscribers())
  {
    diagnostic_msgs::DiagnosticStatus msg;
    msg.level = diagnostic_msgs::DiagnosticStatus::OK;
    msg.name = "UFOMap GPU information distribution timings";
    msg.values.resize(12);
    // msg.values[0].key = "Min Copy map Host to device time [s]";
    // msg.values[0].value = std::to_string(_min_info_metric_gpu[0]);
    // msg.values[1].key = "Min Copy information variables Host to device time [s]";
    // msg.values[1].value = std::to_string(_min_info_metric_gpu[1]);
    // msg.values[2].key = "Min Kernel time to process all rays [s]";
    // msg.values[2].value = std::to_string(_min_info_metric_gpu[2]);
    // msg.values[3].key = "Min Copy device results to host time [s]";
    // msg.values[3].value = std::to_string(_min_info_metric_gpu[3]);
    // msg.values[4].key = "Min Data transfer time (Host to device and device to host) [s]";
    // msg.values[4].value = std::to_string(_min_info_metric_gpu[4]);
    // msg.values[5].key = "Min Total distribution time [s]";
    // msg.values[5].value = std::to_string(_min_info_metric_gpu[5]);

    // msg.values[6].key = "Max Copy Map Host to device time [s]";
    // msg.values[6].value = std::to_string(_max_info_metric_gpu[0]);
    // msg.values[7].key = "Max Copy information variables Host to device time [s]";
    // msg.values[7].value = std::to_string(_max_info_metric_gpu[1]);
    // msg.values[8].key = "Max Kernel time to process all rays [s]";
    // msg.values[8].value = std::to_string(_max_info_metric_gpu[2]);
    // msg.values[9].key = "Max Copy device results to host time [s]";
    // msg.values[9].value = std::to_string(_max_info_metric_gpu[3]);
    // msg.values[10].key = "Max Data transfer time (Host to device and device to host) [s]";
    // msg.values[10].value = std::to_string(_max_info_metric_gpu[4]);
    // msg.values[11].key = "Max Total distribution time [s]";
    // msg.values[11].value = std::to_string(_max_info_metric_gpu[5]);

    msg.values[0].key = "Average Copy map Host to device time [s]";
    msg.values[0].value = std::to_string(_accumulated_info_metric_gpu[0] / _num_info_metrics_gpu);
    msg.values[1].key = "Average Copy information variables Host to device time [s]";
    msg.values[1].value = std::to_string(_accumulated_info_metric_gpu[1] / _num_info_metrics_gpu);
    msg.values[2].key = "Average information Kernel time to process all rays [s]";
    msg.values[2].value = std::to_string(_accumulated_info_metric_gpu[2] / _num_info_metrics_gpu);
    msg.values[3].key = "Average Copy device information results to host time [s]";
    msg.values[3].value = std::to_string(_accumulated_info_metric_gpu[3] / _num_info_metrics_gpu);
    msg.values[4].key = "Average information Data transfer time (Host to device and device to host) [s]";
    msg.values[4].value = std::to_string(_accumulated_info_metric_gpu[4] / _num_info_metrics_gpu);
    msg.values[5].key = "Average Total distribution time [s]";
    msg.values[5].value = std::to_string((_accumulated_info_metric_gpu[5]) / _num_info_metrics_gpu);

    msg.values[6].key = "Average Copy map Host to device time [s]";
    msg.values[6].value = std::to_string(_accumulated_occ_metric_gpu[0] / _num_occ_metrics_gpu);
    msg.values[7].key = "Average Copy occlusion variables Host to device time [s]";
    msg.values[7].value = std::to_string(_accumulated_occ_metric_gpu[1] / _num_occ_metrics_gpu);
    msg.values[8].key = "Average occlusion Kernel time to process all rays [s]";
    msg.values[8].value = std::to_string(_accumulated_occ_metric_gpu[2] / _num_occ_metrics_gpu);
    msg.values[9].key = "Average Copy device occlusion results to host time [s]";
    msg.values[9].value = std::to_string(_accumulated_occ_metric_gpu[3] / _num_occ_metrics_gpu);
    msg.values[10].key = "Average occlusion Data transfer time (Host to device and device to host) [s]";
    msg.values[10].value = std::to_string(_accumulated_occ_metric_gpu[4] / _num_occ_metrics_gpu);
    msg.values[11].key = "Average Total distribution time [s]";
    msg.values[11].value = std::to_string((_accumulated_occ_metric_gpu[5]) / _num_occ_metrics_gpu);

    // msg.values[18].key = "Average Copy map Host to device time [Hz]";
    // msg.values[18].value = std::to_string(1.0 / (_accumulated_info_metric_gpu[0] / _num_info_metrics_gpu));
    // msg.values[19].key = "Average Copy information variables Host to device time [Hz]";
    // msg.values[19].value = std::to_string(1.0 / (_accumulated_info_metric_gpu[1] / _num_info_metrics_gpu));
    // msg.values[20].key = "Average Kernel time to process all rays [Hz]";
    // msg.values[20].value = std::to_string(1.0 / (_accumulated_info_metric_gpu[2] / _num_info_metrics_gpu));
    // msg.values[21].key = "Average Copy device results to host time [Hz]";
    // msg.values[21].value = std::to_string(1.0 / (_accumulated_info_metric_gpu[3] / _num_info_metrics_gpu));
    // msg.values[22].key = "Average Data transfer time (Host to device and device to host) [Hz]";
    // msg.values[22].value = std::to_string(1.0 / (_accumulated_info_metric_gpu[4] / _num_info_metrics_gpu));
    // msg.values[23].key = "Average Total distribution time [Hz]";
    // msg.values[23].value = std::to_string(1.0 / ((_accumulated_info_metric_gpu[5]) / _num_info_metrics_gpu));

    _gpu_time_pub.publish(msg);
  }
}

void Server::mapConnectCallback(ros::SingleSubscriberPublisher const& pub, int depth)
{
  // When a new node subscribes we will publish the whole map to that node.

  // TODO(UNKNOWN): Make this async

  std::visit(
      [this, &pub, depth](auto& map) {
        if constexpr (!std::is_same_v<std::decay_t<decltype(map)>, std::monostate>)
        {
          auto start = std::chrono::steady_clock::now();

          ufomap_bundled::UFOMapStamped::Ptr msg(new ufomap_bundled::UFOMapStamped);
          if (ufomap_bundled::ufoToMsg(map, msg->map, _compress, depth))
          {
            msg->header.stamp = ros::Time::now();
            msg->header.frame_id = _frame_id;
            pub.publish(msg);
          }

          double whole_time =
              std::chrono::duration<float, std::chrono::seconds::period>(std::chrono::steady_clock::now() - start)
                  .count();
          if (0 == _num_wholes || whole_time < _min_whole_time)
          {
            _min_whole_time = whole_time;
          }
          if (whole_time > _max_whole_time)
          {
            _max_whole_time = whole_time;
          }
          _accumulated_whole_time += whole_time;
          ++_num_wholes;
        }
      },
      _map);
}

bool Server::getMapCallback(ufomap_bundled::GetMap::Request& request, ufomap_bundled::GetMap::Response& response)
{
  std::visit(
      [this, &request, &response](auto& map) {
        if constexpr (!std::is_same_v<std::decay_t<decltype(map)>, std::monostate>)
        {
          ufo::geometry::BoundingVolume bv = ufomap_bundled::msgToUfo(request.bounding_volume);
          response.success = ufomap_bundled::ufoToMsg(map, response.map, bv, request.compress, request.depth);
        }
        else
        {
          response.success = false;
        }
      },
      _map);
  return true;
}

bool Server::clearVolumeCallback(ufomap_bundled::ClearVolume::Request& request,
                                 ufomap_bundled::ClearVolume::Response& response)
{
  std::visit(
      [this, &request, &response](auto& map) {
        if constexpr (!std::is_same_v<std::decay_t<decltype(map)>, std::monostate>)
        {
          ufo::geometry::BoundingVolume bv = ufomap_bundled::msgToUfo(request.bounding_volume);
          for (auto& b : bv)
          {
            map.setValueVolume(b, map.getClampingThresMin(), request.depth);
          }
          response.success = true;
        }
        else
        {
          response.success = false;
        }
      },
      _map);
  return true;
}

bool Server::setVolumeCallback(ufomap_bundled::ClearVolume::Request& request,
                               ufomap_bundled::ClearVolume::Response& response)
{
  std::visit(
      [this, &request, &response](auto& map) {
        if constexpr (std::is_same_v<std::decay_t<decltype(map)>, ufo::map::OccupancyMap>)
        {
          ufo::geometry::BoundingVolume bv = ufomap_bundled::msgToUfo(request.bounding_volume);
          for (auto& b : bv)
          {
            map.setValueVolume(b, map.getClampingThresMax(), request.depth);
          }
          response.success = true;
        }
        else if constexpr (std::is_same_v<std::decay_t<decltype(map)>, ufo::map::OccupancyMapColor>)
        {
          ufo::geometry::BoundingVolume bv = ufomap_bundled::msgToUfo(request.bounding_volume);
          for (auto& b : bv)
          {
            map.setValueVolume(b, map.getClampingThresMax(), request.depth);
          }
          response.success = true;
        }
        else
        {
          response.success = false;
        }
      },
      _map);
  return true;
}

bool Server::setVolumeHuman()
{
  for (auto const& human : _human_obstacle_list)
  {
    for (auto& bdy : human._body_parts)
    {
      double t = 0.0;
      for (int i = bdy.second.state._times.size() - 1; i >= 0; i--)
      {
        t = bdy.second.state._times[i];
        if (bdy.second.bounding_box.type == mhp_robot::robot_misc::BoundingBoxType::SPHERE)
        {
          if (bdy.second.bounding_box.radius == 0.0)
          {
            continue;
          }

          _pose_hum_bp = bdy.second.state.getPose(t) * bdy.second.bounding_box.T;
          ufo::geometry::Sphere sphere(ufo::map::Point3(_pose_hum_bp(0, 3), _pose_hum_bp(1, 3), _pose_hum_bp(2, 3)),
                                       bdy.second.bounding_box.radius);
          _bv_human.add(sphere);
          if (t == 0.0)
          {
            _colors_human.push_back(ufo::map::Color(0, 0, 0));
            _occupancy_human.push_back(1.0);
          }
          else
          {
            _colors_human.push_back(ufo::map::Color(255, 0, 0));
            _occupancy_human.push_back(1.0);
          }
          ufo::map::Prediction pred;
          if (!pred.setPredictionAtElem(i, true))
            ROS_WARN("Could not set prediction time");
          _prediction_human.push_back(std::make_tuple(t, pred));
        }
        else if (bdy.second.bounding_box.type == mhp_robot::robot_misc::BoundingBoxType::CYLINDER)
        {
          // Transform into start point and let z axis point into cylinder direction
          _pose_hum_bp = bdy.second.state.getPose(t) * bdy.second.bounding_box.T *
                         mhp_robot::robot_misc::Common::roty(M_PI / 2.0);
          for (double distance = _resolution; distance <= bdy.second.bounding_box.length_x;
               distance += _resolution * std::pow(2, _depth_human - 1))  // 2^2 = 4 for depth 3
          {
            // Calculate origin of the cylinder marker
            _tmp_translation(0) = 0;
            _tmp_translation(1) = 0;
            _tmp_translation(2) = distance;
            _tmp_translation = _pose_hum_bp.block<3, 3>(0, 0) * _tmp_translation;
            ufo::geometry::Sphere sphere(ufo::map::Point3(_pose_hum_bp(0, 3) + _tmp_translation(0),
                                                          _pose_hum_bp(1, 3) + _tmp_translation(1),
                                                          _pose_hum_bp(2, 3) + _tmp_translation(2)),
                                         bdy.second.bounding_box.radius);

            _bv_human.add(sphere);

            if (t == 0.0)
            {
              _colors_human.push_back(ufo::map::Color(0, 0, 0));
              _occupancy_human.push_back(1.0);
            }
            else
            {
              _colors_human.push_back(ufo::map::Color(255, 0, 0));
              _occupancy_human.push_back(1.0);
            }
            ufo::map::Prediction pred;
            if (!pred.setPredictionAtElem(i, true))
              ROS_WARN("Could not set prediction time");
            _prediction_human.push_back(std::make_tuple(t, pred));
          }
        }
        else
        {
          ROS_WARN("Server: Human obstacle bounding box is unknown");
        }
      }
    }
  }

  // Test Sphere
  // ufo::geometry::Sphere sphere(ufo::map::Point3(-1.2, 0.0, 0.0), 0.2);
  // _bv_human.add(sphere);
  // _bv_human.add(sphere);

  // _colors_human.push_back(ufo::map::Color(0, 0, 0));
  // _colors_human.push_back(ufo::map::Color(255, 0, 0));
  // _occupancy_human.push_back(0.8);
  // _occupancy_human.push_back(0.8);
  // ufo::map::Prediction pred;
  // pred.setPredictionAtTime(0.0, true);
  // _prediction_human.push_back(std::make_tuple(0.0, pred));
  // pred.setPredictionAtTime(0.5, true);
  // _prediction_human.push_back(std::make_tuple(0.5, pred));

  std::visit(
      [this](auto& map) {
        if constexpr (std::is_same_v<std::decay_t<decltype(map)>, ufo::map::OccupancyMapColor>)
        {
          // ufo::map::PointCloud cloud;
          // cloud.push_back(sphere.center);
          // map.insertPointCloudDiscrete(sphere.center, cloud, 1.0, 0, _simple_ray_casting, _early_stopping,
          // _async);
          int j = 0;
          for (auto& b : _bv_human)
          {
            if (_occupancy_human[j] > 0)
            {
              map.setValueVolume(b, _occupancy_human[j], _depth_human, _colors_human[j]);
            }
            map.setPredictionVolume(b, _prediction_human[j], _depth_human);
            if (const ufo::geometry::AABB* pval = std::get_if<ufo::geometry::AABB>(&b))
            {
              ufo::map::Point3 min = pval->getMin();
              ufo::map::Point3 max = pval->getMax();
              // compare against _min_change_for_obs and take the minimum
              _min_change_for_obs.x() = std::min(_min_change_for_obs.x(), min.x());
              _min_change_for_obs.y() = std::min(_min_change_for_obs.y(), min.y());
              _min_change_for_obs.z() = std::min(_min_change_for_obs.z(), min.z());

              _max_change_for_obs.x() = std::max(_max_change_for_obs.x(), max.x());
              _max_change_for_obs.y() = std::max(_max_change_for_obs.y(), max.y());
              _max_change_for_obs.z() = std::max(_max_change_for_obs.z(), max.z());
            }

            else if (const ufo::geometry::OBB* pval = std::get_if<ufo::geometry::OBB>(&b))
            {
              ufo::map::Point3 min = pval->getMin();
              ufo::map::Point3 max = pval->getMax();
              // compare against _min_change_for_obs and take the minimum
              _min_change_for_obs.x() = std::min(_min_change_for_obs.x(), min.x());
              _min_change_for_obs.y() = std::min(_min_change_for_obs.y(), min.y());
              _min_change_for_obs.z() = std::min(_min_change_for_obs.z(), min.z());

              _max_change_for_obs.x() = std::max(_max_change_for_obs.x(), max.x());
              _max_change_for_obs.y() = std::max(_max_change_for_obs.y(), max.y());
              _max_change_for_obs.z() = std::max(_max_change_for_obs.z(), max.z());
            }
            else if (const ufo::geometry::Sphere* pval = std::get_if<ufo::geometry::Sphere>(&b))
            {
              ufo::map::Point3 center = pval->center;
              double radius = pval->radius;
              // compare against _min_change_for_obs and take the minimum
              _min_change_for_obs.x() = std::min(_min_change_for_obs.x(), center.x() - radius);
              _min_change_for_obs.y() = std::min(_min_change_for_obs.y(), center.y() - radius);
              _min_change_for_obs.z() = std::min(_min_change_for_obs.z(), center.z() - radius);

              _max_change_for_obs.x() = std::max(_max_change_for_obs.x(), center.x() + radius);
              _max_change_for_obs.y() = std::max(_max_change_for_obs.y(), center.y() + radius);
              _max_change_for_obs.z() = std::max(_max_change_for_obs.z(), center.z() + radius);
            }
            j++;
          }
        }
      },
      _map);

  // _bv_human.clear();
  // _colors_human.clear();
  // _occupancy_human.clear();
  // _prediction_human.clear();
  return true;
}

bool Server::setVolumeDynamicObstacle()
{
  for (auto const& obstacle : _dynamic_obstacle_list)
  {
    // ROS_WARN("Server: Setting dynamic obstacles for obstacle %s", obstacle.name.c_str());

    double t = 0.0;
    for (int i = obstacle.state._times.size() - 1; i >= 0; i--)
    {
      // ROS_WARN("Server: Setting dynamic obstacles for obstacle %s at time %f", obstacle.name.c_str(),
      //  obstacle.state._times[i]);
      t = obstacle.state._times[i];
      if (obstacle.bounding_box.type == mhp_robot::robot_misc::BoundingBoxType::SPHERE)
      {
        if (obstacle.bounding_box.radius == 0.0)
        {
          continue;
        }

        _pose_dynamic_obstacle_bp = obstacle.state.getPose(t) * obstacle.bounding_box.T;
        ufo::geometry::Sphere sphere(ufo::map::Point3(_pose_dynamic_obstacle_bp(0, 3), _pose_dynamic_obstacle_bp(1, 3),
                                                      _pose_dynamic_obstacle_bp(2, 3)),
                                     obstacle.bounding_box.radius);
        _bv_dynamic_obstacle.add(sphere);
        if (t == 0.0)
        {
          _colors_dynamic_obstacle.push_back(ufo::map::Color(0, 0, 0));
          _occupancy_dynamic_obstacle.push_back(1.0);
        }
        else
        {
          _colors_dynamic_obstacle.push_back(ufo::map::Color(255, 0, 0));
          _occupancy_dynamic_obstacle.push_back(1.0);
        }
        ufo::map::Prediction pred;
        if (!pred.setPredictionAtElem(i, true))
          ROS_WARN("Could not set prediction time");
        _prediction_dynamic_obstacle.push_back(std::make_tuple(t, pred));
      }
      else if (obstacle.bounding_box.type == mhp_robot::robot_misc::BoundingBoxType::CYLINDER)
      {
        // Transform into start point and let z axis point into cylinder direction
        _pose_dynamic_obstacle_bp =
            obstacle.state.getPose(t) * obstacle.bounding_box.T * mhp_robot::robot_misc::Common::roty(M_PI / 2.0);
        for (double distance = _resolution; distance <= obstacle.bounding_box.length_x;
             distance += _resolution * std::pow(2, _depth_human - 1))  // 2^2 = 4 for depth 3
        {
          // Calculate origin of the cylinder marker
          _tmp_translation(0) = 0;
          _tmp_translation(1) = 0;
          _tmp_translation(2) = distance;
          _tmp_translation = _pose_dynamic_obstacle_bp.block<3, 3>(0, 0) * _tmp_translation;
          ufo::geometry::Sphere sphere(ufo::map::Point3(_pose_dynamic_obstacle_bp(0, 3) + _tmp_translation(0),
                                                        _pose_dynamic_obstacle_bp(1, 3) + _tmp_translation(1),
                                                        _pose_dynamic_obstacle_bp(2, 3) + _tmp_translation(2)),
                                       obstacle.bounding_box.radius);

          _bv_dynamic_obstacle.add(sphere);

          if (t == 0.0)
          {
            _colors_dynamic_obstacle.push_back(ufo::map::Color(0, 0, 0));
            _occupancy_dynamic_obstacle.push_back(1.0);
          }
          else
          {
            _colors_dynamic_obstacle.push_back(ufo::map::Color(255, 0, 0));
            _occupancy_dynamic_obstacle.push_back(1.0);
          }
          ufo::map::Prediction pred;
          if (!pred.setPredictionAtElem(i, true))
            ROS_WARN("Could not set prediction time");
          _prediction_dynamic_obstacle.push_back(std::make_tuple(t, pred));
        }
      }
      else if (obstacle.bounding_box.type == mhp_robot::robot_misc::BoundingBoxType::EBOX)
      {
        ROS_WARN("Server: Dynamic obstacle EBOX not switched to OBB yet");
        _pose_dynamic_obstacle_bp = obstacle.state.getPose(t) * obstacle.bounding_box.T;
        ufo::geometry::AABB aabb;
        aabb.center = ufo::map::Point3(_pose_dynamic_obstacle_bp(0, 3) + obstacle.bounding_box.length_x / 2.0,
                                       _pose_dynamic_obstacle_bp(1, 3) + obstacle.bounding_box.length_y / 2.0,
                                       _pose_dynamic_obstacle_bp(2, 3) / 2.0);
        aabb.half_size = ufo::map::Point3(obstacle.bounding_box.length_x / 2.0, obstacle.bounding_box.length_y / 2.0,
                                          _pose_dynamic_obstacle_bp(2, 3) / 2.0);
        _bv_dynamic_obstacle.add(aabb);
        if (t == 0.0)
        {
          _colors_dynamic_obstacle.push_back(ufo::map::Color(0, 0, 0));
          _occupancy_dynamic_obstacle.push_back(1.0);
        }
        else
        {
          _colors_dynamic_obstacle.push_back(ufo::map::Color(255, 0, 0));
          _occupancy_dynamic_obstacle.push_back(1.0);
        }
        ufo::map::Prediction pred;
        if (!pred.setPredictionAtElem(i, true))
          ROS_WARN("Could not set prediction time");
        _prediction_dynamic_obstacle.push_back(std::make_tuple(t, pred));
      }
      else if (obstacle.bounding_box.type == mhp_robot::robot_misc::BoundingBoxType::BOX)
      {
        _pose_dynamic_obstacle_bp = obstacle.state.getPose(t) * obstacle.bounding_box.T;

        // consider orientation in _pose_dynamic_obstacle_bp
        Eigen::Vector3d rotated_half_size =
            _pose_dynamic_obstacle_bp.block<3, 3>(0, 0) *
            Eigen::Vector3d(obstacle.bounding_box.length_x / 2.0, obstacle.bounding_box.length_y / 2.0, 0.0);

        ufo::geometry::OBB obb;
        obb.center = ufo::map::Point3(_pose_dynamic_obstacle_bp(0, 3) + rotated_half_size(0),
                                      _pose_dynamic_obstacle_bp(1, 3) + rotated_half_size(1),
                                      _pose_dynamic_obstacle_bp(2, 3) + rotated_half_size(2));
        obb.half_size = ufo::map::Point3(obstacle.bounding_box.length_x / 2.0 + obstacle.bounding_box.radius,
                                         obstacle.bounding_box.length_y / 2.0 + obstacle.bounding_box.radius,
                                         obstacle.bounding_box.radius);

        Eigen::Quaterniond q(_pose_dynamic_obstacle_bp.block<3, 3>(0, 0));
        obb.rotation = ufo::math::Quaternion(q.w(), -q.x(), -q.y(), -q.z());

        // aabb.center = ufo::map::Point3(_pose_dynamic_obstacle_bp(0, 3) + rotated_half_size2(0),
        //                                _pose_dynamic_obstacle_bp(1, 3) + rotated_half_size2(1),
        //                                _pose_dynamic_obstacle_bp(2, 3) + rotated_half_size2(2));

        // aabb.half_size = ufo::map::Point3(std::abs(rotated_half_size(0)), std::abs(rotated_half_size(1)),
        //                                   std::abs(rotated_half_size(2)));

        // std::cout << "AABB center: " << aabb.center.x() << " " << aabb.center.y() << " " << aabb.center.z()
        //           << std::endl;
        // std::cout << "AABB half_size: " << aabb.half_size.x() << " " << aabb.half_size.y() << " " <<
        // aabb.half_size.z()
        //           << std::endl;

        _bv_dynamic_obstacle.add(obb);
        if (t == 0.0)
        {
          _colors_dynamic_obstacle.push_back(ufo::map::Color(0, 0, 0));
          _occupancy_dynamic_obstacle.push_back(0.7);
        }
        else
        {
          _colors_dynamic_obstacle.push_back(ufo::map::Color(255, 0, 0));
          _occupancy_dynamic_obstacle.push_back(0.7);
        }
        ufo::map::Prediction pred;
        if (!pred.setPredictionAtElem(i, true))
          ROS_WARN("Could not set prediction time");
        _prediction_dynamic_obstacle.push_back(std::make_tuple(t, pred));
      }
      else
      {
        ROS_WARN("Server: Dynamic obstacle bounding box is unknown");
      }
    }
  }

  std::visit(
      [this](auto& map) {
        if constexpr (std::is_same_v<std::decay_t<decltype(map)>, ufo::map::OccupancyMapColor>)
        {
          int j = 0;
          for (auto& b : _bv_dynamic_obstacle)
          {
            if (_occupancy_dynamic_obstacle[j] > 0)
            {
              map.setValueVolume(b, _occupancy_dynamic_obstacle[j], _depth_dynamic_obstacle,
                                 _colors_dynamic_obstacle[j]);
            }
            map.setPredictionVolume(b, _prediction_dynamic_obstacle[j], _depth_dynamic_obstacle);

            // Get type of bounding box
            if (const ufo::geometry::AABB* pval = std::get_if<ufo::geometry::AABB>(&b))
            {
              ufo::map::Point3 min = pval->getMin();
              ufo::map::Point3 max = pval->getMax();
              // compare against _min_change_for_obs and take the minimum
              _min_change_for_obs.x() = std::min(_min_change_for_obs.x(), min.x());
              _min_change_for_obs.y() = std::min(_min_change_for_obs.y(), min.y());
              _min_change_for_obs.z() = std::min(_min_change_for_obs.z(), min.z());

              _max_change_for_obs.x() = std::max(_max_change_for_obs.x(), max.x());
              _max_change_for_obs.y() = std::max(_max_change_for_obs.y(), max.y());
              _max_change_for_obs.z() = std::max(_max_change_for_obs.z(), max.z());
            }

            else if (const ufo::geometry::OBB* pval = std::get_if<ufo::geometry::OBB>(&b))
            {
              ufo::map::Point3 min = pval->getMin();
              ufo::map::Point3 max = pval->getMax();
              // compare against _min_change_for_obs and take the minimum
              _min_change_for_obs.x() = std::min(_min_change_for_obs.x(), min.x());
              _min_change_for_obs.y() = std::min(_min_change_for_obs.y(), min.y());
              _min_change_for_obs.z() = std::min(_min_change_for_obs.z(), min.z());

              _max_change_for_obs.x() = std::max(_max_change_for_obs.x(), max.x());
              _max_change_for_obs.y() = std::max(_max_change_for_obs.y(), max.y());
              _max_change_for_obs.z() = std::max(_max_change_for_obs.z(), max.z());
            }
            else if (const ufo::geometry::Sphere* pval = std::get_if<ufo::geometry::Sphere>(&b))
            {
              ufo::map::Point3 center = pval->center;
              double radius = pval->radius;
              // compare against _min_change_for_obs and take the minimum
              _min_change_for_obs.x() = std::min(_min_change_for_obs.x(), center.x() - radius);
              _min_change_for_obs.y() = std::min(_min_change_for_obs.y(), center.y() - radius);
              _min_change_for_obs.z() = std::min(_min_change_for_obs.z(), center.z() - radius);

              _max_change_for_obs.x() = std::max(_max_change_for_obs.x(), center.x() + radius);
              _max_change_for_obs.y() = std::max(_max_change_for_obs.y(), center.y() + radius);
              _max_change_for_obs.z() = std::max(_max_change_for_obs.z(), center.z() + radius);
            }
            j++;
          }
        }
      },
      _map);

  return true;
}  // namespace ufomap_mapping

bool Server::setVolumeStaticObstacle()
{
  ROS_WARN("Server: Setting static obstacles");
  for (auto const& obstacle : _static_obstacle_list)
  {
    ROS_WARN("Server: Setting static obstacles for obstacle %s", obstacle.name.c_str());

    double t = 0.0;
    Eigen::VectorXd times = Eigen::VectorXd::LinSpaced(30, 0, 2.9);
    for (int i = times.size() - 1; i >= 0; i--)
    {
      ROS_WARN("Server: Setting static obstacles for obstacle %s at time %f", obstacle.name.c_str(), times[i]);
      t = times[i];
      if (obstacle.bounding_box.type == mhp_robot::robot_misc::BoundingBoxType::SPHERE)
      {
        if (obstacle.bounding_box.radius == 0.0)
        {
          continue;
        }

        _pose_static_obstacle_bp = obstacle.state.getPose(t) * obstacle.bounding_box.T;
        ufo::geometry::Sphere sphere(ufo::map::Point3(_pose_static_obstacle_bp(0, 3), _pose_static_obstacle_bp(1, 3),
                                                      _pose_static_obstacle_bp(2, 3)),
                                     obstacle.bounding_box.radius);
        _bv_static_obstacle.add(sphere);
        if (t == 0.0)
        {
          _colors_static_obstacle.push_back(ufo::map::Color(0, 0, 0));
          _occupancy_static_obstacle.push_back(1.0);
        }
        else
        {
          _colors_static_obstacle.push_back(ufo::map::Color(255, 0, 0));
          _occupancy_static_obstacle.push_back(1.0);
        }
        ufo::map::Prediction pred;
        if (!pred.setPredictionAtElem(i, true))
          ROS_WARN("Could not set prediction time");
        _prediction_static_obstacle.push_back(std::make_tuple(t, pred));
      }
      else if (obstacle.bounding_box.type == mhp_robot::robot_misc::BoundingBoxType::CYLINDER)
      {
        // Transform into start point and let z axis point into cylinder direction
        _pose_static_obstacle_bp =
            obstacle.state.getPose(t) * obstacle.bounding_box.T * mhp_robot::robot_misc::Common::roty(M_PI / 2.0);
        for (double distance = _resolution; distance <= obstacle.bounding_box.length_x;
             distance += _resolution * std::pow(2, _depth_human - 1))  // 2^2 = 4 for depth 3
        {
          // Calculate origin of the cylinder marker
          _tmp_translation(0) = 0;
          _tmp_translation(1) = 0;
          _tmp_translation(2) = distance;
          _tmp_translation = _pose_static_obstacle_bp.block<3, 3>(0, 0) * _tmp_translation;
          ufo::geometry::Sphere sphere(ufo::map::Point3(_pose_static_obstacle_bp(0, 3) + _tmp_translation(0),
                                                        _pose_static_obstacle_bp(1, 3) + _tmp_translation(1),
                                                        _pose_static_obstacle_bp(2, 3) + _tmp_translation(2)),
                                       obstacle.bounding_box.radius);

          _bv_static_obstacle.add(sphere);

          if (t == 0.0)
          {
            _colors_static_obstacle.push_back(ufo::map::Color(0, 0, 0));
            _occupancy_static_obstacle.push_back(1.0);
          }
          else
          {
            _colors_static_obstacle.push_back(ufo::map::Color(255, 0, 0));
            _occupancy_static_obstacle.push_back(1.0);
          }
          ufo::map::Prediction pred;
          if (!pred.setPredictionAtElem(i, true))
            ROS_WARN("Could not set prediction time");
          _prediction_static_obstacle.push_back(std::make_tuple(t, pred));
        }
      }
      else if (obstacle.bounding_box.type == mhp_robot::robot_misc::BoundingBoxType::EBOX)
      {
        ROS_WARN("Server: Static obstacle bounding box is EBOX; Not added since not implemented yet");
      }
      else if (obstacle.bounding_box.type == mhp_robot::robot_misc::BoundingBoxType::BOX)
      {
        _pose_static_obstacle_bp = obstacle.state.getPose(t) * obstacle.bounding_box.T;

        // consider orientation in _pose_dynamic_obstacle_bp
        Eigen::Vector3d rotated_half_size =
            _pose_static_obstacle_bp.block<3, 3>(0, 0) *
            Eigen::Vector3d(obstacle.bounding_box.length_x / 2.0, obstacle.bounding_box.length_y / 2.0, 0.0);

        ufo::geometry::OBB obb;
        obb.center = ufo::map::Point3(_pose_static_obstacle_bp(0, 3) + rotated_half_size(0),
                                      _pose_static_obstacle_bp(1, 3) + rotated_half_size(1),
                                      _pose_static_obstacle_bp(2, 3) + rotated_half_size(2));
        obb.half_size = ufo::map::Point3(obstacle.bounding_box.length_x / 2.0 + obstacle.bounding_box.radius,
                                         obstacle.bounding_box.length_y / 2.0 + obstacle.bounding_box.radius,
                                         obstacle.bounding_box.radius);

        Eigen::Quaterniond q(_pose_static_obstacle_bp.block<3, 3>(0, 0));
        obb.rotation = ufo::math::Quaternion(q.w(), -q.x(), -q.y(), -q.z());

        _bv_static_obstacle.add(obb);

        // ufo::geometry::AABB aabb;
        // aabb.center = ufo::map::Point3(_pose_static_obstacle_bp(0, 3) + obstacle.bounding_box.length_x / 2.0,
        //                                _pose_static_obstacle_bp(1, 3) + obstacle.bounding_box.length_y / 2.0,
        //                                _pose_static_obstacle_bp(2, 3));
        // aabb.half_size = ufo::map::Point3(obstacle.bounding_box.length_x / 2.0 + obstacle.bounding_box.radius,
        //                                   obstacle.bounding_box.length_y / 2.0 + obstacle.bounding_box.radius,
        //                                   obstacle.bounding_box.radius);
        // _bv_static_obstacle.add(aabb);
        if (t == 0.0)
        {
          _colors_static_obstacle.push_back(ufo::map::Color(0, 0, 0));
          _occupancy_static_obstacle.push_back(1.0);
        }
        else
        {
          _colors_static_obstacle.push_back(ufo::map::Color(255, 0, 0));
          _occupancy_static_obstacle.push_back(1.0);
        }
        ufo::map::Prediction pred;
        if (!pred.setPredictionAtElem(i, true))
          ROS_WARN("Could not set prediction time");
        _prediction_static_obstacle.push_back(std::make_tuple(t, pred));
      }
      else
      {
        ROS_WARN("Server: Static obstacle bounding box is unknown");
      }
    }
  }

  std::visit(
      [this](auto& map) {
        if constexpr (std::is_same_v<std::decay_t<decltype(map)>, ufo::map::OccupancyMapColor>)
        {
          int j = 0;
          for (auto& b : _bv_static_obstacle)
          {
            if (_occupancy_static_obstacle[j] > 0)
            {
              map.setValueVolume(b, _occupancy_static_obstacle[j], _depth_static_obstacle, _colors_static_obstacle[j]);
            }
            map.setPredictionVolume(b, _prediction_static_obstacle[j], _depth_static_obstacle);
            j++;
          }
        }
      },
      _map);
  _static_obstacles_added = true;
  return true;
}

bool Server::resetVolumeHuman()
{
  std::visit(
      [this](auto& map) {
        if constexpr (std::is_same_v<std::decay_t<decltype(map)>, ufo::map::OccupancyMapColor>)
        {
          // ufo::map::PointCloud cloud;
          // cloud.push_back(sphere.center);
          // map.insertPointCloudDiscrete(sphere.center, cloud, 1.0, 0, _simple_ray_casting, _early_stopping,
          // _async);
          int j = 0;
          for (auto& b : _bv_human)
          {
            if (_occupancy_human[j] > 0)
            {
              _occupancy_human[j] = 0.0;
              map.setValueVolume(b, _occupancy_human[j], _depth_human, _colors_human[j]);
            }
            std::get<1>(_prediction_human[j]).reset();
            map.setPredictionVolume(b, _prediction_human[j], _depth_human);
            j++;
          }
        }
      },
      _map);

  _bv_human.clear();
  _colors_human.clear();
  _occupancy_human.clear();
  _prediction_human.clear();
  return true;
}

bool Server::resetVolumeDynamicObstacle()
{
  std::visit(
      [this](auto& map) {
        if constexpr (std::is_same_v<std::decay_t<decltype(map)>, ufo::map::OccupancyMapColor>)
        {
          int j = 0;
          for (auto& b : _bv_dynamic_obstacle)
          {
            if (_occupancy_dynamic_obstacle[j] > 0)
            {
              _occupancy_dynamic_obstacle[j] = 0.0;
              map.setValueVolume(b, _occupancy_dynamic_obstacle[j], _depth_dynamic_obstacle,
                                 _colors_dynamic_obstacle[j]);
            }
            std::get<1>(_prediction_dynamic_obstacle[j]).reset();
            map.setPredictionVolume(b, _prediction_dynamic_obstacle[j], _depth_dynamic_obstacle);
            j++;
          }
        }
      },
      _map);

  _bv_dynamic_obstacle.clear();
  _colors_dynamic_obstacle.clear();
  _occupancy_dynamic_obstacle.clear();
  _prediction_dynamic_obstacle.clear();
  return true;
}

bool Server::resetCallback(ufomap_bundled::Reset::Request& request, ufomap_bundled::Reset::Response& response)
{
  std::visit(
      [this, &request, &response](auto& map) {
        if constexpr (!std::is_same_v<std::decay_t<decltype(map)>, std::monostate>)
        {
          map.clear(request.new_resolution, request.new_depth_levels);
          response.success = true;
        }
        else
        {
          response.success = false;
        }
      },
      _map);
  return true;
}

bool Server::saveMapCallback(ufomap_bundled::SaveMap::Request& request, ufomap_bundled::SaveMap::Response& response)
{
  std::visit(
      [this, &request, &response](auto& map) {
        if constexpr (!std::is_same_v<std::decay_t<decltype(map)>, std::monostate>)
        {
          ufo::geometry::BoundingVolume bv = ufomap_bundled::msgToUfo(request.bounding_volume);
          response.success =
              map.write(request.filename, bv, request.compress, request.depth, 1, request.compression_level);
        }
        else
        {
          response.success = false;
        }
      },
      _map);
  return true;
}

void Server::timerCallback(ros::TimerEvent const& event)
{
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = _frame_id;

  if (!_map_pub.empty())
  {
    for (int i = 0; i < _map_pub.size(); ++i)
    {
      if (_map_pub[i] && (0 < _map_pub[i].getNumSubscribers() || _map_pub[i].isLatched()))
      {
        std::visit(
            [this, &header, i](auto& map) {
              if constexpr (!std::is_same_v<std::decay_t<decltype(map)>, std::monostate>)
              {
                auto start = std::chrono::steady_clock::now();

                ufomap_bundled::UFOMapStamped::Ptr msg(new ufomap_bundled::UFOMapStamped);
                if (ufomap_bundled::ufoToMsg(map, msg->map, _compress, i))
                {
                  msg->header = header;
                  _map_pub[i].publish(msg);
                }

                double whole_time =
                    std::chrono::duration<float, std::chrono::seconds::period>(std::chrono::steady_clock::now() - start)
                        .count();
                if (0 == _num_wholes || whole_time < _min_whole_time)
                {
                  _min_whole_time = whole_time;
                }
                if (whole_time > _max_whole_time)
                {
                  _max_whole_time = whole_time;
                }
                _accumulated_whole_time += whole_time;
                ++_num_wholes;
              }
            },
            _map);
      }
    }
  }
  publishInfo();
}

void Server::configCallback(ufomap_bundled::ServerConfig& config, uint32_t level)
{
  // Read parameters
  _frame_id = config.frame_id;

  _verbose = config.verbose;

  _max_range = config.max_range;
  _insert_depth = config.insert_depth;
  _simple_ray_casting = config.simple_ray_casting;
  _early_stopping = config.early_stopping;
  _async = config.async;

  _clear_robot = config.clear_robot;
  _robot_frame_id = config.robot_frame_id;
  _robot_height = config.robot_height;
  _robot_radius = config.robot_radius;
  _clearing_depth = config.clearing_depth;

  _compress = config.compress;
  _update_part_of_map = config.update_part_of_map;
  _publish_depth = config.publish_depth;

  // _scaling_factor_gridpoints = config.scaling_factor_gridpoints;

  std::visit(
      [this, &config](auto& map) {
        if constexpr (!std::is_same_v<std::decay_t<decltype(map)>, std::monostate>)
        {
          map.setProbHit(config.prob_hit);
          map.setProbMiss(config.prob_miss);
          map.setClampingThresMin(config.clamping_thres_min);
          map.setClampingThresMax(config.clamping_thres_max);
        }
      },
      _map);

  _transform_timeout.fromSec(config.transform_timeout);

  // Set up publisher
  if (_map_pub.empty() || _map_pub[0].isLatched() != config.map_latch || _map_queue_size != config.map_queue_size)
  {
    _map_pub.resize(_publish_depth + 1);
    for (int i = 0; i < _map_pub.size(); ++i)
    {
      _map_queue_size = config.map_queue_size;
      std::string final_topic = i == 0 ? "map" : "map_depth_" + std::to_string(i);
      _map_pub[i] = _nh_priv.advertise<ufomap_bundled::UFOMapStamped>(
          final_topic, _map_queue_size, boost::bind(&Server::mapConnectCallback, this, _1, i),
          ros::SubscriberStatusCallback(), ros::VoidConstPtr(), config.map_latch);
    }
  }

  // Set up subscriber
  if (!_cloud_sub || _cloud_in_queue_size != config.cloud_in_queue_size)
  {
    _cloud_in_queue_size = config.cloud_in_queue_size;
    _cloud_sub = _nh.subscribe("cloud_in", _cloud_in_queue_size, &Server::cloudCallback, this);
  }
  _save_map_sub = _nh.subscribe("/save_map_topic", 10, &Server::saveMapTopicCallback, this);

  // Set up timer
  if (!_pub_timer || _pub_rate != config.pub_rate)
  {
    _pub_rate = config.pub_rate;
    if (0 < _pub_rate)
    {
      _pub_timer = _nh_priv.createTimer(ros::Rate(_pub_rate), &Server::timerCallback, this);
    }
    else
    {
      _pub_timer.stop();
    }
  }

  // Set up update rate
  if (config.update_rate != 0)
  {
    _update_rate = ros::Duration(1.0 / config.update_rate);
  }
  else
  {
    _update_rate = ros::Duration(0.0);
  }
}

void Server::calculateTargetFromPoint(const std_msgs::Header& msg_header)
{
  if (!_human_obstacle_list.empty())
  {
    // Get target from first human
    mhp_robot::robot_misc::Human human = _human_obstacle_list[0];
    mhp_robot::robot_misc::Obstacle rHand = human._body_parts["RHand"];
    mhp_robot::robot_misc::Obstacle lHand = human._body_parts["LHand"];

    // Get pose from each hand at each time step
    Eigen::Matrix4d rhandPose = rHand.state.getPose(0.0);
    Eigen::Matrix4d lhandPose = lHand.state.getPose(0.0);

    Eigen::Vector3d center_of_hands = (rhandPose.block<3, 1>(0, 3) + lhandPose.block<3, 1>(0, 3)) / 2.0;
    // std::cout << "Center of hands: " << center_of_hands.transpose() << std::endl;
    geometry_msgs::PoseStamped hand_com_msg;
    hand_com_msg.header.frame_id = _frame_id;
    hand_com_msg.header.stamp = msg_header.stamp;
    hand_com_msg.pose.position.x = center_of_hands[0];
    hand_com_msg.pose.position.y = center_of_hands[1];
    hand_com_msg.pose.position.z = center_of_hands[2];
    hand_com_msg.pose.orientation.x = 0.0;
    hand_com_msg.pose.orientation.y = 0.0;
    hand_com_msg.pose.orientation.z = 0.0;
    hand_com_msg.pose.orientation.w = 1.0;
    _hand_com_pub.publish(hand_com_msg);

    // from the center of hands calculate the ray endpoints (same as in the video target)
    // Sample ray endpoints in spherical coordinates around the target
    double phi = 0.0;
    double theta = 0.0;
    std::vector<ufo::map::Point3> ray_endpoints;
    std::vector<Eigen::Vector3d> sphere_coords;
    int arrowPlotCounter = 0;
    while (phi < _max_phi)
    {
      theta = 0.0;
      while (theta < _max_theta)
      {
        ufo::map::Point3 ray_endpoint(center_of_hands.x() + _radius * sin(theta) * cos(phi),
                                      center_of_hands.y() + _radius * sin(theta) * sin(phi),
                                      center_of_hands.z() + _radius * cos(theta));
        // plotArrow(target, ray_endpoint, arrowPlotCounter, true, false);
        // arrowPlotCounter++;
        // check if ray endpoint is NaN
        if (isnan(ray_endpoint.x()) || isnan(ray_endpoint.y()) || isnan(ray_endpoint.z()))
        {
          // std::cout << "Ray endpoint is NaN" << std::endl;
          theta += _step_size_theta;
          continue;
        }

        ray_endpoints.push_back(ray_endpoint);
        sphere_coords.push_back(Eigen::Vector3d(_radius, theta, phi));
        theta += _step_size_theta;
      }
      phi += _step_size_phi;
    }
    ROS_INFO_ONCE("Number of rays for occlusion distribution: %d", static_cast<int>(ray_endpoints.size()));
    if (ray_endpoints.size() == 0)
    {
      ROS_WARN_ONCE(
          "UFOMap: No valid ray endpoints found for occlusion calculation. Skip this target; Message only shown "
          "once.");
      return;
    }

    // Set the ray endpoints and startpoint for occlusion calculation
    _occ_mutex.lock();
    _ray_endpoints_occlusion.clear();

    _start_points_occlusion.clear();
    _start_points_occlusion.push_back(
        std::make_tuple(0, ufo::map::Point3(center_of_hands[0], center_of_hands[1], center_of_hands[2]),
                        Eigen::AngleAxisd(1.0, Eigen::Vector3d(0.0, 0.0, 1.0))));
    _ray_endpoints_occlusion[0] = ray_endpoints;
    _ray_endpoints_occlusion_sphere_coords[0] = sphere_coords;
    _occlusion_info_matrix.resize(_pred_steps_occ, _ray_endpoints_occlusion[0].size());
    _occ_mutex.unlock();
  }
  else
  {
    ROS_WARN_ONCE("UFOMap: No human obstacle list found. Skip this target; Message only shown once.");
  }
}

void Server::targetCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  _hand_com_pub.publish(msg);

  ROS_INFO_THROTTLE(2.0, "Server: Received target pose. (Throttled msg)");
  // print target position
  auto start_time = std::chrono::steady_clock::now();

  // Start position is target and needs to be transferred into _start_points_occlusion
  ufo::map::Point3 target(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

  // Sample ray endpoints in spherical coordinates around the target
  double phi = 0.0;
  double theta = 0.0;
  std::vector<ufo::map::Point3> ray_endpoints;
  std::vector<Eigen::Vector3d> sphere_coords;
  int arrowPlotCounter = 0;
  while (phi < _max_phi)
  {
    theta = 0.0;
    while (theta < _max_theta)
    {
      ufo::map::Point3 ray_endpoint(target.x() + _radius * sin(theta) * cos(phi),
                                    target.y() + _radius * sin(theta) * sin(phi), target.z() + _radius * cos(theta));
      // plotArrow(target, ray_endpoint, arrowPlotCounter, true, false);
      // arrowPlotCounter++;
      // check if ray endpoint is NaN
      if (isnan(ray_endpoint.x()) || isnan(ray_endpoint.y()) || isnan(ray_endpoint.z()))
      {
        // std::cout << "Ray endpoint is NaN" << std::endl;
        theta += _step_size_theta;
        continue;
      }

      ray_endpoints.push_back(ray_endpoint);
      sphere_coords.push_back(Eigen::Vector3d(_radius, theta, phi));
      theta += _step_size_theta;
    }
    phi += _step_size_phi;
  }
  ROS_INFO_ONCE("Number of rays for occlusion distribution: %d", static_cast<int>(ray_endpoints.size()));
  if (ray_endpoints.size() == 0)
  {
    ROS_WARN_ONCE(
        "UFOMap: No valid ray endpoints found for occlusion calculation. Skip this target; Message only shown once.");
    return;
  }

  _occ_mutex.lock();
  _ray_endpoints_occlusion.clear();

  _start_points_occlusion.clear();
  _start_points_occlusion.push_back(
      std::make_tuple(0, ufo::map::Point3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z),
                      Eigen::AngleAxisd(Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x,
                                                           msg->pose.orientation.y, msg->pose.orientation.z))));
  _ray_endpoints_occlusion[0] = ray_endpoints;
  _ray_endpoints_occlusion_sphere_coords[0] = sphere_coords;
  _occlusion_info_matrix.resize(_pred_steps_occ, _ray_endpoints_occlusion[0].size());
  _occ_mutex.unlock();

  if (_target_switch_mode)
  {
    _video_target = true;
    _skeleton_target = false;
    _video_target_start_time = ros::Time::now();
  }
}

void Server::obstacleCallback(const mhp_robot::MsgObstacleListConstPtr& msg)
{
  ROS_INFO_ONCE("Server: Received virtual dynamic obstacle pose.");

  _obstacle_manager._mutex.lock();
  mhp_robot::robot_misc::Common::parseObstacleMsg(msg, _obstacle_manager._static_obstacles,
                                                        _obstacle_manager._dynamic_obstacles, _obstacle_manager._humans,
                                                        _obstacle_manager._planes);
  _obstacle_manager._mutex.unlock();
}

void Server::plotArrow(const ufo::map::Point3& start, const ufo::map::Point3& end, const int type, const int id,
                       const bool mark_start, const Eigen::Vector3d color) const
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = _frame_id;
  marker.header.stamp = ros::Time::now();
  switch (type)
  {
    case 0:
      marker.ns = "info_arrows";
      break;
    case 1:
      marker.ns = "occ_arrows";
      break;
    default:
      ROS_WARN("UFOMapServer: Unknown arrow type");
      break;
  }
  marker.id = id;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;

  geometry_msgs::Point start_p, end_p;
  start_p.x = start.x();
  start_p.y = start.y();
  start_p.z = start.z();
  end_p.x = end.x();
  end_p.y = end.y();
  end_p.z = end.z();
  marker.scale.x = marker.scale.x = marker.scale.y = 0.01;
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = 0.5;
  marker.points.push_back(start_p);
  marker.points.push_back(end_p);

  switch (type)
  {
    case 0:  // information arrows
      _info_arrow_pub.publish(marker);
      break;
    case 1:  // occlusion arrows
      _occ_arrow_pub.publish(marker);
      break;
    default:
      ROS_WARN("UFOMapServer: Unknown arrow type");
      break;
  }
}

bool Server::calculateRayEndpoints(const double far_dist)
{
  int plot_one = 0;
  _ray_endpoints.clear();
  for (auto start_it = _start_points.begin(); start_it != _start_points.end(); start_it++)
  {
    // Define the start point for the information distribution
    std::tuple<int, ufo::map::Point3, Eigen::AngleAxisd>& start = *start_it;
    // Define the ray endpoints for the information distribution in the current
    // perspective (all endpoints on the far plane of the depth camera
    ufo::map::Point3 end(std::get<1>(start).x() + std::get<2>(start).axis()[0] * far_dist,
                         std::get<1>(start).y() + std::get<2>(start).axis()[1] * far_dist,
                         std::get<1>(start).z() + std::get<2>(start).axis()[2] * far_dist);

    // Calculate the frustrum points (fixed since we assume a fixed camera perspective along the z-axis)
    double v_foi = 65 * M_PI / 180;  // From Azure Kinect Specifications
    double h_foi = 75 * M_PI / 180;  // From Azure Kinect Specifications
    // double v_foi  = 10 * M_PI / 180;
    // double h_foi  = 10 * M_PI / 180;
    double height = tan(v_foi / 2) * far_dist;
    double width = tan(h_foi / 2) * far_dist;

    _frustrum_points.clear();
    _frustrum_points.insert({ "end", end });
    _frustrum_points.insert({ "start", std::get<1>(start) });
    _frustrum_points.insert({ "up_left", ufo::map::Point3(end(0) + width, end(1) - height, end(2)) });
    _frustrum_points.insert({ "up_right", ufo::map::Point3(end(0) + width, end(1) + height, end(2)) });
    _frustrum_points.insert({ "down_left", ufo::map::Point3(end(0) - width, end(1) - height, end(2)) });
    _frustrum_points.insert({ "down_right", ufo::map::Point3(end(0) - width, end(1) + height, end(2)) });

    if (_arrow_cnt > 1000000)
    {
      _arrow_cnt = 0;
    }

    if (_plot_arrows && plot_one == 10)
    {
      plotArrow(std::get<1>(start), end, 0, _arrow_cnt, true, Eigen::Vector3d(1.0, 0.0, 0.0));
      _arrow_cnt++;

      plotArrow(std::get<1>(start), _frustrum_points.at("up_left"), 0, _arrow_cnt, true,
                Eigen::Vector3d(0.0, 0.0, 1.0));
      _arrow_cnt++;

      plotArrow(std::get<1>(start), _frustrum_points.at("up_right"), 0, _arrow_cnt, true,
                Eigen::Vector3d(0.0, 0.0, 1.0));
      _arrow_cnt++;

      plotArrow(std::get<1>(start), _frustrum_points.at("down_left"), 0, _arrow_cnt, true,
                Eigen::Vector3d(0.0, 0.0, 1.0));
      _arrow_cnt++;

      plotArrow(std::get<1>(start), _frustrum_points.at("down_right"), 0, _arrow_cnt, true,
                Eigen::Vector3d(0.0, 0.0, 1.0));
      _arrow_cnt++;
    }
    // Variant 1: Only the center of the far plane
    // Variant 2: The four corners of the far plane + the center of the far plane
    // Variant 3: Full grid of endpoints on the far plane (using thk octomap
    // resolution of the smallest possible node size with a scaling factor of 100)
    std::vector<ufo::map::Point3> ray_endpoints;
    switch (_gridpoint_mode)
    {
      case CENTER:
        ray_endpoints.resize(1);
        ray_endpoints.at(0) = end;
        break;
      case FRUSTRUM:
        ray_endpoints.resize(5);
        ray_endpoints.at(0) = end;
        ray_endpoints.at(1) = _frustrum_points.at("up_left");
        ray_endpoints.at(2) = _frustrum_points.at("up_right");
        ray_endpoints.at(3) = _frustrum_points.at("down_left");
        ray_endpoints.at(4) = _frustrum_points.at("down_right");
        break;
      case FULL:
        // Calculate the number of gridpoints in the x and y direction
        double scale_res = _resolution * _scaling_factor_gridpoints;
        int num_x = (int)(2 * width / scale_res);
        int num_y = (int)(2 * height / scale_res);

        // Resize the vector of ray endpoints
        ray_endpoints.clear();
        ray_endpoints.resize((num_x + 1) * (num_y + 1));

        // Iterate over the gridpoints
        for (int i = 0; i <= num_x; i++)
        {
          for (int j = 0; j <= num_y; j++)
          {
            // if ((i == 0 && j == 0) || (i == num_x && j == num_y))
            //   continue;
            // Calculate the current ray endpoint
            ufo::map::Point3 stepUp;
            ufo::map::Point3 stepRight;
            stepUp = (_frustrum_points.at("up_left") - _frustrum_points.at("down_left")) / num_y;
            stepRight = (_frustrum_points.at("down_right") - _frustrum_points.at("down_left")) / num_x;
            ufo::map::Point3 ray_endpoint;
            ray_endpoint = _frustrum_points.at("down_left") + stepUp * j + stepRight * i;

            ray_endpoints.at((i * (num_y + 1) + j)) = ray_endpoint;

            // Plot the current ray endpoint
            if (_plot_arrows)
            {
              if (_arrow_cnt > 1000000)
              {
                _arrow_cnt = 0;
              }
              // std::cout<< "x = " << x << std::endl;
              // std::cout<< "y = " << y << std::endl;
              plotArrow(std::get<1>(start), ray_endpoints.at(i * num_y + j), 0, _arrow_cnt, true,
                        Eigen::Vector3d(1.0, 0.0, 0.0));
              _arrow_cnt++;
            }
          }
        }
        break;
    }
    // ROS_WARN_STREAM("Number endpoints = " << ray_endpoints.size());
    _ray_endpoints.insert({ std::get<0>(start), ray_endpoints });
    plot_one++;
    // ROS_WARN_STREAM("Number endpoints = " << ray_endpoints_.size());
  }
  return true;
}

bool Server::calculateInformationForPerspective(auto& map, int start_point_id)
{
  _accumulated_info_val_normalized = 0.0;
  int end_point_id = 0;
  for (auto endpoint_it = _ray_endpoints[start_point_id].begin(); endpoint_it != _ray_endpoints[start_point_id].end();
       endpoint_it++)
  {
    // ROS_WARN_STREAM("endpoint_it = " << std::get<1>(start_points_.at(start_point_id)).x() << " " <<
    // std::get<1>(start_points_.at(start_point_id)).y() << " " << std::get<1>(start_points_.at(start_point_id)).z());
    // ROS_WARN_STREAM("endpoint_it = " << endpoint_it->x() << " " << endpoint_it->y() << " " << endpoint_it->z());
    double info_val = 0;
    // Get the keys of all the nodes on the ray
    // ROS_WARN("Start computeRay");
    ufo::map::CodeRay allNodesOnCast = map.computeRay(std::get<1>(_start_points.at(start_point_id)), *endpoint_it);

    // Iterate over the ufomap nodes on the ray
    int cnt = 0;
    bool occupied = false;
    // std::cout << "Start point id = " << start_point_id << " has " << allNodesOnCast.size() << " nodes on cast"
    //           << std::endl;
    for (auto it = allNodesOnCast.begin(); it != allNodesOnCast.end(); it++)
    {
      // Search for the node in the octree

      // std::cout << "cnt = " << cnt << std::endl;
      if (!occupied)
      {
        cnt++;
        ufo::map::OccupancyState state = map.getState(*it);

        switch (state)
        {
          case ufo::map::OccupancyState::occupied:  // If the node is occupied, we
                                                    // want to know less about it
                                                    // --> Onyl small increae
            info_val = info_val + (1 - map.getOccupancy(*it));
            occupied = true;
            break;
          case ufo::map::OccupancyState::free:  // If the node is free,  we
            // want to know less about it
            // --> Onyl small increae
            info_val = info_val + (map.getOccupancy(*it));
            break;
          case ufo::map::OccupancyState::unknown:  // If the node is unknown -->
                                                   // We want to know more
            info_val++;
            break;
        }
      }
    }

    // ROS_WARN_STREAM("allNodesOnCast.size() = " << allNodesOnCast.size());
    // ROS_WARN_STREAM("info_val = " << info_val);
    // ROS_WARN_STREAM("info_val = " << info_val / cnt);
    // ROS_WARN_STREAM("accumulated_info_val_normalized_ = " << _accumulated_info_val_normalized);

    if (cnt > 0)
    {
      _accumulated_info_val_normalized += (info_val / allNodesOnCast.size());
      _info_metric_matrix(start_point_id, end_point_id) = (info_val / allNodesOnCast.size());
      end_point_id++;
    };
  }

  return true;
}

bool Server::calculateOcclusionForPerspective(auto& map, int start_point_id)
{
  ROS_WARN("Start calculateOcclusionForPerspective");
  _accumulated_occ_val_normalized = 0.0;
  _occ_mutex.lock();
  std::vector<ufo::map::Point3> ray_ep = _ray_endpoints_occlusion[start_point_id];
  _occ_mutex.unlock();
  for (int pred_step = 0; pred_step < _pred_steps_occ; pred_step++)
  {
    int end_point_id = 0;

    for (auto endpoint_it = ray_ep.begin(); endpoint_it != ray_ep.end(); endpoint_it++)
    {
      double occ_val = 0;
      // Get the keys of all the nodes on the ray
      // ROS_WARN("Start computeRay");
      ufo::map::CodeRay allNodesOnCast =
          map.computeRay(std::get<1>(_start_points_occlusion.at(start_point_id)), *endpoint_it);

      // Iterate over the ufomap nodes on the ray
      int cnt = 0;
      bool occupied = false;
      for (auto it = allNodesOnCast.begin(); it != allNodesOnCast.end(); it++)
      {
        // Search for the node in the octree
        if (!occupied)
        {
          cnt++;
          ufo::map::OccupancyState state = map.getState(*it);
          ufo::map::Prediction pred = map.getPrediction(*it);
          ufo::map::Color color = map.getColor(*it);

          if (pred_step == 0 && color.r != 255)
          {
            switch (state)
            {
              case ufo::map::OccupancyState::occupied:  // if the node is occupied it is occluded
                occ_val = 0;
                occupied = true;
                break;
              case ufo::map::OccupancyState::free:  // If the node is free,  the hands are not occluded
                occ_val++;
                break;
              case ufo::map::OccupancyState::unknown:  // If the node is unknown the hands are maybe occluded
                // TODO(renz): Maybe add a smaller increase here instead of 1
                occ_val++;
                break;
            }
          }
          else if (pred_step == 0 && color.r == 255)
          {
            occ_val++;
          }
          else if (pred_step > 0)
          {
            if (pred.getPredictionAtElement(pred_step))
            {
              occ_val = 0;
              occupied = true;
            }
            else
            {
              occ_val++;
            }
          }
        }
      }

      if (cnt > 0)
      {
        _occ_mutex.lock();  // required else interference with targetCallback thread
        _accumulated_occ_val_normalized += (occ_val / cnt);
        _occlusion_info_matrix(pred_step, end_point_id) = (occ_val / cnt);
        _occ_mutex.unlock();

        end_point_id++;
      };
    }
  }
  return true;
}

bool Server::calculateStartPoints(const ufo::map::Point3& poi, double distance)
{
  // Calculate the number of start points
  // cubical grid of start points around the poi with size distance
  // currently only the 8 corner points
  switch (_startpoint_mode)
  {
    case CORNER: {  // Case as explicit code block
      int num_start_points_corner = 8;
      // Resize the vector of start points
      _start_points.resize(num_start_points_corner);
      // Generate corner points with orientation towards POI
      for (int i = 0; i < num_start_points_corner; i++)
      {
        ufo::map::Point3 start_point = ufo::map::Point3(poi(0) + distance / 2 * (i % 2 == 0 ? 1 : -1),
                                                        poi(1) + distance / 2 * ((i / 2) % 2 == 0 ? 1 : -1),
                                                        poi(2) + distance / 2 * ((i / 4) % 2 == 0 ? 1 : -1));
        Eigen::AngleAxisd axis(
            0, Eigen::Vector3d{ poi.x() - start_point.x(), poi.y() - start_point.y(), poi.z() - start_point.z() }
                   .normalized());
        _start_points.at(i) = std::make_tuple(i, start_point, axis);
      }
      break;
    }
    case RANDOM: {
      // Resize the vector of start points
      int number_points = _num_startpoints_random * 10;
      _start_points.resize(_num_startpoints_random);

      // Define the area around the POI in which we want to generate random start points based on the distance input
      // Eigen::Rand::P8_mt19937_64 _urng{42};  // Using same seed to get same startpoints each cycle

      Eigen::MatrixXf randX(number_points, 3);
      randX = Eigen::Rand::normal<Eigen::MatrixXf>(number_points, 3, _urng);
      Eigen::VectorXf randU(number_points);
      randU = Eigen::Rand::uniformReal<Eigen::VectorXf>(number_points, 1, _urng);
      randU = randU.array().pow(1.0 / 3.0) * (distance);

      Eigen::MatrixXf randX_norm = (randX.rowwise().normalized().array().colwise() * randU.array()).matrix();
      // std::cout << "randX_norm = " << randX_norm<< std::endl;
      int num_start_point_cnt = 0;

      // Filter all start points outside robot reachability (base height 0.9273m)
      // Substract robot base from start points
      // Eigen::MatrixXf randX_norm_robot = randX_norm.rowwise() - robot_base.transpose();
      // Calculate the distance from the robot base
      Eigen::Vector3f poi_eigen = Eigen::Vector3f(poi.x(), poi.y(), poi.z());
      Eigen::VectorXf dist_robot =
          ((randX_norm.rowwise() + poi_eigen.transpose()).rowwise() - _robot_base.transpose()).rowwise().norm();

      for (int i = 0; i < number_points; i++)
      {
        // if ((randX_norm.block(i, 0, 1, 3) - _robot_base.transpose()).norm() < _reachability)
        if (_restrict_to_reachable)
        {
          if ((randX_norm(i, 2) + poi(2)) > _ground_height && dist_robot(i) < _reachability)
          {
            ufo::map::Point3 start_point =
                ufo::map::Point3(randX_norm(i, 0) + poi(0), randX_norm(i, 1) + poi(1), randX_norm(i, 2) + poi(2));
            Eigen::AngleAxisd axis(
                0, Eigen::Vector3d{ poi.x() - start_point.x(), poi.y() - start_point.y(), poi.z() - start_point.z() }
                       .normalized());
            _start_points.at(num_start_point_cnt) = std::make_tuple(num_start_point_cnt, start_point, axis);
            num_start_point_cnt++;
            if (num_start_point_cnt == _num_startpoints_random)
              break;
          }
        }
        else
        {
          if ((randX_norm(i, 2) + poi(2)) > _ground_height)
          {
            ufo::map::Point3 start_point =
                ufo::map::Point3(randX_norm(i, 0) + poi(0), randX_norm(i, 1) + poi(1), randX_norm(i, 2) + poi(2));
            Eigen::AngleAxisd axis(
                0, Eigen::Vector3d{ poi.x() - start_point.x(), poi.y() - start_point.y(), poi.z() - start_point.z() }
                       .normalized());
            _start_points.at(num_start_point_cnt) = std::make_tuple(num_start_point_cnt, start_point, axis);
            num_start_point_cnt++;
            if (num_start_point_cnt == _num_startpoints_random)
              break;
          }
        }
      }
      if (num_start_point_cnt < _num_startpoints_random)
      {
        ROS_WARN_ONCE("Not enough start points after filtering generated. Repeating start points.");
        for (int i = 0; i < _num_startpoints_random - num_start_point_cnt; i++)
        {
          _start_points.at(num_start_point_cnt + i) = _start_points.at(i);
        }
      }

      // std::cout << "Number start points = " << num_start_point_cnt << std::endl;
      // _start_points.resize(_num_startpoints_random);  // Resize the vector of filtered start points
      break;
    }
    default:
      ROS_ERROR("UFOMap_server: Startpoint mode not implemented");
      return false;
  }
  if (_plot_startpoints)
  {
    plotStartPoints();
  }
  return true;
}

void Server::saveMapTopicCallback(std_msgs::Bool::ConstPtr const& msg)
{
  if (msg->data && !_saved)
  {
    std::visit(
        [this](auto& map) {
          if constexpr (!std::is_same_v<std::decay_t<decltype(map)>, std::monostate>)
          {
            std::string map_filename;
            _nh.param<std::string>("map_filename", map_filename, "noName.um");
            map.write(map_filename);
            _saved = true;
            ROS_INFO("Map saved");
          }
        },
        _map);
  }
}

void Server::calculateInformationAndOcclusionDistributions(auto& map, sensor_msgs::PointCloud2::ConstPtr const& msg)
{
  // Distribution calculations
  // Options: information_distribution, information_distribution_gpu, occlusion_distribution_gpu (currently only GPU)

  // Information distribution
  // Idea: Sample different start points around a POI and check the perspective to build up an information distribution

  // Occlusion distribution
  // Idea: Sample different points around an POI and check the occlusion of the points to build up an occlusion
  // distribution

  // Time measurement for information distribution calculation
  auto start_time_full_info_dist = std::chrono::steady_clock::now();

  // Preparations for the information distribution calculation (preparation for occlusion is separated in extra
  // callback)
  if (_information_distribution)
  {
    _marker.header.stamp = ros::Time::now();
    _poi_pub.publish(_marker);
    double far_dist = 3.8;  // From Azure Kinect Specifications
    // Define the POI (Point of Interest) in which direct environment we want to calculate the information
    // distribution ufo::map::Point3 poi(0.0, 0.0, 0.5);          // as fixed_depth-camera coordinate

    // Time measurement for perspective calculation
    auto start_time = std::chrono::steady_clock::now();
    // define the start points around the POI from which we want to calculate the information distribution
    if (!calculateStartPoints(_poi_world, 1.0))
      ROS_ERROR("UFOMap_server: Could not calculate start points");
    if (!calculateRayEndpoints(far_dist))
      ROS_ERROR("UFOMap_server: Could not calculate ray endpoints");
    double perspective_time =
        std::chrono::duration<float, std::chrono::seconds::period>(std::chrono::steady_clock::now() - start_time)
            .count();

    if (0 == _num_perspectives || perspective_time < _min_perspective_time)
    {
      _min_perspective_time = perspective_time;
    }
    if (perspective_time > _max_perspective_time)
    {
      _max_perspective_time = perspective_time;
    }
    _accumulated_perspective_time += perspective_time;
    ++_num_perspectives;

    // Rearrange the result matrix
    _info_metric_matrix.resize(_start_points.size(), _ray_endpoints[0].size());
  }

  // Distribution calculation

  if (_information_distribution && !_information_distribution_gpu && !_occlusion_distribution)
  // Only info distribution on CPU
  {
    ROS_WARN_ONCE("UFOMap_server: Only CPU calculation of information distribution selected");

    for (auto start_it = _start_points.begin(); start_it != _start_points.end(); start_it++)
    {
      std::tuple start = *start_it;
      if (!calculateInformationForPerspective(map, std::get<0>(start)))
        ROS_ERROR("UFOMap_server: Could not calculate information distribution for perspective");
      _info_metric_vec = _info_metric_matrix.rowwise().mean();
    }
  }
  else if (_information_distribution && _information_distribution_gpu && !_occlusion_distribution)
  // Info distribution on GPU
  {
    ROS_WARN_ONCE("UFOMap_server: Only GPU calculation of information distribution selected");

    auto start_time = std::chrono::steady_clock::now();
    if constexpr (std::is_same_v<std::decay_t<decltype(map)>,
                                 ufo::map::OccupancyMapColor>)  // enter if map is colorOccupancyMap
                                                                // (currently not implemented for
                                                                // OccupancyMap) TODO(renz): Check if simple
                                                                // OccupancyMap needs changes
    {
      _map_gpu.calcInfoGain(_ray_endpoints[0].size(), 1024, &_start_points, &_ray_endpoints, -1.0, &_info_metric_matrix,
                            &_time_metric_vec);  // Max 1024 Threads per Block#
      _info_metric_vec = _info_metric_matrix.rowwise().mean();
      double distribution_gpu_time =
          std::chrono::duration<float, std::chrono::seconds::period>(std::chrono::steady_clock::now() - start_time)
              .count();

      if (0 == _num_distributions_gpu || distribution_gpu_time < _min_distribution_gpu_time)
      {
        _min_distribution_gpu_time = distribution_gpu_time;
      }
      if (distribution_gpu_time > _max_distribution_gpu_time)
      {
        _max_distribution_gpu_time = distribution_gpu_time;
      }
      _accumulated_distribution_gpu_time += distribution_gpu_time;
      ++_num_distributions_gpu;

      // Publish the GPU times (for additional information of time consumption for    each subtask)
      for (int i = 0; i < _time_metric_vec.size(); i++)
      {
        if (0 == _num_info_metrics_gpu || _time_metric_vec[i] < _min_info_metric_gpu[i])
        {
          _min_info_metric_gpu[i] = _time_metric_vec[i];
        }
        if (_time_metric_vec[i] > _max_info_metric_gpu[i])
        {
          _max_info_metric_gpu[i] = _time_metric_vec[i];
        }

        _accumulated_info_metric_gpu[i] += _time_metric_vec[i];
      }
      ++_num_info_metrics_gpu;
      publishInfoGPU();
    }
    else
    {
      ROS_ERROR("UFOMap_server: Map is not a color map");
    }
  }
  else if (_information_distribution && _information_distribution_gpu && _occlusion_distribution &&
           _occlusion_distribution_gpu)  // Info and occlusion distribution on GPU
  {
    ROS_WARN_ONCE("UFOMap_server: GPU calculation of information and occlusion distribution selected");
    auto start_time = std::chrono::steady_clock::now();
    if constexpr (std::is_same_v<std::decay_t<decltype(map)>,
                                 ufo::map::OccupancyMapColor>)  // enter if map is colorOccupancyMap
                                                                // (currently not implemented for
                                                                // OccupancyMap) TODO(renz): Check if simple
                                                                // OccupancyMap needs changes
    {
      _occ_mutex.lock();
      _map_gpu.calcInfoAndOccGain(_ray_endpoints[0].size(), _ray_endpoints_occlusion[0].size(), _pred_steps_occ, 1024,
                                  &_start_points, &_start_points_occlusion, &_ray_endpoints, &_ray_endpoints_occlusion,
                                  -1.0, &_info_metric_matrix, &_occlusion_info_matrix, &_time_metric_vec,
                                  &_time_metric_vec_occ);  // Max 1024 Threads per Block#
      _info_metric_vec = _info_metric_matrix.rowwise().mean();
      _occ_mutex.unlock();

      double distribution_gpu_time =
          std::chrono::duration<float, std::chrono::seconds::period>(std::chrono::steady_clock::now() - start_time)
              .count();

      if (0 == _num_distributions_gpu || distribution_gpu_time < _min_distribution_gpu_time)
      {
        _min_distribution_gpu_time = distribution_gpu_time;
      }
      if (distribution_gpu_time > _max_distribution_gpu_time)
      {
        _max_distribution_gpu_time = distribution_gpu_time;
      }
      _accumulated_distribution_gpu_time += distribution_gpu_time;
      ++_num_distributions_gpu;

      // Publish the GPU times (for additional information of time consumption for    each subtask)
      for (int i = 0; i < _time_metric_vec.size(); i++)
      {
        if (0 == _num_info_metrics_gpu || _time_metric_vec[i] < _min_info_metric_gpu[i])
        {
          _min_info_metric_gpu[i] = _time_metric_vec[i];
        }
        if (_time_metric_vec[i] > _max_info_metric_gpu[i])
        {
          _max_info_metric_gpu[i] = _time_metric_vec[i];
        }

        _accumulated_info_metric_gpu[i] += _time_metric_vec[i];
      }
      ++_num_info_metrics_gpu;

      // Publish the GPU times (for additional information of time consumption for each subtask)
      for (int i = 0; i < _time_metric_vec_occ.size(); i++)
      {
        if (0 == _num_occ_metrics_gpu || _time_metric_vec_occ[i] < _min_occ_metric_gpu[i])
        {
          _min_occ_metric_gpu[i] = _time_metric_vec_occ[i];
        }
        if (_time_metric_vec_occ[i] > _max_occ_metric_gpu[i])
        {
          _max_occ_metric_gpu[i] = _time_metric_vec_occ[i];
        }

        _accumulated_occ_metric_gpu[i] += _time_metric_vec_occ[i];
      }
      ++_num_occ_metrics_gpu;

      publishInfoGPU();
    }
    else
    {
      ROS_ERROR("UFOMap_server: Map is not a color map");
    }
  }
  else if (_information_distribution && _information_distribution_gpu && _occlusion_distribution &&
           !_occlusion_distribution_gpu)  // Info and occlusion distribution on GPU
  {
    ROS_WARN_ONCE(
        "UFOMap_server: GPU calculation of information distribution and CPU for occlusion distribution selected");
    auto start_time = std::chrono::steady_clock::now();
    if constexpr (std::is_same_v<std::decay_t<decltype(map)>,
                                 ufo::map::OccupancyMapColor>)  // enter if map is colorOccupancyMap
                                                                // (currently not implemented for
                                                                // OccupancyMap) TODO(renz): Check if simple
                                                                // OccupancyMap needs changes
    {
      _occ_mutex.lock();
      // Occlusion distribution on CPU
      for (auto start_it = _start_points_occlusion.begin(); start_it != _start_points_occlusion.end(); start_it++)
      {
        std::tuple start = *start_it;
        if (!calculateOcclusionForPerspective(map, std::get<0>(start)))
          ROS_ERROR("UFOMap_server: Could not calculate occlusion distribution for perspective");
      }
      _occ_mutex.unlock();
      _map_gpu.calcInfoGain(_ray_endpoints[0].size(), 1024, &_start_points, &_ray_endpoints, -1.0, &_info_metric_matrix,
                            &_time_metric_vec);  // Max 1024 Threads per Block#
      _info_metric_vec = _info_metric_matrix.rowwise().mean();

      double distribution_gpu_time =
          std::chrono::duration<float, std::chrono::seconds::period>(std::chrono::steady_clock::now() - start_time)
              .count();

      if (0 == _num_distributions_gpu || distribution_gpu_time < _min_distribution_gpu_time)
      {
        _min_distribution_gpu_time = distribution_gpu_time;
      }
      if (distribution_gpu_time > _max_distribution_gpu_time)
      {
        _max_distribution_gpu_time = distribution_gpu_time;
      }
      _accumulated_distribution_gpu_time += distribution_gpu_time;
      ++_num_distributions_gpu;

      // Publish the GPU times (for additional information of time consumption for    each subtask)
      for (int i = 0; i < _time_metric_vec.size(); i++)
      {
        if (0 == _num_info_metrics_gpu || _time_metric_vec[i] < _min_info_metric_gpu[i])
        {
          _min_info_metric_gpu[i] = _time_metric_vec[i];
        }
        if (_time_metric_vec[i] > _max_info_metric_gpu[i])
        {
          _max_info_metric_gpu[i] = _time_metric_vec[i];
        }

        _accumulated_info_metric_gpu[i] += _time_metric_vec[i];
      }
      ++_num_info_metrics_gpu;

      // Publish the GPU times (for additional information of time consumption for each subtask)
      for (int i = 0; i < _time_metric_vec_occ.size(); i++)
      {
        if (0 == _num_occ_metrics_gpu || _time_metric_vec_occ[i] < _min_occ_metric_gpu[i])
        {
          _min_occ_metric_gpu[i] = _time_metric_vec_occ[i];
        }
        if (_time_metric_vec_occ[i] > _max_occ_metric_gpu[i])
        {
          _max_occ_metric_gpu[i] = _time_metric_vec_occ[i];
        }

        _accumulated_occ_metric_gpu[i] += _time_metric_vec_occ[i];
      }
      ++_num_occ_metrics_gpu;

      publishInfoGPU();
    }
    else
    {
      ROS_ERROR("UFOMap_server: Map is not a color map");
    }
  }
  else if (_information_distribution && !_information_distribution_gpu && _occlusion_distribution &&
           _occlusion_distribution_gpu)
  // Info distribution on CPU and occlusion distribution on GPU
  {
    ROS_WARN_ONCE(
        "UFOMap_server: Only GPU calculation of occlusion distribution selected; Information distribution on CPU");
    auto start_time = std::chrono::steady_clock::now();
    for (auto start_it = _start_points.begin(); start_it != _start_points.end(); start_it++)
    {
      std::tuple start = *start_it;
      if (!calculateInformationForPerspective(map, std::get<0>(start)))
        ROS_ERROR("UFOMap_server: Could not calculate information distribution for perspective");
      _info_metric_vec = _info_metric_matrix.rowwise().mean();
    }
    auto start_time_gpu = std::chrono::steady_clock::now();

    if constexpr (std::is_same_v<std::decay_t<decltype(map)>,
                                 ufo::map::OccupancyMapColor>)  // enter if map is colorOccupancyMap
                                                                // (currently not implemented for
                                                                // OccupancyMap) TODO(renz): Check if simple
                                                                // OccupancyMap needs changes
    {
      _occ_mutex.lock();
      _map_gpu.calcOccGain(_ray_endpoints_occlusion[0].size(), _pred_steps_occ, 1024, &_start_points_occlusion,
                           &_ray_endpoints_occlusion, -1.0, &_occlusion_info_matrix,
                           &_time_metric_vec_occ);  // Max 1024 Threads per Block#
      _occ_mutex.unlock();
      _info_metric_vec = _info_metric_matrix.rowwise().mean();

      double distribution_gpu_time =
          std::chrono::duration<float, std::chrono::seconds::period>(std::chrono::steady_clock::now() - start_time_gpu)
              .count();

      if (0 == _num_distributions_gpu || distribution_gpu_time < _min_distribution_gpu_time)
      {
        _min_distribution_gpu_time = distribution_gpu_time;
      }
      if (distribution_gpu_time > _max_distribution_gpu_time)
      {
        _max_distribution_gpu_time = distribution_gpu_time;
      }
      _accumulated_distribution_gpu_time += distribution_gpu_time;
      ++_num_distributions_gpu;

      // Publish the GPU times (for additional information of time consumption for each subtask)
      for (int i = 0; i < _time_metric_vec_occ.size(); i++)
      {
        if (0 == _num_occ_metrics_gpu || _time_metric_vec_occ[i] < _min_occ_metric_gpu[i])
        {
          _min_occ_metric_gpu[i] = _time_metric_vec_occ[i];
        }
        if (_time_metric_vec_occ[i] > _max_occ_metric_gpu[i])
        {
          _max_occ_metric_gpu[i] = _time_metric_vec_occ[i];
        }

        _accumulated_occ_metric_gpu[i] += _time_metric_vec_occ[i];
      }
      ++_num_occ_metrics_gpu;

      publishInfoGPU();
    }
    else
    {
      ROS_ERROR("UFOMap_server: Map is not a color map");
    }
  }
  else if (!_information_distribution && _occlusion_distribution && _occlusion_distribution_gpu)
  // Only occlusion distribution on GPU
  {
    ROS_WARN_ONCE("UFOMap_server: Only GPU calculation of occlusion distribution selected");
    auto start_time = std::chrono::steady_clock::now();
    if constexpr (std::is_same_v<std::decay_t<decltype(map)>,
                                 ufo::map::OccupancyMapColor>)  // enter if map is colorOccupancyMap
                                                                // (currently not implemented for
                                                                // OccupancyMap) TODO(renz): Check if simple
                                                                // OccupancyMap needs changes
    {
      _occ_mutex.lock();
      _map_gpu.calcOccGain(_ray_endpoints_occlusion[0].size(), _pred_steps_occ, 1024, &_start_points_occlusion,
                           &_ray_endpoints_occlusion, -1.0, &_occlusion_info_matrix,
                           &_time_metric_vec_occ);  // Max 1024 Threads per Block#
      _occ_mutex.unlock();
      _info_metric_vec = _info_metric_matrix.rowwise().mean();

      double distribution_gpu_time =
          std::chrono::duration<float, std::chrono::seconds::period>(std::chrono::steady_clock::now() - start_time)
              .count();

      if (0 == _num_distributions_gpu || distribution_gpu_time < _min_distribution_gpu_time)
      {
        _min_distribution_gpu_time = distribution_gpu_time;
      }
      if (distribution_gpu_time > _max_distribution_gpu_time)
      {
        _max_distribution_gpu_time = distribution_gpu_time;
      }
      _accumulated_distribution_gpu_time += distribution_gpu_time;
      ++_num_distributions_gpu;

      // Publish the GPU times (for additional information of time consumption for each subtask)
      for (int i = 0; i < _time_metric_vec_occ.size(); i++)
      {
        if (0 == _num_occ_metrics_gpu || _time_metric_vec_occ[i] < _min_occ_metric_gpu[i])
        {
          _min_occ_metric_gpu[i] = _time_metric_vec_occ[i];
        }
        if (_time_metric_vec_occ[i] > _max_occ_metric_gpu[i])
        {
          _max_occ_metric_gpu[i] = _time_metric_vec_occ[i];
        }

        _accumulated_occ_metric_gpu[i] += _time_metric_vec_occ[i];
      }
      ++_num_occ_metrics_gpu;

      publishInfoGPU();
    }
    else
    {
      ROS_ERROR("UFOMap_server: Map is not a color map");
    }
  }
  else if (!_information_distribution && _occlusion_distribution && !_occlusion_distribution_gpu)
  // Only occlusion distribution on CPU
  {
    ROS_WARN_ONCE("UFOMap_server: Only CPU calculation of occlusion distribution selected");
    auto start_time = std::chrono::steady_clock::now();
    if constexpr (std::is_same_v<std::decay_t<decltype(map)>,
                                 ufo::map::OccupancyMapColor>)  // enter if map is colorOccupancyMap
                                                                // (currently not implemented for
                                                                // OccupancyMap) TODO(renz): Check if simple
                                                                // OccupancyMap needs changes
    {
      for (auto start_it = _start_points_occlusion.begin(); start_it != _start_points_occlusion.end(); start_it++)
      {
        std::tuple start = *start_it;
        if (!calculateOcclusionForPerspective(map, std::get<0>(start)))
          ROS_ERROR("UFOMap_server: Could not calculate occlusion distribution for perspective");
      }

      double distribution_gpu_time =
          std::chrono::duration<float, std::chrono::seconds::period>(std::chrono::steady_clock::now() - start_time)
              .count();

      if (0 == _num_distributions_gpu || distribution_gpu_time < _min_distribution_gpu_time)
      {
        _min_distribution_gpu_time = distribution_gpu_time;
      }
      if (distribution_gpu_time > _max_distribution_gpu_time)
      {
        _max_distribution_gpu_time = distribution_gpu_time;
      }
      _accumulated_distribution_gpu_time += distribution_gpu_time;
      ++_num_distributions_gpu;

      // Publish the GPU times (for additional information of time consumption for each subtask)
      for (int i = 0; i < _time_metric_vec_occ.size(); i++)
      {
        if (0 == _num_occ_metrics_gpu || _time_metric_vec_occ[i] < _min_occ_metric_gpu[i])
        {
          _min_occ_metric_gpu[i] = _time_metric_vec_occ[i];
        }
        if (_time_metric_vec_occ[i] > _max_occ_metric_gpu[i])
        {
          _max_occ_metric_gpu[i] = _time_metric_vec_occ[i];
        }

        _accumulated_occ_metric_gpu[i] += _time_metric_vec_occ[i];
      }
      ++_num_occ_metrics_gpu;

      publishInfoGPU();
    }
    else
    {
      ROS_ERROR("UFOMap_server: Map is not a color map");
    }
  }
  else if (!_information_distribution && !_occlusion_distribution)
  // No distribution calculation selected
  {
    ROS_WARN_ONCE("UFOMap_server: No distribution calculation selected");
  }
  else
  {
    ROS_ERROR("UFOMap_server: No valid distribution calculation selected");
  }

  double distribution_time = std::chrono::duration<float, std::chrono::seconds::period>(
                                 std::chrono::steady_clock::now() - start_time_full_info_dist)
                                 .count();
  // std::cout << "Time difference CPU= " << distribution_time << "[s]" << std::endl;

  if (0 == _num_distributions || distribution_time < _min_distribution_time)
  {
    _min_distribution_time = distribution_time;
  }
  if (distribution_time > _max_distribution_time)
  {
    _max_distribution_time = distribution_time;
  }
  _accumulated_distribution_time += distribution_time;
  ++_num_distributions;

  // Postprocessing of the distributions
  if (_information_distribution)
  {
    // Add information distribution to the buffer and start points
    std::rotate(_info_metric_buffer.begin(), _info_metric_buffer.begin() + 1, _info_metric_buffer.end());
    _info_metric_buffer.back() = _info_metric_matrix;

    std::rotate(_start_points_buffer.begin(), _start_points_buffer.begin() + 1, _start_points_buffer.end());
    _start_points_buffer.back() = _start_points;

    // Publish the information distribution
    if (_buffer_pcl)
    {
      // Prepare array to send all pcls to planner
      mhp_robot::MsgInfoPCLS all_pcls;

      // Publish the information distribution of the last 10 frames (_buffer_size)
      for (int i = 0; i < _buffer_size; i++)
      {
        sensor_msgs::PointCloud2 msg_cloud;
        toMessage(msg_cloud, i);
        all_pcls.pcls.push_back(msg_cloud);  // oldest to newest (last element)
        _info_dist_pub_cloud[i].publish(msg_cloud);
      }

      _info_dist_pub_all.publish(all_pcls);
    }
    else
    {
      sensor_msgs::PointCloud2 msg_cloud;
      toMessage(msg_cloud);
      _info_dist_pub_cloud[0].publish(msg_cloud);
    }

    // Send the point with the highest information gain to the planner
    geometry_msgs::Pose max_info_pose;
    int idx;
    _info_metric_vec.maxCoeff(&idx);
    max_info_pose.position.x = std::get<1>(_start_points[idx]).x();
    max_info_pose.position.y = std::get<1>(_start_points[idx]).y();
    max_info_pose.position.z = std::get<1>(_start_points[idx]).z();

    Eigen::AngleAxisd aa = std::get<2>(_start_points[idx]);
    // Attention we are actually saving the axis representation instead of a quaternion
    max_info_pose.orientation.x = aa.axis()[0];
    max_info_pose.orientation.y = aa.axis()[1];
    max_info_pose.orientation.z = aa.axis()[2];
    max_info_pose.orientation.w = aa.angle();
    _max_info_point_pub.publish(max_info_pose);

    visualization_msgs::Marker max_info_point_vis;
    max_info_point_vis.header.frame_id = _frame_id;
    max_info_point_vis.header.stamp = msg->header.stamp;
    max_info_point_vis.ns = "info_dist_points_max";
    max_info_point_vis.id = 0;
    max_info_point_vis.type = visualization_msgs::Marker::ARROW;
    max_info_point_vis.action = visualization_msgs::Marker::ADD;
    geometry_msgs::Point p;
    p.x = std::get<1>(_start_points[idx]).x();
    p.y = std::get<1>(_start_points[idx]).y();
    p.z = std::get<1>(_start_points[idx]).z();
    max_info_point_vis.points.push_back(p);
    p.x = std::get<1>(_start_points[idx]).x() + std::get<2>(_start_points[idx]).axis()[0];
    p.y = std::get<1>(_start_points[idx]).y() + std::get<2>(_start_points[idx]).axis()[1];
    p.z = std::get<1>(_start_points[idx]).z() + std::get<2>(_start_points[idx]).axis()[2];
    max_info_point_vis.points.push_back(p);

    max_info_point_vis.scale.x = 0.1;
    max_info_point_vis.scale.y = 0.1;
    max_info_point_vis.scale.z = 0.1;
    max_info_point_vis.color.a = 1.0;  // Don't forget to set the alpha!
    max_info_point_vis.color.r = 1.0;
    max_info_point_vis.color.g = 1.0;
    max_info_point_vis.color.b = 0.0;
    _max_info_point_pub_vis.publish(max_info_point_vis);
  }

  if (_occlusion_distribution)
  {
    if (_occlusion_info_matrix.rows() > 0)
    {
      // Fill msg and publish occlusion distribution
      mhp_robot::MsgOcclusionDist msg;
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = _frame_id;

      for (int i = 0; i < _occlusion_info_matrix.cols(); i++)
      {
        msg.occlusion.push_back(_occlusion_info_matrix(0, i));
        msg.radius.push_back(_ray_endpoints_occlusion_sphere_coords[0][i][0]);
        msg.theta.push_back(_ray_endpoints_occlusion_sphere_coords[0][i][1]);
        msg.phi.push_back(_ray_endpoints_occlusion_sphere_coords[0][i][2]);
      }
      std_msgs::Float64MultiArray msg_pred_occlusions;

      tf::matrixEigenToMsg(_occlusion_info_matrix, msg_pred_occlusions);
      msg.pred_occlusion = msg_pred_occlusions;

      msg.center.x = std::get<1>(_start_points_occlusion[0]).x();
      msg.center.y = std::get<1>(_start_points_occlusion[0]).y();
      msg.center.z = std::get<1>(_start_points_occlusion[0]).z();

      _occlusion_distribution_pub.publish(msg);

      // Check the distance of each free occlusion point to the next occlusion point
      // TODO(renz): Check if this is needed

      double distance = 0.0;

      _occ_mutex.lock();
      std::vector<ufo::map::Point3> ray_endpoints_occ = _ray_endpoints_occlusion[0];
      _occ_mutex.unlock();
      Eigen::VectorXd distances = Eigen::VectorXd::Zero(_occlusion_info_matrix.cols());
      Eigen::VectorXd distances_to_world = 10 * Eigen::VectorXd::Ones(_occlusion_info_matrix.cols());
      Eigen::VectorXi indices = Eigen::VectorXi::Zero(_occlusion_info_matrix.cols());

      for (int i = 0; i < _occlusion_info_matrix.cols(); i++)
      {
        indices[i] = i;
        if (_occlusion_info_matrix(0, i) > 0.5)  // only rays that are not occluded
        {
          distances[i] = 10.0;
          distances_to_world[i] =
              (ray_endpoints_occ[i] - ufo::map::Point3{ _robot_base[0], _robot_base[1], _robot_base[2] })
                  .norm();  // Distance to ur10 base on pedestal
          // Check the distance to the next occlusion point
          for (int j = 0; j < _occlusion_info_matrix.cols(); j++)
          {
            if (_occlusion_info_matrix(0, j) < 0.5)  // only rays that are occluded
            {
              double dist = (ray_endpoints_occ[i] - ray_endpoints_occ[j]).norm();
              if (dist < distances[i])
              {
                if (_restrict_to_reachable_occlusion)
                {
                  // If the occlusion point is not reachable don't save the distance
                  if ((distances_to_world[i] < _reachability))
                  {
                    distances[i] = dist;
                  }
                  else
                  {
                    distances[i] = 0.0;
                  }
                }
                else
                {
                  // Save the distance and the index of the occlusion point
                  distances[i] = dist;
                }
              }
            }
          }
        }
      }
      // Find the maximum distance and the index of the occlusion point
      double max_distance = distances.maxCoeff(&_target_index);

      if (max_distance == 0)
      {
        ROS_WARN_THROTTLE(10.0,
                          "UFOMap_server: Target Point for observation occluded by obstacle; Set Target index to 1");
        _target_index = 1;
      }
      else if (max_distance == 10.0)
      {
        ROS_WARN_THROTTLE(10.0,
                          "UFOMap_server: No occlusions in the environment; Set Target index to the occlusion point "
                          "closest to the "
                          "robot base");
        distances_to_world.minCoeff(&_target_index);
      }

      // Publish the occlusion point with the highest distance to the next occlusion point
      sendTaskSpaceTarget(ray_endpoints_occ[_target_index], true);

      // Plotting
      if (_plot_occlusion_arrows)
      {
        // Delete all existing arrows of occlusion points
        visualization_msgs::Marker marker_occ;
        marker_occ.header.frame_id = _frame_id;
        marker_occ.header.stamp = ros::Time::now();
        marker_occ.ns = "occ_arrows";
        marker_occ.action = visualization_msgs::Marker::DELETEALL;
        _occ_arrow_pub.publish(marker_occ);

        // Copy to temporary variables
        _occ_mutex.lock();
        ufo::map::Point3 start_tmp = std::get<1>(_start_points_occlusion[0]);
        std::vector<ufo::map::Point3> ray_endpoints_tmp = _ray_endpoints_occlusion[0];
        Eigen::VectorXd occlusion_info_tmp = _occlusion_info_matrix.row(0);
        _occ_mutex.unlock();

        for (int i = 0; i < _occlusion_info_matrix.cols(); i++)
        {
          // check if ray endpoint is nan or empty
          plotArrow(start_tmp, ray_endpoints_tmp[i], 1, i, true,
                    occlusion_info_tmp(i) > 0.5 ? Eigen::Vector3d{ 1.0, 0.0, 0.0 } : Eigen::Vector3d{ 0.0, 0.0, 1.0 });
        }
      }
    }
  }

}  // NOLINT(readability/fn_size)

void Server::plotStartPoints() const
{
  // Plot the start point
  visualization_msgs::Marker marker_start;
  marker_start.header.frame_id = _frame_id;
  marker_start.header.stamp = ros::Time::now();
  marker_start.ns = "info_dist_points_start";
  marker_start.id = 0;
  marker_start.type = visualization_msgs::Marker::POINTS;
  marker_start.action = visualization_msgs::Marker::ADD;

  for (auto start_it = _start_points.begin(); start_it != _start_points.end(); start_it++)
  {
    geometry_msgs::Point p;
    p.x = std::get<1>(*start_it).x();
    p.y = std::get<1>(*start_it).y();
    p.z = std::get<1>(*start_it).z();
    marker_start.points.push_back(p);
  }
  // std::cout << "Number start points = " << marker_start.points.size() << std::endl;
  // marker_start.pose.position.x    = start.x();
  // marker_start.pose.position.y    = start.y();
  // marker_start.pose.position.z    = start.z();
  // marker_start.pose.orientation.w = 1.0;
  // marker_start.pose.orientation.x = 0.0;
  // marker_start.pose.orientation.y = 0.0;
  // marker_start.pose.orientation.z = 0.0;
  marker_start.scale.x = marker_start.scale.z = marker_start.scale.y = 0.02;
  marker_start.color.r = 0.0f;
  marker_start.color.g = 1.0f;
  marker_start.color.b = 0.0f;
  marker_start.color.a = 1.0;
  _start_pub.publish(marker_start);
}

void Server::plotPoi()
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "";
  marker.id = 122;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = _poi_world.x();
  marker.pose.position.y = _poi_world.y();
  marker.pose.position.z = _poi_world.z();
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0;  // Don't forget to set the alpha!
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  _marker = marker;
  _poi_pub.publish(_marker);
}

void Server::plotTarget(geometry_msgs::Pose msg)
{
  // Publish the occlusion point with the highest distance to the next occlusion point
  visualization_msgs::Marker max_occ_point_vis;
  max_occ_point_vis.header.frame_id = _frame_id;
  max_occ_point_vis.header.stamp = ros::Time::now();
  max_occ_point_vis.ns = "occlusion_points_max";
  max_occ_point_vis.id = 0;
  max_occ_point_vis.type = visualization_msgs::Marker::ARROW;
  max_occ_point_vis.action = visualization_msgs::Marker::ADD;

  max_occ_point_vis.pose = msg;
  // max_occ_point_vis.pose.position.x = _ray_endpoints_occlusion[0][_target_index].x();
  // max_occ_point_vis.pose.position.y = _ray_endpoints_occlusion[0][_target_index].y();
  // max_occ_point_vis.pose.position.z = _ray_endpoints_occlusion[0][_target_index].z();

  // Eigen::Quaterniond quaternion;
  // ufo::map::Point3 start_pos = _ray_endpoints_occlusion[0][_target_index];
  // ufo::map::Point3 target_pos = std::get<1>(_start_points_occlusion[0]);
  // Eigen::Vector3d start_pos_eigen = Eigen::Vector3d(start_pos.x(), start_pos.y(), start_pos.z());
  // Eigen::Vector3d target_pos_eigen = Eigen::Vector3d(target_pos.x(), target_pos.y(), target_pos.z());

  // geometry_msgs::TransformStamped camera_transform =
  //     _tf_buffer.lookupTransform("world", _frame_id, ros::Time(0), ros::Duration(0.5));
  // Eigen::Vector3d camera_position =
  //     Eigen::Vector3d(camera_transform.transform.translation.x, camera_transform.transform.translation.y,
  //                     camera_transform.transform.translation.z);
  // Eigen::Matrix3d rotation_basis =
  //     Eigen::Quaterniond(camera_transform.transform.rotation.w, camera_transform.transform.rotation.x,
  //                        camera_transform.transform.rotation.y, camera_transform.transform.rotation.z)
  //         .toRotationMatrix();
  // // get up vector in camera frame
  // Eigen::Vector3d up_vector = rotation_basis * Eigen::Vector3d(0, 0, 1);
  // LookAtQuat(target_pos_eigen,start_pos_eigen, up_vector, quaternion);
  // max_occ_point_vis.pose.orientation.x = quaternion.x();
  // max_occ_point_vis.pose.orientation.y = quaternion.y();
  // max_occ_point_vis.pose.orientation.z = quaternion.z();
  // max_occ_point_vis.pose.orientation.w = quaternion.w();

  max_occ_point_vis.scale.x = 1.0;
  max_occ_point_vis.scale.y = 0.1;
  max_occ_point_vis.scale.z = 0.1;
  max_occ_point_vis.color.a = 1.0;  // Don't forget to set the alpha!
  max_occ_point_vis.color.r = 1.0;
  max_occ_point_vis.color.g = 0.0;
  max_occ_point_vis.color.b = 0.0;
  _max_occ_point_pub.publish(max_occ_point_vis);
}

void Server::sendTaskSpaceTarget(ufo::map::Point3 target_endpoint, bool plot_flag)
{
  geometry_msgs::PoseStamped msg;
  msg.header.frame_id = _frame_id;
  msg.header.stamp = ros::Time::now();

  // Get translation part
  msg.pose.position.x = target_endpoint.x();
  msg.pose.position.y = target_endpoint.y();
  msg.pose.position.z = target_endpoint.z();

  // Get rotation part from end to start of the ray
  Eigen::Quaterniond quaternion;
  Eigen::Vector3d start_pos_eigen = Eigen::Vector3d(target_endpoint.x(), target_endpoint.y(), target_endpoint.z());
  Eigen::Vector3d target_pos_eigen =
      Eigen::Vector3d(std::get<1>(_start_points_occlusion[0]).x(), std::get<1>(_start_points_occlusion[0]).y(),
                      std::get<1>(_start_points_occlusion[0]).z());
  geometry_msgs::TransformStamped camera_transform = _tf_buffer.lookupTransform("world", _frame_id, ros::Time(0));
  Eigen::Vector3d camera_position =
      Eigen::Vector3d(camera_transform.transform.translation.x, camera_transform.transform.translation.y,
                      camera_transform.transform.translation.z);
  Eigen::Matrix3d rotation_basis =
      Eigen::Quaterniond(camera_transform.transform.rotation.w, camera_transform.transform.rotation.x,
                         camera_transform.transform.rotation.y, camera_transform.transform.rotation.z)
          .toRotationMatrix();
  // get up vector in camera frame
  Eigen::Vector3d up_vector = rotation_basis * Eigen::Vector3d(0, 0, 1);

  LookAtQuat(target_pos_eigen, start_pos_eigen, up_vector, quaternion);

  msg.pose.orientation.x = quaternion.x();
  msg.pose.orientation.y = quaternion.y();
  msg.pose.orientation.z = quaternion.z();
  msg.pose.orientation.w = quaternion.w();

  // Publish the target
  // std::cout << "UFOMap_server: Publishing task space target at: " << msg.pose.position.x << ", " <<
  // msg.pose.position.y
  // << ", " << msg.pose.position.z << std::endl;
  _max_occ_task_space_target_pub.publish(msg);

  // Plot the target
  if (plot_flag)
  {
    plotTarget(msg.pose);
  }
}

void Server::toMessage(ufomap_bundled::MsgInfoDist& msg) const
{
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = _frame_id;

  msg.points.resize(_start_points.size());
  msg.info_gains.resize(_start_points.size());
  for (int i = 0; i < _start_points.size(); i++)
  {
    msg.points[i].x = std::get<1>(_start_points[i]).x();
    msg.points[i].y = std::get<1>(_start_points[i]).y();
    msg.points[i].z = std::get<1>(_start_points[i]).z();
    msg.info_gains[i] = _info_metric_vec[i];
  }
}

void Server::toMessage(sensor_msgs::PointCloud2& msg, int buffer_elem) const
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.resize(_start_points_buffer[buffer_elem].size());
  for (int i = 0; i < _start_points_buffer[buffer_elem].size(); i++)
  {
    float intensity = _info_metric_buffer[buffer_elem].row(i).mean();
    pcl::PointXYZI point;
    point.x = std::get<1>(_start_points_buffer[buffer_elem][i]).x();
    point.y = std::get<1>(_start_points_buffer[buffer_elem][i]).y();
    point.z = std::get<1>(_start_points_buffer[buffer_elem][i]).z();
    point.intensity = intensity;
    cloud.at(i) = point;
  }

  pcl::toROSMsg(cloud, msg);
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = _frame_id;
  // }
}

void Server::LookAtQuat(Eigen::Ref<Eigen::Vector3d> start_pos, Eigen::Ref<Eigen::Vector3d> target_pos,
                        Eigen::Ref<Eigen::Vector3d> up_vector, Eigen::Quaterniond& quaternion)
{
  // get normalized direction vector from camera to center of mass
  Eigen::Vector3d direction_vector = (target_pos - start_pos).normalized();

  // check if parallel with direction vector and change up vector
  if (up_vector.dot(direction_vector) == 1)
  {
    // get x axis
    Eigen::Vector3d x_axis = Eigen::Vector3d(1, 0, 0);
    // get up vector
    up_vector = x_axis.cross(direction_vector);
    ROS_ERROR("LookAtQuat: Parallel");
  }

  // get rotation basis in Eigen matrix
  Eigen::Matrix3d rotation_basis = Eigen::Matrix3d::Zero();
  rotation_basis << -direction_vector, up_vector.cross(-direction_vector).normalized(),
      (-direction_vector).cross(up_vector.cross(-direction_vector).normalized()).normalized();

  // get quaternion from lookat
  quaternion = Eigen::Quaterniond(rotation_basis);
}

bool Server::validMinMaxChange() const
{
  for (int i : { 0, 1, 2 })
  {
    if (_min_change_for_obs[i] > _max_change_for_obs[i])
    {
      return false;
    }
  }
  return true;
}

void Server::resetMinMaxChange()
{
  _min_change_for_obs[0] = _min_change_for_obs[1] = _min_change_for_obs[2] = std::numeric_limits<double>::max();
  _max_change_for_obs[0] = _max_change_for_obs[1] = _max_change_for_obs[2] = std::numeric_limits<double>::min();
}
}  // namespace ufomap_mapping
