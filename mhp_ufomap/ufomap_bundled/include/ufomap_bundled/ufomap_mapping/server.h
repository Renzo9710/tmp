/**
 * UFOMap Mapping
 *
 * @author D. Duberg, KTH Royal Institute of Technology, Copyright (c) 2020.
 * @see https://github.com/UnknownFreeOccupied/ufomap_mapping
 * License: BSD 3
 *
 *
 * @Modified by: Heiko Renz, 2026, Institute of Control Theory and Systems Engineering, TU Dortmund University, Germany
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

#ifndef UFO_MAP_MAPPING_SERVER_H
#define UFO_MAP_MAPPING_SERVER_H

// UFO
#include <mhp_robot/MsgErrorTracking.h>
#include <mhp_robot/MsgInfoPCLS.h>
#include <mhp_robot/MsgOcclusionDist.h>
#include <ufo/map/occupancy_map.h>
#include <ufo/map/occupancy_map_color.h>
#include <ufomap_bundled/ClearVolume.h>
#include <ufomap_bundled/GetMap.h>
#include <ufomap_bundled/MsgInfoDist.h>
#include <ufomap_bundled/Reset.h>
#include <ufomap_bundled/SaveMap.h>
#include <ufomap_bundled/ServerConfig.h>
#include <ufomap_bundled/ufomap_mapping/ufomapGPU.cuh>
// ROS
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <mhp_robot/robot_obstacle/obstacle_list.h>
#include <ros/ros.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/Marker.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <EigenRand/EigenRand>


// STD
#include <future>
#include <variant>
#include <vector>
// CUDA
#ifdef __CUDACC__
#ifndef CUDA_CALL
#define CUDA_CALL __host__ __device__
#endif
#else
#ifndef CUDA_CALL
#define CUDA_CALL
#endif
#endif

namespace ufomap_mapping
{
class Server
{
 public:
  using Ptr = std::shared_ptr<Server>;
  using UPtr = std::unique_ptr<Server>;

  Server(ros::NodeHandle& nh, ros::NodeHandle& nh_priv, int num_startpoints = 500, int num_scaling_gridpoints = 10,
         int buffer_size = 20);
  ~Server();

 private:
  void cloudCallback(sensor_msgs::PointCloud2::ConstPtr const& msg);

  void publishInfo();

  void publishInfoGPU();

  void mapConnectCallback(ros::SingleSubscriberPublisher const& pub, int depth);

  bool getMapCallback(ufomap_bundled::GetMap::Request& request, ufomap_bundled::GetMap::Response& response);

  bool clearVolumeCallback(ufomap_bundled::ClearVolume::Request& request,
                           ufomap_bundled::ClearVolume::Response& response);

  bool setVolumeCallback(ufomap_bundled::ClearVolume::Request& request,
                         ufomap_bundled::ClearVolume::Response& response);

  bool setVolumeHuman();

  bool setVolumeStaticObstacle();

  bool setVolumeDynamicObstacle();

  bool resetVolumeHuman();

  bool resetVolumeDynamicObstacle();

  bool resetCallback(ufomap_bundled::Reset::Request& request, ufomap_bundled::Reset::Response& response);

  bool saveMapCallback(ufomap_bundled::SaveMap::Request& request, ufomap_bundled::SaveMap::Response& response);

  void timerCallback(ros::TimerEvent const& event);

  void configCallback(ufomap_bundled::ServerConfig& config, uint32_t level);

  void calculateTargetFromPoint(const std_msgs::Header& msg_header);

  void targetCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

  void obstacleCallback(const mhp_robot::MsgObstacleListConstPtr& msg);

  // Modifications information distribution
  void plotArrow(const ufo::map::Point3& start, const ufo::map::Point3& end, const int type, const int id,
                 const bool mark_start, const Eigen::Vector3d color) const;

  void plotStartPoints() const;

  void plotPoi();

  void plotTarget(geometry_msgs::Pose msg);

  void sendTaskSpaceTarget(ufo::map::Point3 target_endpoint, bool plot_flag = true);

  void sendTaskSpaceTarget(bool plot_flag = true)
  {
    if (_target_index >= _ray_endpoints_occlusion[0].size())
    {
      ROS_WARN("Target index out of range");
      return;
    }
    else if (_ray_endpoints_occlusion[0].size() == 0)
    {
      ROS_WARN("No occlusion points available");
      return;
    }
    else
    {
      _occ_mutex.lock();
      ufo::map::Point3 target_endpoint = _ray_endpoints_occlusion[0][_target_index];
      _occ_mutex.unlock();
      sendTaskSpaceTarget(target_endpoint, plot_flag);
    }
  }

  bool calculateRayEndpoints(const double far_dist = 3.8);

  bool calculateInformationForPerspective(auto& map, int start_point_id);

  bool calculateOcclusionForPerspective(auto& map, int start_point_id);

  bool calculateStartPoints(const ufo::map::Point3& poi, const double distance = 3.8);

  void calculateInformationAndOcclusionDistributions(auto& map, sensor_msgs::PointCloud2::ConstPtr const& msg);

  void saveMapTopicCallback(std_msgs::Bool::ConstPtr const& msg);

  void toMessage(ufomap_bundled::MsgInfoDist& msg) const;

  void toMessage(sensor_msgs::PointCloud2& msg, int buffer_elem = 0) const;

  void LookAtQuat(Eigen::Ref<Eigen::Vector3d> start_pos, Eigen::Ref<Eigen::Vector3d> target_pos,
                  Eigen::Ref<Eigen::Vector3d> up_vector, Eigen::Quaterniond& quaternion);

  bool validMinMaxChange() const;

  void resetMinMaxChange();
 private:
  // Robot Utilities adaptions
  using ObstacleList = mhp_robot::robot_obstacle::ObstacleList;

  ros::Subscriber _virtual_obstacle_sub;
  ObstacleList _obstacle_manager;
  std::vector<mhp_robot::robot_misc::Human> _human_obstacle_list;
  std::vector<mhp_robot::robot_misc::Obstacle> _static_obstacle_list;
  std::vector<mhp_robot::robot_misc::Obstacle> _dynamic_obstacle_list;

  //
  // ROS parameters
  //
  const int _buffer_size = 10;
  // Node handles
  ros::NodeHandle& _nh;
  ros::NodeHandle& _nh_priv;

  // Subscribers
  ros::Subscriber _cloud_sub;
  unsigned int _cloud_in_queue_size;
  ros::Subscriber _save_map_sub;

  // Publishers
  std::vector<ros::Publisher> _map_pub;
  unsigned int _map_queue_size;
  ros::Timer _pub_timer;
  double _pub_rate;
  ros::Duration _update_rate;
  ros::Time _last_update_time;
  ros::Publisher _info_pub;
  ros::Publisher _info_arrow_pub;  // Modification information distribution
  ros::Publisher _occ_arrow_pub;   // Modification information distribution
  ros::Publisher _start_pub;       // Modification information distribution
  ros::Publisher _info_dist_pub;   // Modification information distribution
  std::vector<ros::Publisher> _info_dist_pub_cloud =
      std::vector<ros::Publisher>(_buffer_size);  // Modification information distribution
  ros::Publisher _info_dist_pub_all;              // Modification information distribution
  ros::Publisher _gpu_time_pub;                   // Modification information distribution
  ros::Publisher _max_info_point_pub;             // Modification information distribution
  ros::Publisher _max_info_point_pub_vis;         // Modification information distribution
  ros::Publisher _occlusion_distribution_pub;     // Modification information distribution

  ros::Publisher _hand_com_pub;  // Publishing the hand center of mass in case of skeleton target in tracking
  // Services
  ros::ServiceServer _get_map_server;
  ros::ServiceServer _clear_volume_server;
  ros::ServiceServer _set_volume_server;
  ros::ServiceServer _reset_server;
  ros::ServiceServer _save_map_server;

  // TF2
  tf2_ros::Buffer _tf_buffer;
  tf2_ros::TransformListener _tf_listener;
  ros::Duration _transform_timeout;

  // Dynamic reconfigure
  dynamic_reconfigure::Server<ufomap_bundled::ServerConfig> _cs;

  //
  // UFO Parameters
  //

  // Map
  std::variant<std::monostate, ufo::map::OccupancyMap, ufo::map::OccupancyMapColor> _map;
  ufomapGPU::UFOMapGPU _map_gpu;

  std::string _frame_id;

  // Integration
  double _max_range;
  ufo::map::DepthType _insert_depth;
  bool _simple_ray_casting;
  unsigned int _early_stopping;
  bool _async;

  // Clear robot
  bool _clear_robot;
  std::string _robot_frame_id;
  double _robot_height;
  double _robot_radius;
  int _clearing_depth;

  // Publishing
  bool _compress;
  bool _update_part_of_map;
  ufo::map::DepthType _publish_depth;
  std::future<void> _update_async_handler;

  //
  // Information
  //

  // Integration
  double _min_integration_time;
  double _max_integration_time = 0.0;
  double _accumulated_integration_time = 0.0;
  int _num_integrations = 0;

  // Clear robot
  double _min_clear_time;
  double _max_clear_time = 0.0;
  double _accumulated_clear_time = 0.0;
  int _num_clears = 0;

  // Publish update
  double _min_update_time;
  double _max_update_time = 0.0;
  double _accumulated_update_time = 0.0;
  int _num_updates = 0;

  // Publish whole
  double _min_whole_time;
  double _max_whole_time = 0.0;
  double _accumulated_whole_time = 0.0;
  int _num_wholes = 0;

  // Times for occlusion area estimation
  double _min_target_time;
  double _max_target_time = 0.0;
  double _accumulated_target_time = 0.0;
  int _num_targets = 0;

  double _min_obstacle_time;
  double _max_obstacle_time = 0.0;
  double _accumulated_obstacle_time = 0.0;
  int _num_obstacles = 0;

  double _min_occlusion_time;
  double _max_occlusion_time = 0.0;
  double _accumulated_occlusion_time = 0.0;
  int _num_occlusions = 0;

  double _min_error_time;
  double _max_error_time = 0.0;
  double _accumulated_error_time = 0.0;
  int _num_errors = 0;

  double _min_delete_time;
  double _max_delete_time = 0.0;  
  double _accumulated_delete_time = 0.0;
  int _num_deletes = 0;

  double _min_all_time;
  double _max_all_time = 0.0;
  double _accumulated_all_time = 0.0;
  int _num_alls = 0;
  // Information distribution
  double _min_distribution_time;
  double _max_distribution_time = 0.0;
  double _accumulated_distribution_time = 0.0;
  int _num_distributions = 0;

  double _min_distribution_gpu_time;
  double _max_distribution_gpu_time = 0.0;
  double _accumulated_distribution_gpu_time = 0.0;
  int _num_distributions_gpu = 0;

  double _min_perspective_time;
  double _max_perspective_time = 0.0;
  double _accumulated_perspective_time = 0.0;
  int _num_perspectives = 0;

  Eigen::VectorXd _min_info_metric_gpu = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd _max_info_metric_gpu = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd _accumulated_info_metric_gpu = Eigen::VectorXd::Zero(6);
  int _num_info_metrics_gpu = 0;

  Eigen::VectorXd _min_occ_metric_gpu = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd _max_occ_metric_gpu = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd _accumulated_occ_metric_gpu = Eigen::VectorXd::Zero(6);
  int _num_occ_metrics_gpu = 0;

  // Mutex
  std::mutex _occ_mutex;
  std::mutex _map_mutex;
  // Verbose
  bool _verbose;

  // Adaptions for information distribution
  bool _information_distribution = true;      // Calculation of information distribution (CPU or GPU)
  bool _information_distribution_gpu = true;  // Calculation of information distribution on GPU (if false, CPU is used)
  bool _information_distribution_adapted =
      true;  // Adapted information distribution; Adds new points to the distirbution instead of overwriting
  bool _information_distribution_time_decrease =
      true;  // Decrease the information gain over time including a irrelevant threshold

  std::map<int, std::vector<ufo::map::Point3>> _ray_endpoints;
  std::vector<std::tuple<int, ufo::map::Point3, Eigen::AngleAxisd>> _start_points;
  std::vector<std::vector<std::tuple<int, ufo::map::Point3, Eigen::AngleAxisd>>> _start_points_buffer =
      std::vector<std::vector<std::tuple<int, ufo::map::Point3, Eigen::AngleAxisd>>>(_buffer_size);
  std::map<std::string, ufo::map::Point3> _frustrum_points;  // order: up_left; up_right; down_left; down_right
  bool _plot_arrows = true;
  bool _plot_startpoints = true;
  enum gridpoint_mode
  {
    CENTER,
    FRUSTRUM,
    FULL
  };
  enum startpoint_mode
  {
    RANDOM,
    CORNER
  };
  startpoint_mode _startpoint_mode = RANDOM;
  gridpoint_mode _gridpoint_mode = FULL;
  double _ground_height = 0.0;  // Height of the ground plane; Filter all startpoints below this height
  double _reachability = 1.3;
  bool _restrict_to_reachable = true;

  bool _restrict_to_reachable_occlusion = true;
  
  Eigen::Vector3f _robot_base = Eigen::Vector3f(0.0, 0.0, 0.9273);

  bool _buffer_pcl = true;
  std::vector<Eigen::MatrixXd> _info_metric_buffer = std::vector<Eigen::MatrixXd>(_buffer_size);
  Eigen::MatrixXd _info_metric_matrix;
  Eigen::VectorXd _info_metric_vec;
  Eigen::VectorXd _time_metric_vec = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd _time_metric_vec_occ = Eigen::VectorXd::Zero(6);

  const int _num_startpoints_random;
  double _resolution;

  Eigen::Rand::P8_mt19937_64 _urng{ static_cast<std::size_t>(time(NULL)) };  // Changing seed to get new startpoints
                                                                             // each cycle
  //  Eigen::Rand::P8_mt19937_64 _urng{42};                                    // Using same seed to get same
  //  startpoints each cycle
  double _accumulated_info_val_normalized = 0.0;
  double _accumulated_occ_val_normalized = 0.0;
  int _scaling_factor_gridpoints;
  int _arrow_cnt = 0;
  ufo::map::OccupancyMapColor _map_color = ufo::map::OccupancyMapColor(0.1);
  bool _saved = false;

  // POI for information distribution
  ufo::map::Point3 _poi_world;
  ros::Publisher _poi_pub;
  visualization_msgs::Marker _marker;

  // Occlusion variables
  ros::Publisher _max_occ_point_pub;
  ros::Publisher _max_occ_task_space_target_pub;
  int _target_index = 0;
  bool _occlusion_distribution;
  bool _occlusion_distribution_gpu;

  bool _target_switch_mode = false;  // Switch between video and skeleton target
  bool _video_target = false;
  ros::Time _video_target_start_time;
  double _video_target_duration = 1.0;  // Duration to use video target before switching back to skeleton target
  bool _skeleton_target = true;
  const double _radius = 1.0;
  const double _step_size_theta = M_PI / 64;
  const double _step_size_phi = M_PI / 32;
  const double _max_phi = 2 * M_PI;
  const double _max_theta = M_PI;

  ros::Subscriber _target_sub;

  // Error calculation metrics
  ros::Publisher _error_pub;
  Eigen::Affine3d _camera_pose = Eigen::Affine3d::Identity();
  double _distance_cam_tracking = 0.0;
  double _distance_poi_to_tracking = 0.0;
  double _angle_cam_tracking = 0.0;
  bool _line_of_sight_occluded = false;
  Eigen::Vector4d _z_axis_world = Eigen::Vector4d{ 0, 0, 1, 1 };
  bool _error_calculation = true;

  // Occlusion matrices 
  Eigen::MatrixXd _occlusion_info_matrix;
  std::map<int, std::vector<ufo::map::Point3>> _ray_endpoints_occlusion;
  std::map<int, std::vector<Eigen::Vector3d>> _ray_endpoints_occlusion_sphere_coords;
  std::vector<std::tuple<int, ufo::map::Point3, Eigen::AngleAxisd>> _start_points_occlusion;
  bool _plot_occlusion_arrows = true;
  int _pred_steps_occ;

  // Obstacle addition with prediction extension
  Eigen::Matrix4d _pose_hum_bp = Eigen::Matrix4d::Identity();
  ufo::geometry::BoundingVolume _bv_human;
  Eigen::Vector3d _tmp_translation = Eigen::Vector3d::Zero();
  std::vector<ufo::map::Color> _colors_human;
  std::vector<double> _occupancy_human;
  std::vector<std::tuple<double, ufo::map::Prediction>> _prediction_human;

  Eigen::Matrix4d _pose_static_obstacle_bp = Eigen::Matrix4d::Identity();
  ufo::geometry::BoundingVolume _bv_static_obstacle;
  std::vector<ufo::map::Color> _colors_static_obstacle;
  std::vector<double> _occupancy_static_obstacle;
  std::vector<std::tuple<double, ufo::map::Prediction>> _prediction_static_obstacle;

  Eigen::Matrix4d _pose_dynamic_obstacle_bp = Eigen::Matrix4d::Identity();
  ufo::geometry::BoundingVolume _bv_dynamic_obstacle;
  std::vector<ufo::map::Color> _colors_dynamic_obstacle;
  std::vector<double> _occupancy_dynamic_obstacle;
  std::vector<std::tuple<double, ufo::map::Prediction>> _prediction_dynamic_obstacle;

  bool _static_obstacles_added = false;
  const int _depth_human = 3;
  const int _depth_static_obstacle = 3;
  const int _depth_dynamic_obstacle = 3;

  ufo::map::Point3 _min_change_for_obs;
  ufo::map::Point3 _max_change_for_obs;
};
}  // namespace ufomap_mapping

#endif  // UFO_MAP_MAPPING_SERVER_H