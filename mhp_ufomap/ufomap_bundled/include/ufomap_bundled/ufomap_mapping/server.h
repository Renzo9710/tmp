/**
 * UFOMap Mapping
 *
 * @author D. Duberg, KTH Royal Institute of Technology, Copyright (c) 2020.
 * @see https://github.com/UnknownFreeOccupied/ufomap_mapping
 * License: BSD 3
 *
 *
 * @Modified by: Heiko Renz, 2024, Institute of Control Theory and Systems Engineering, TU Dortmund University, Germany
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
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <visualization_msgs/Marker.h>
#include <mhp_robot/MsgInfoPCLS.h>
// STD
#include <future>
#include <variant>
#include <vector>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <EigenRand/EigenRand>

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
      Server(ros::NodeHandle &nh, ros::NodeHandle &nh_priv, int num_startpoints = 500, int num_scaling_gridpoints = 10, int buffer_size = 20);

      ~Server();

   private:
      void cloudCallback(sensor_msgs::PointCloud2::ConstPtr const &msg);

      void publishInfo();

      void publishInfoGPU();

      void mapConnectCallback(ros::SingleSubscriberPublisher const &pub, int depth);

      bool getMapCallback(ufomap_bundled::GetMap::Request &request, ufomap_bundled::GetMap::Response &response);

      bool clearVolumeCallback(ufomap_bundled::ClearVolume::Request &request, ufomap_bundled::ClearVolume::Response &response);

      bool resetCallback(ufomap_bundled::Reset::Request &request, ufomap_bundled::Reset::Response &response);

      bool saveMapCallback(ufomap_bundled::SaveMap::Request &request, ufomap_bundled::SaveMap::Response &response);

      void timerCallback(ros::TimerEvent const &event);

      void configCallback(ufomap_bundled::ServerConfig &config, uint32_t level);

      // Modifications information distribution
      void plotArrow(const ufo::map::Point3 &start, const ufo::map::Point3 &end, const int id = 0, bool mark_start = true, bool blue = false) const;

      void plotStartPoints() const;

      void plotPoi();

      bool calculateRayEndpoints(const double far_dist = 3.8);

      bool calculateInformationForPerspective(auto &map, int start_point_id);

      bool calculateStartPoints(const ufo::map::Point3 &poi, const double distance = 3.8);

      void saveMapTopicCallback(std_msgs::Bool::ConstPtr const &msg);

      void toMessage(ufomap_bundled::MsgInfoDist &msg) const;

      void toMessage(sensor_msgs::PointCloud2 &msg, int buffer_elem = 0) const;

   private:
      //
      // ROS parameters
      //
      const int _buffer_size = 10;

      // Node handles
      ros::NodeHandle &_nh;
      ros::NodeHandle &_nh_priv;

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
      ros::Publisher _arrow_pub;                                                                    // Modification information distribution
      ros::Publisher _start_pub;                                                                    // Modification information distribution
      ros::Publisher _info_dist_pub;                                                                // Modification information distribution
      std::vector<ros::Publisher> _info_dist_pub_cloud = std::vector<ros::Publisher>(_buffer_size); // Modification information distribution
      ros::Publisher _info_dist_pub_all;                                                            // Modification information distribution
      ros::Publisher _gpu_time_pub;                                                                 // Modification information distribution
      ros::Publisher _max_info_point_pub;                                                           // Modification information distribution
      ros::Publisher _max_info_point_pub_vis;                                                       // Modification information distribution

      // Services
      ros::ServiceServer _get_map_server;
      ros::ServiceServer _clear_volume_server;
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

      Eigen::Vector4d _min_info_metric_gpu = Eigen::Vector4d::Zero();
      Eigen::Vector4d _max_info_metric_gpu = Eigen::Vector4d::Zero();
      Eigen::Vector4d _accumulated_info_metric_gpu = Eigen::Vector4d::Zero();
      int _num_info_metrics_gpu = 0;

      // Verbose
      bool _verbose;

      // Adaptions for informaiton distribution
      bool _information_distribution = false;
      bool _information_distribution_gpu = true; // Calculation of information distribution on GPU (if false, CPU is used)
      bool _information_distribution_adapted =
          true; // Adapted information distribution; Adds new points to the distirbution instead of overwriting
      bool _information_distribution_time_decrease =
          true; // Decrease the information gain over time including a irrelevant threshold
      bool _restrict_to_reachable = false;
      double _ground_height = 0.0; // Height of the ground plane; Filter all startpoints below this height
      double _reachability = 1.3;
      Eigen::Vector3f _robot_base = Eigen::Vector3f(0.0, 0.0, 0.9273);

      std::map<int, std::vector<ufo::map::Point3>> _ray_endpoints;
      std::vector<std::tuple<int, ufo::map::Point3, Eigen::AngleAxisd>> _start_points;
      std::vector<std::vector<std::tuple<int, ufo::map::Point3, Eigen::AngleAxisd>>> _start_points_buffer = std::vector<std::vector<std::tuple<int, ufo::map::Point3, Eigen::AngleAxisd>>>(_buffer_size);
      std::map<std::string, ufo::map::Point3> _frustrum_points; // order: up_left; up_right; down_left; down_right
      bool _plot_arrows = false;
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

      bool _buffer_pcl = true;
      std::vector<Eigen::MatrixXd> _info_metric_buffer = std::vector<Eigen::MatrixXd>(_buffer_size);

      Eigen::MatrixXd _info_metric_matrix;
      Eigen::VectorXd _info_metric_vec;
      Eigen::Vector4d _time_metric_vec = Eigen::Vector4d::Zero();

      const int _num_startpoints_random;
      double _resolution;

      Eigen::Rand::P8_mt19937_64 _urng{static_cast<std::size_t>(time(NULL))}; // Changing seed to get new startpoints each cycle

      double _accumulated_info_val_normalized = 0.0;
      int _scaling_factor_gridpoints;
      int _arrow_cnt = 0;
      ufo::map::OccupancyMapColor _map_color = ufo::map::OccupancyMapColor(0.1);
      bool _saved = false;

      // POI for information distribution
      ufo::map::Point3 _poi_world;
      ros::Publisher _poi_pub;
      visualization_msgs::Marker _marker;
   };
} // namespace ufomap_mapping

#endif // UFO_MAP_MAPPING_SERVER_H