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
    Server::Server(ros::NodeHandle &nh, ros::NodeHandle &nh_priv, int num_startpoints, int num_scaling_gridpoints, int buffer_size)
        : _nh(nh), _nh_priv(nh_priv), _tf_listener(_tf_buffer), _cs(nh_priv), _num_startpoints_random(num_startpoints), _buffer_size(buffer_size)
    {
        // Set up map
        double resolution = _nh_priv.param("resolution", 0.05);
        _resolution = resolution;
        ufo::map::DepthType depth_levels = _nh_priv.param("depth_levels", 16);

        _information_distribution = _nh_priv.param("information_distribution", false);
        _information_distribution_gpu = _nh_priv.param("information_distribution_gpu", false);
        _buffer_pcl = _nh_priv.param("buffer_pcl", false);
        std::vector<double> poi_world = _nh_priv.param("poi_world", std::vector<double>{1.37, 0.16, 0.2});
        _poi_world = ufo::map::Point3(poi_world[0], poi_world[1], poi_world[2]);
        std::cout << "POI World: " << _poi_world.x() << " " << _poi_world.y() << " " << _poi_world.z() << std::endl;

        _poi_pub = _nh_priv.advertise<visualization_msgs::Marker>("poi", 100, false);
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
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        _marker = marker;
        _poi_pub.publish(_marker);

        // Automatic pruning is disabled so we can work in multiple threads for subscribers,
        // services and publishers

        if (_nh_priv.param("color_map", false))
        {
            ROS_WARN_ONCE("Color map is enabled");
            if (_information_distribution_gpu)
            {
                ROS_WARN_ONCE("GPU support activated");
                _map.emplace<ufo::map::OccupancyMapColor>(resolution, depth_levels, false);
                std::visit(
                    [this](auto &map)
                    {
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
            [this](auto &map)
            {
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
        _arrow_pub = _nh_priv.advertise<visualization_msgs::Marker>("info_dist_points", 1, false);       // Plotting arrows for each ray
        _start_pub = _nh_priv.advertise<visualization_msgs::Marker>("info_dist_points_start", 1, false); // Plotting markers for each start point
        _info_dist_pub = _nh_priv.advertise<ufomap_bundled::MsgInfoDist>("info_dist", 10, false);        //  Publishing information gains for start points
        _max_info_point_pub = _nh_priv.advertise<geometry_msgs::Pose>("info_dist_points_max", 1, false); // Send point for max information point
        _max_info_point_pub_vis =
            _nh_priv.advertise<visualization_msgs::Marker>("info_dist_points_max_vis", 1, false); // Send point for max information point

        if (_buffer_pcl)
        {
            _info_dist_pub_all = _nh_priv.advertise<mhp_robot::MsgInfoPCLS>("info_dist_cloud", 1, false);

            for (int i = 0; i < _buffer_size; i++)
            {
                std::string name = "info_dist_cloud_" + std::to_string(i);
                _info_dist_pub_cloud[i] =
                    _nh_priv.advertise<sensor_msgs::PointCloud2>(name, 10, false); //  Publishing information gains for start points
            }
        }
        else // Publish only the last information distribution
        {
            _info_dist_pub_cloud[0] =
                _nh_priv.advertise<sensor_msgs::PointCloud2>("info_dist_cloud", 10, false); //  Publishing information gains for start points
        }
        _gpu_time_pub =
            _nh_priv.advertise<diagnostic_msgs::DiagnosticStatus>("info_gpu_times", 10, false); //  Publishing information gains for start points
        // Enable services
        _get_map_server = _nh_priv.advertiseService("get_map", &Server::getMapCallback, this);
        _clear_volume_server = _nh_priv.advertiseService("clear_volume", &Server::clearVolumeCallback, this);
        _reset_server = _nh_priv.advertiseService("reset", &Server::resetCallback, this);
        _save_map_server = _nh_priv.advertiseService("save_map", &Server::saveMapCallback, this);

        _scaling_factor_gridpoints = num_scaling_gridpoints;
    }

    Server::~Server() { _map_gpu.clearMapGPU(); }

    void Server::cloudCallback(sensor_msgs::PointCloud2::ConstPtr const &msg)
    {
        ufo::math::Pose6 transform;
        try
        {
            transform =
                ufomap_ros::rosToUfo(_tf_buffer.lookupTransform(_frame_id, msg->header.frame_id, msg->header.stamp, _transform_timeout).transform);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN_THROTTLE(1, "%s", ex.what());
            return;
        }

        std::visit(
            [this, &msg, &transform](auto &map)
            {
                if constexpr (!std::is_same_v<std::decay_t<decltype(map)>, std::monostate>) // enter if map is colorOccupancyMap or OccupancyMap
                {
                    auto start = std::chrono::steady_clock::now();

                    // Update map
                    ufo::map::PointCloudColor cloud;
                    ufomap_ros::rosToUfo(*msg, cloud);
                    cloud.transform(transform, true);

                    map.insertPointCloudDiscrete(transform.translation(), cloud, _max_range, _insert_depth, _simple_ray_casting, _early_stopping, _async);

                    double integration_time =
                        std::chrono::duration<float, std::chrono::seconds::period>(std::chrono::steady_clock::now() - start).count();

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
                                _tf_buffer.lookupTransform(_frame_id, _robot_frame_id, msg->header.stamp, _transform_timeout).transform);
                        }
                        catch (tf2::TransformException &ex)
                        {
                            ROS_WARN_THROTTLE(1, "%s", ex.what());
                            return;
                        }

                        ufo::map::Point3 r(_robot_radius, _robot_radius, _robot_height / 2.0);
                        ufo::geometry::AABB aabb(transform.translation() - r, transform.translation() + r);
                        map.setValueVolume(aabb, map.getClampingThresMin(), _clearing_depth);

                        double clear_time = std::chrono::duration<float, std::chrono::seconds::period>(std::chrono::steady_clock::now() - start).count();
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
                    if (!_map_pub.empty() && _update_part_of_map && map.validMinMaxChange() &&
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

                            ufo::geometry::AABB aabb(map.minChange(), map.maxChange());
                            // TODO: should this be here?
                            map.resetMinMaxChangeDetection();

                            _update_async_handler = std::async(std::launch::async, [this, aabb, stamp = msg->header.stamp]()
                                                               { std::visit(
                                                                     [this, &aabb, stamp](auto &map)
                                                                     {
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
                                                                     _map); });

                            double update_time =
                                std::chrono::duration<float, std::chrono::seconds::period>(std::chrono::steady_clock::now() - start).count();
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

                    // Information distribution
                    // Idea: Sample different start points around a POI and check the perspective to build up an information distribution
                    if (_information_distribution)
                    {
                        ROS_WARN_ONCE("Information distribution is enabled");
                        _marker.header.stamp = ros::Time::now();
                        _poi_pub.publish(_marker);
                        double far_dist = 3.8; // From Azure Kinect Specifications
                        // Define the POI (Point of Interest) in which direct environment we want to calculate the information distribution
                        // ufo::map::Point3 poi(0.0, 0.0, 0.5);          // as fixed_depth-camera coordinate

                        // Time measurement for perspective calculation
                        auto start_time = std::chrono::steady_clock::now();
                        // define the start points around the POI from which we want to calculate the information distribution
                        if (!calculateStartPoints(_poi_world, 1.0))
                            ROS_ERROR("UFOMap_server: Could not calculate start points");
                        if (!calculateRayEndpoints(far_dist))
                            ROS_ERROR("UFOMap_server: Could not calculate ray endpoints");
                        double perspective_time =
                            std::chrono::duration<float, std::chrono::seconds::period>(std::chrono::steady_clock::now() - start_time).count();

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

                        // Time measurement for information distribution calculation
                        start_time = std::chrono::steady_clock::now();

                        // Rearrange the result matrix
                        _info_metric_matrix.resize(_start_points.size(), _ray_endpoints[0].size());

                        if (_information_distribution_gpu) // GPU calculation
                        {

                            if constexpr (std::is_same_v<std::decay_t<decltype(map)>,
                                                         ufo::map::OccupancyMapColor>) // enter if map is colorOccupancyMap (currently not implemented
                                                                                       // for OccupancyMap) TODO(renz): Check if simple OccupancyMap
                                                                                       // needs changes
                            {
                                // ufo::map::OccupancyMapColor map_color(map);
                                _map_gpu.calcInfoGain(_ray_endpoints[0].size(), 1024, &_start_points, &_ray_endpoints, -1.0, &_info_metric_matrix,
                                                      &_time_metric_vec); // Max 1024 Threads per Block#
                                _info_metric_vec = _info_metric_matrix.rowwise().mean();
                                double distribution_gpu_time =
                                    std::chrono::duration<float, std::chrono::seconds::period>(std::chrono::steady_clock::now() - start_time).count();
                                // std::cout << "Time difference GPU= " << distribution_gpu_time << "[s]" << std::endl;

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
                        else // CPU calculation
                        {
                            for (auto start_it = _start_points.begin(); start_it != _start_points.end(); start_it++)
                            {
                                std::tuple start = *start_it;
                                if (!calculateInformationForPerspective(map, std::get<0>(start)))
                                    ROS_ERROR("UFOMap_server: Could not calculate information distribution for perspective");
                                _info_metric_vec = _info_metric_matrix.rowwise().mean();
                            }
                        }

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

                            // Publish the information distribution of the last 10 frames (buffer_size)
                            for (int i = 0; i < _buffer_size; i++)
                            {
                                sensor_msgs::PointCloud2 msg_cloud;
                                toMessage(msg_cloud, i);
                                all_pcls.pcls.push_back(msg_cloud); // oldest to newest (last element)
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

                        double distribution_time =
                            std::chrono::duration<float, std::chrono::seconds::period>(std::chrono::steady_clock::now() - start_time).count();
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
                        max_info_point_vis.color.a = 1.0; // Don't forget to set the alpha!
                        max_info_point_vis.color.r = 1.0;
                        max_info_point_vis.color.g = 1.0;
                        max_info_point_vis.color.b = 0.0;
                        _max_info_point_pub_vis.publish(max_info_point_vis);
                    }
                    publishInfo();
                }
            },
            _map);
    }
    void Server::publishInfo()
    {
        if (_verbose)
        {
            printf("\nTimings:\n");
            if (0 != _num_integrations)
            {
                printf("\tIntegration time (s): %5d %09.6f\t(%09.6f +- %09.6f)\n", _num_integrations, _accumulated_integration_time,
                       _accumulated_integration_time / _num_integrations, _max_integration_time);
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
            msg.values.resize(21);
            msg.values[0].key = "Min integration time (s)";
            msg.values[0].value = std::to_string(_min_integration_time);
            msg.values[1].key = "Max integration time (s)";
            msg.values[1].value = std::to_string(_max_integration_time);
            msg.values[2].key = "Average integration time (s)";
            msg.values[2].value = std::to_string(_accumulated_integration_time / _num_integrations);
            msg.values[3].key = "Min clear time (s)";
            msg.values[3].value = std::to_string(_min_clear_time);
            msg.values[4].key = "Max clear time (s)";
            msg.values[4].value = std::to_string(_max_clear_time);
            msg.values[5].key = "Average clear time (s)";
            msg.values[5].value = std::to_string(_accumulated_clear_time / _num_clears);
            msg.values[6].key = "Min update time (s)";
            msg.values[6].value = std::to_string(_min_update_time);
            msg.values[7].key = "Max update time (s)";
            msg.values[7].value = std::to_string(_max_update_time);
            msg.values[8].key = "Average update time (s)";
            msg.values[8].value = std::to_string(_accumulated_update_time / _num_updates);
            msg.values[9].key = "Min distribution time (s)";
            msg.values[9].value = std::to_string(_min_distribution_time);
            msg.values[10].key = "Max distribution time (s)";
            msg.values[10].value = std::to_string(_max_distribution_time);
            msg.values[11].key = "Average distribution time (s)";
            msg.values[11].value = std::to_string(_accumulated_distribution_time / _num_distributions);
            msg.values[12].key = "Min whole time (s)";
            msg.values[12].value = std::to_string(_min_whole_time);
            msg.values[13].key = "Max whole time (s)";
            msg.values[13].value = std::to_string(_max_whole_time);
            msg.values[14].key = "Average whole time (s)";
            msg.values[14].value = std::to_string(_accumulated_whole_time / _num_wholes);
            msg.values[15].key = "Min perspective time (s)";
            msg.values[15].value = std::to_string(_min_perspective_time);
            msg.values[16].key = "Max perspective time (s)";
            msg.values[16].value = std::to_string(_max_perspective_time);
            msg.values[17].key = "Average perspective time (s)";
            msg.values[17].value = std::to_string(_accumulated_perspective_time / _num_perspectives);
            msg.values[18].key = "Min distribution GPU time (s)";
            msg.values[18].value = std::to_string(_min_distribution_gpu_time);
            msg.values[19].key = "Max distribution GPU time (s)";
            msg.values[19].value = std::to_string(_max_distribution_gpu_time);
            msg.values[20].key = "Average distribution GPU time (s)";
            msg.values[20].value = std::to_string(_accumulated_distribution_gpu_time / _num_distributions_gpu);
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
                printf("\tCopy Host to device time (averaged over all start points) [s]: %5f \n", _time_metric_vec[0]);
                printf("\tKernel time to process all rays [s]: %5f \n", _time_metric_vec[1]);
                printf("\tCopy device results to host time [s]: %5f \n", _time_metric_vec[2]);
                printf("\tData transfer time (Host to device and device to host) [s]: %5f \n", _time_metric_vec[3]);
                printf("\tTotal distribution time [s]: %5f \n", _time_metric_vec[3] + _time_metric_vec[1]);
            }
        }
        if (_gpu_time_pub && 0 < _gpu_time_pub.getNumSubscribers())
        {
            diagnostic_msgs::DiagnosticStatus msg;
            msg.level = diagnostic_msgs::DiagnosticStatus::OK;
            msg.name = "UFOMap GPU information distribution timings";
            msg.values.resize(20);
            msg.values[0].key = "Min Copy Host to device time (averaged over all start points) [s]";
            msg.values[0].value = std::to_string(_min_info_metric_gpu[0]);
            msg.values[1].key = "Min Kernel time to process all rays [s]";
            msg.values[1].value = std::to_string(_min_info_metric_gpu[1]);
            msg.values[2].key = "Min Copy device results to host time [s]";
            msg.values[2].value = std::to_string(_min_info_metric_gpu[2]);
            msg.values[3].key = "Min Data transfer time (Host to device and device to host) [s]";
            msg.values[3].value = std::to_string(_min_info_metric_gpu[3]);
            msg.values[4].key = "Min Total distribution time [s]";
            msg.values[4].value = std::to_string(_min_info_metric_gpu[3] + _min_info_metric_gpu[1]);

            msg.values[5].key = "Max Copy Host to device time (averaged over all start points) [s]";
            msg.values[5].value = std::to_string(_max_info_metric_gpu[0]);
            msg.values[6].key = "Max Kernel time to process all rays [s]";
            msg.values[6].value = std::to_string(_max_info_metric_gpu[1]);
            msg.values[7].key = "Max Copy device results to host time [s]";
            msg.values[7].value = std::to_string(_max_info_metric_gpu[2]);
            msg.values[8].key = "Max Data transfer time (Host to device and device to host) [s]";
            msg.values[8].value = std::to_string(_max_info_metric_gpu[3]);
            msg.values[9].key = "Max Total distribution time [s]";
            msg.values[9].value = std::to_string(_max_info_metric_gpu[3] + _max_info_metric_gpu[1]);

            msg.values[10].key = "Average Copy Host to device time (averaged over all start points) [s]";
            msg.values[10].value = std::to_string(_accumulated_info_metric_gpu[0] / _num_info_metrics_gpu);
            msg.values[11].key = "Average Kernel time to process all rays [s]";
            msg.values[11].value = std::to_string(_accumulated_info_metric_gpu[1] / _num_info_metrics_gpu);
            msg.values[12].key = "Average Copy device results to host time [s]";
            msg.values[12].value = std::to_string(_accumulated_info_metric_gpu[2] / _num_info_metrics_gpu);
            msg.values[13].key = "Average Data transfer time (Host to device and device to host) [s]";
            msg.values[13].value = std::to_string(_accumulated_info_metric_gpu[3] / _num_info_metrics_gpu);
            msg.values[14].key = "Average Total distribution time [s]";
            msg.values[14].value = std::to_string((_accumulated_info_metric_gpu[3] + _accumulated_info_metric_gpu[1]) / _num_info_metrics_gpu);

            msg.values[15].key = "Average Copy Host to device time (averaged over all start points) [Hz]";
            msg.values[15].value = std::to_string(1.0 / (_accumulated_info_metric_gpu[0] / _num_info_metrics_gpu));
            msg.values[16].key = "Average Kernel time to process all rays [Hz]";
            msg.values[16].value = std::to_string(1.0 / (_accumulated_info_metric_gpu[1] / _num_info_metrics_gpu));
            msg.values[17].key = "Average Copy device results to host time [Hz]";
            msg.values[17].value = std::to_string(1.0 / (_accumulated_info_metric_gpu[2] / _num_info_metrics_gpu));
            msg.values[18].key = "Average Data transfer time (Host to device and device to host) [Hz]";
            msg.values[18].value = std::to_string(1.0 / (_accumulated_info_metric_gpu[3] / _num_info_metrics_gpu));
            msg.values[19].key = "Average Total distribution time [Hz]";
            msg.values[19].value = std::to_string(1.0 / ((_accumulated_info_metric_gpu[3] + _accumulated_info_metric_gpu[1]) / _num_info_metrics_gpu));

            _gpu_time_pub.publish(msg);
        }
    }
    void Server::mapConnectCallback(ros::SingleSubscriberPublisher const &pub, int depth)
    {
        // When a new node subscribes we will publish the whole map to that node.

        // TODO: Make this async

        std::visit(
            [this, &pub, depth](auto &map)
            {
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

                    double whole_time = std::chrono::duration<float, std::chrono::seconds::period>(std::chrono::steady_clock::now() - start).count();
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

    bool Server::getMapCallback(ufomap_bundled::GetMap::Request &request, ufomap_bundled::GetMap::Response &response)
    {
        std::visit(
            [this, &request, &response](auto &map)
            {
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

    bool Server::clearVolumeCallback(ufomap_bundled::ClearVolume::Request &request, ufomap_bundled::ClearVolume::Response &response)
    {
        std::visit(
            [this, &request, &response](auto &map)
            {
                if constexpr (!std::is_same_v<std::decay_t<decltype(map)>, std::monostate>)
                {
                    ufo::geometry::BoundingVolume bv = ufomap_bundled::msgToUfo(request.bounding_volume);
                    for (auto &b : bv)
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

    bool Server::resetCallback(ufomap_bundled::Reset::Request &request, ufomap_bundled::Reset::Response &response)
    {
        std::visit(
            [this, &request, &response](auto &map)
            {
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

    bool Server::saveMapCallback(ufomap_bundled::SaveMap::Request &request, ufomap_bundled::SaveMap::Response &response)
    {
        std::visit(
            [this, &request, &response](auto &map)
            {
                if constexpr (!std::is_same_v<std::decay_t<decltype(map)>, std::monostate>)
                {
                    ufo::geometry::BoundingVolume bv = ufomap_bundled::msgToUfo(request.bounding_volume);
                    response.success = map.write(request.filename, bv, request.compress, request.depth, 1, request.compression_level);
                }
                else
                {
                    response.success = false;
                }
            },
            _map);
        return true;
    }

    void Server::timerCallback(ros::TimerEvent const &event)
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
                        [this, &header, i](auto &map)
                        {
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
                                    std::chrono::duration<float, std::chrono::seconds::period>(std::chrono::steady_clock::now() - start).count();
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
    void Server::configCallback(ufomap_bundled::ServerConfig &config, uint32_t level)
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
            [this, &config](auto &map)
            {
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
                _map_pub[i] =
                    _nh_priv.advertise<ufomap_bundled::UFOMapStamped>(final_topic, _map_queue_size, boost::bind(&Server::mapConnectCallback, this, _1, i),
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
    void Server::plotArrow(const ufo::map::Point3 &start, const ufo::map::Point3 &end, const int id, bool mark_start, bool blue) const
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = _frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "info_dist_points";
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
        if (blue)
        {
            marker.color.r = 0.0f;
            marker.color.g = 0.0f;
            marker.color.b = 1.0f;
        }
        else
        {
            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
        }
        marker.color.a = 0.5;
        marker.points.push_back(start_p);
        marker.points.push_back(end_p);
        _arrow_pub.publish(marker);
    }

    bool Server::calculateRayEndpoints(const double far_dist)
    {
        int plot_one = 0;
        _ray_endpoints.clear();
        for (auto start_it = _start_points.begin(); start_it != _start_points.end(); start_it++)
        {

            // Define the start point for the information distribution
            std::tuple<int, ufo::map::Point3, Eigen::AngleAxisd> &start = *start_it;
            // Define the ray endpoints for the information distribution in the current
            // perspective (all endpoints on the far plane of the depth camera
            ufo::map::Point3 end(std::get<1>(start).x() + std::get<2>(start).axis()[0] * far_dist,
                                 std::get<1>(start).y() + std::get<2>(start).axis()[1] * far_dist,
                                 std::get<1>(start).z() + std::get<2>(start).axis()[2] * far_dist);

            // Calculate the frustrum points (fixed since we assume a fixed camera perspective along the z-axis)
            double v_foi = 65 * M_PI / 180; // From Azure Kinect Specifications
            double h_foi = 75 * M_PI / 180; // From Azure Kinect Specifications
            // double v_foi  = 10 * M_PI / 180;
            // double h_foi  = 10 * M_PI / 180;
            double height = tan(v_foi / 2) * far_dist;
            double width = tan(h_foi / 2) * far_dist;

            _frustrum_points.clear();
            _frustrum_points.insert({"end", end});
            _frustrum_points.insert({"start", std::get<1>(start)});
            _frustrum_points.insert({"up_left", ufo::map::Point3(end(0) + width, end(1) - height, end(2))});
            _frustrum_points.insert({"up_right", ufo::map::Point3(end(0) + width, end(1) + height, end(2))});
            _frustrum_points.insert({"down_left", ufo::map::Point3(end(0) - width, end(1) - height, end(2))});
            _frustrum_points.insert({"down_right", ufo::map::Point3(end(0) - width, end(1) + height, end(2))});

            if (_arrow_cnt > 1000000)
            {
                _arrow_cnt = 0;
            }

            if (_plot_arrows && plot_one == 10)
            {
                plotArrow(std::get<1>(start), end, _arrow_cnt, true);
                _arrow_cnt++;

                plotArrow(std::get<1>(start), _frustrum_points.at("up_left"), _arrow_cnt, true, true);
                _arrow_cnt++;

                plotArrow(std::get<1>(start), _frustrum_points.at("up_right"), _arrow_cnt, true, true);
                _arrow_cnt++;

                plotArrow(std::get<1>(start), _frustrum_points.at("down_left"), _arrow_cnt, true, true);
                _arrow_cnt++;

                plotArrow(std::get<1>(start), _frustrum_points.at("down_right"), _arrow_cnt, true, true);
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
                        if ((i == 0 && j == 0) || (i == num_x && j == num_y))
                            continue;
                        // Calculate the current ray endpoint
                        ufo::map::Point3 stepUp;
                        ufo::map::Point3 stepRight;
                        stepUp = (_frustrum_points.at("up_left") - _frustrum_points.at("down_left")) / num_y;
                        stepRight = (_frustrum_points.at("down_right") - _frustrum_points.at("down_left")) / num_x;
                        ufo::map::Point3 ray_endpoint;
                        ray_endpoint = _frustrum_points.at("down_left") + stepUp * j + stepRight * i;
                        ray_endpoints.at(i * num_y + j) = ray_endpoint;

                        // Plot the current ray endpoint
                        if (_plot_arrows && plot_one == 10)
                        {
                            if (_arrow_cnt > 1000000)
                            {
                                _arrow_cnt = 0;
                            }
                            // std::cout<< "x = " << x << std::endl;
                            // std::cout<< "y = " << y << std::endl;
                            plotArrow(std::get<1>(start), ray_endpoints.at(i * num_y + j), _arrow_cnt);
                            _arrow_cnt++;
                        }
                    }
                }
                break;
            }
            // ROS_WARN_STREAM("Number endpoints = " << ray_endpoints.size());
            _ray_endpoints.insert({std::get<0>(start), ray_endpoints});
            plot_one++;
            // ROS_WARN_STREAM("Number endpoints = " << ray_endpoints.size());
        }
        return true;
    }
    bool Server::calculateInformationForPerspective(auto &map, int start_point_id)
    {
        _accumulated_info_val_normalized = 0.0;
        int end_point_id = 0;
        for (auto endpoint_it = _ray_endpoints[start_point_id].begin(); endpoint_it != _ray_endpoints[start_point_id].end(); endpoint_it++)
        {
            double info_val = 0;
            ufo::map::CodeRay allNodesOnCast = map.computeRay(std::get<1>(_start_points.at(start_point_id)), *endpoint_it);

            int cnt = 0;
            bool occupied = false;
            for (auto it = allNodesOnCast.begin(); it != allNodesOnCast.end(); it++)
            {
                if (!occupied)
                {
                    cnt++;
                    ufo::map::OccupancyState state = map.getState(*it);
                    switch (state)
                    {
                    case ufo::map::OccupancyState::occupied:
                        info_val = info_val + (1 - map.getOccupancy(*it));
                        occupied = true;
                        break;
                    case ufo::map::OccupancyState::free:
                        info_val = info_val + (map.getOccupancy(*it));
                        break;
                    case ufo::map::OccupancyState::unknown:
                        info_val++;
                        break;
                    }
                }
            }

            if (cnt > 0)
            {
                _accumulated_info_val_normalized += (info_val / cnt);
                _info_metric_matrix(start_point_id, end_point_id) = (info_val / cnt);
                end_point_id++;
            };
        }
        return true;
    }

    bool Server::calculateStartPoints(const ufo::map::Point3 &poi, double distance)
    {
        switch (_startpoint_mode)
        {
        case CORNER:
        {
            int num_start_points_corner = 8;
            _start_points.resize(num_start_points_corner);
            for (int i = 0; i < num_start_points_corner; i++)
            {
                ufo::map::Point3 start_point =
                    ufo::map::Point3(poi(0) + distance / 2 * (i % 2 == 0 ? 1 : -1), poi(1) + distance / 2 * ((i / 2) % 2 == 0 ? 1 : -1),
                                     poi(2) + distance / 2 * ((i / 4) % 2 == 0 ? 1 : -1));
                Eigen::AngleAxisd axis(0,
                                       Eigen::Vector3d{poi.x() - start_point.x(), poi.y() - start_point.y(), poi.z() - start_point.z()}.normalized());
                _start_points.at(i) = std::make_tuple(i, start_point, axis);
            }
            break;
        }
        case RANDOM:
        {
            int number_points = _num_startpoints_random * 10;
            _start_points.resize(_num_startpoints_random);

            Eigen::MatrixXf randX(number_points, 3);
            randX = Eigen::Rand::normal<Eigen::MatrixXf>(number_points, 3, _urng);
            Eigen::VectorXf randU(number_points);
            randU = Eigen::Rand::uniformReal<Eigen::VectorXf>(number_points, 1, _urng);
            randU = randU.array().pow(1.0 / 3.0) * (distance);

            Eigen::MatrixXf randX_norm = (randX.rowwise().normalized().array().colwise() * randU.array()).matrix();
            int num_start_point_cnt = 0;

            Eigen::Vector3f poi_eigen = Eigen::Vector3f(poi.x(), poi.y(), poi.z());
            Eigen::VectorXf dist_robot = ((randX_norm.rowwise() + poi_eigen.transpose()).rowwise() - _robot_base.transpose()).rowwise().norm();

            for (int i = 0; i < number_points; i++)
            {
                if (_restrict_to_reachable)
                {
                    if ((randX_norm(i, 2) + poi(2)) > _ground_height && dist_robot(i) < _reachability)
                    {
                        ufo::map::Point3 start_point =
                            ufo::map::Point3(randX_norm(i, 0) + poi(0), randX_norm(i, 1) + poi(1), randX_norm(i, 2) + poi(2));
                        Eigen::AngleAxisd axis(
                            0, Eigen::Vector3d{poi.x() - start_point.x(), poi.y() - start_point.y(), poi.z() - start_point.z()}.normalized());
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
                            0, Eigen::Vector3d{poi.x() - start_point.x(), poi.y() - start_point.y(), poi.z() - start_point.z()}.normalized());
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

    void Server::saveMapTopicCallback(std_msgs::Bool::ConstPtr const &msg)
    {
        if (msg->data && !_saved)
        {
            std::visit(
                [this](auto &map)
                {
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

    void Server::plotStartPoints() const
    {
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
        marker_start.scale.x = marker_start.scale.z = marker_start.scale.y = 0.02;
        marker_start.color.r = 0.0f;
        marker_start.color.g = 1.0f;
        marker_start.color.b = 0.0f;
        marker_start.color.a = 1.0;
        _start_pub.publish(marker_start);
    }

    void Server::toMessage(ufomap_bundled::MsgInfoDist &msg) const
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

    void Server::toMessage(sensor_msgs::PointCloud2 &msg, int buffer_elem) const
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
    }
} // namespace ufomap_mapping