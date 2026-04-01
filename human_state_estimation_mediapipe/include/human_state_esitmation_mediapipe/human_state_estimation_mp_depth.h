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

#ifndef HUMAN_STATE_ESTIMATION_MP_DEPTH_H
#define HUMAN_STATE_ESTIMATION_MP_DEPTH_H

#include <human_state_estimation_mediapipe/MsgMediapipeData.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <geometry_msgs/PoseArray.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <ros/ros.h>
#include <iostream>

#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio.hpp>

namespace human_state_estimation_mediapipe {
class HumanStateEstimationMPDepth
{
 public:
    using Ptr  = std::shared_ptr<HumanStateEstimationMPDepth>;
    using UPtr = std::unique_ptr<HumanStateEstimationMPDepth>;

    explicit HumanStateEstimationMPDepth(bool show_depth_image = true, bool save_depth_image = true);
    ~HumanStateEstimationMPDepth();

    void publish();

 private:
    ros::NodeHandle _nh;
    ros::NodeHandle _nh_pub;

    bool _is_initialized;
    bool _show_depth_image;
    bool _save_depth_image;

    cv::VideoWriter _writer;

    ros::Subscriber _rgb_image_sub;
    ros::Subscriber _depth_image_sub;
    ros::Subscriber _mp_left_hand_sub;
    ros::Subscriber _mp_right_hand_sub;
    ros::Subscriber _mp_camera_info_sub;

    tf::TransformBroadcaster _tf_broadcaster;
    tf::TransformListener _tf_listener;

    // Tuple with pixels, normalized coordinates and depth values and flag if depth value is valid
    std::tuple<Eigen::MatrixXi, Eigen::MatrixXd, Eigen::VectorXd, bool> _right_hand;
    std::tuple<Eigen::MatrixXi, Eigen::MatrixXd, Eigen::VectorXd, bool> _left_hand;
    bool _left_hand_actual  = false;
    bool _right_hand_actual = false;

    cv_bridge::CvImageConstPtr _depth_image_ptr;

    const int _filter_size         = 80;
    std::string _frame_name_camera = "camera_3d_depth_camera_link";

    bool _camera_parameters_initialized = false;
    int _image_width;
    int _image_height;
    // make matrix 3x4
    Eigen::Matrix<double, 3, 4> _image_depth_to_rgb_projection_matrix;

    bool initialize();

    void rgbImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void depthImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void mpLeftHandCallback(const human_state_estimation_mediapipe::MsgMediapipeDataConstPtr& msg);
    void mpRightHandCallback(const human_state_estimation_mediapipe::MsgMediapipeDataConstPtr& msg);
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);

    bool getPixelDepth(int x, int y, int i, bool right_hand_flag);
};

}  // namespace human_state_estimation_mediapipe

#endif  // HUMAN_STATE_ESTIMATION_MP_DEPTH_H
