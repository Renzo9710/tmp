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

#include <human_state_esitmation_mediapipe/human_state_estimation_mp_depth.h>

namespace human_state_estimation_mediapipe {
HumanStateEstimationMPDepth::HumanStateEstimationMPDepth(bool show_depth_image, bool save_depth_image)
: _nh("~"), _nh_pub(""), _show_depth_image(show_depth_image), _save_depth_image(save_depth_image)
{
    if (!initialize())
    {
        ROS_ERROR("HumanStateEstimationMPDepth: Initialization failed");
    }
    else
    {
        ROS_INFO("HumanStateEstimationMPDepth: Initialization successful");
        _is_initialized = true;
    }

    // Initialize video writer with current date and time
    auto in_time_t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::stringstream datetime;
    datetime << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S");

    // Create video path
    std::string video_path = "/home/localadmin/ur10_ws/mp_hand_estimation_depth_" + datetime.str() + ".mp4";

    // Codec choices:
    // cv::VideoWriter::fourcc('a', 'v', 'c', '1'); for mp4
    int codec = cv::VideoWriter::fourcc('h', '2', '6', '4');  // saves data compared to mp4v
    // int codec = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
    // int codec = cv::VideoWriter::fourcc('X', 'V', 'I', 'D');
    // int codec = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');

    // Initialize video writer
    _writer = cv::VideoWriter(video_path, codec, 30.0, cv::Size(1280, 720), false);

    // Check if the video writer is opened successfully
    if (!_writer.isOpened())
    {
        std::cerr << "Could not open the output video file for write\n";
    }
}

HumanStateEstimationMPDepth::~HumanStateEstimationMPDepth()
{
    _writer.release();
    cv::destroyAllWindows();
    std::cout << "HumanStateEstimationMPDepth: Destructor called, resources released." << std::endl;
}

void HumanStateEstimationMPDepth::publish()
{
    if (!_is_initialized)
    {
        ROS_ERROR("HumanStateEstimationMPDepth: Not initialized");
        return;
    }
    if (!_camera_parameters_initialized)
    {
        ROS_ERROR_THROTTLE(1, "HumanStateEstimationMPDepth: Camera parameters not initialized");
        return;
    }
    if (!std::get<3>(_left_hand))
    {
        ROS_WARN_ONCE("HumanStateEstimationMPDepth: Could not get pixel depth left hand pixels use old one; Attention only once");
    }
    else
    {
        if (_left_hand_actual)
        {
            tf::Transform transform;
            for (int i = 0; i < std::get<2>(_left_hand).size(); i++)
            {
                double x = std::get<2>(_left_hand)(i) / _image_depth_to_rgb_projection_matrix(0, 0) *
                           (std::get<0>(_left_hand)(0, i) - _image_depth_to_rgb_projection_matrix(0, 2));  // x = z/fx * (u - cx)
                double y = std::get<2>(_left_hand)(i) / _image_depth_to_rgb_projection_matrix(1, 1) *
                           (std::get<0>(_left_hand)(1, i) - _image_depth_to_rgb_projection_matrix(1, 2));  // y = z/fy * (v - cy)
                double z = std::get<2>(_left_hand)(i);
                transform.setOrigin(tf::Vector3(x, y, z));
                transform.setRotation(tf::Quaternion(0, 0, 0, 1));
                _tf_broadcaster.sendTransform(
                    tf::StampedTransform(transform, ros::Time::now(), _frame_name_camera, "left_hand_" + std::to_string(i)));
                _left_hand_actual = false;
            }
        }
    }
    if (!std::get<3>(_right_hand))
    {
        ROS_WARN_ONCE("HumanStateEstimationMPDepth: Could not get pixel depth right hand pixels use old one; Attention only once");
    }
    else
    {
        if (_right_hand_actual)
        {
            ROS_WARN_ONCE("HumanStateEstimationMPDepth: Right hand actual; Attention only once");
            _right_hand_actual = false;
            tf::Transform transform;
            for (int i = 0; i < std::get<2>(_right_hand).size(); i++)
            {
                double x = std::get<2>(_right_hand)(i) / _image_depth_to_rgb_projection_matrix(0, 0) *
                           (std::get<0>(_right_hand)(0, i) - _image_depth_to_rgb_projection_matrix(0, 2));  // x = z/fx * (u - cx)
                double y = std::get<2>(_right_hand)(i) / _image_depth_to_rgb_projection_matrix(1, 1) *
                           (std::get<0>(_right_hand)(1, i) - _image_depth_to_rgb_projection_matrix(1, 2));  // y = z/fy * (v - cy)
                double z = std::get<2>(_right_hand)(i);
                transform.setOrigin(tf::Vector3(x, y, z));
                transform.setRotation(tf::Quaternion(0, 0, 0, 1));
                _tf_broadcaster.sendTransform(
                    tf::StampedTransform(transform, ros::Time::now(), _frame_name_camera, "right_hand_" + std::to_string(i)));

                _right_hand_actual = false;
            }
        }
    }

    // Use pinhole camera model to calculate 3D position of left and right hand based on normalized coordinates and pixel
    // depth
}

bool HumanStateEstimationMPDepth::initialize()
{
    std::string camera_prefix;
    _nh.getParam("camera_prefix", camera_prefix);

    _rgb_image_sub = _nh_pub.subscribe(camera_prefix + "/rgb/image_rect_color", 1, &HumanStateEstimationMPDepth::rgbImageCallback, this);

    _depth_image_sub =
        _nh_pub.subscribe(camera_prefix + "/depth_to_rgb/hw_registered/image_rect", 1, &HumanStateEstimationMPDepth::depthImageCallback, this);

    _mp_left_hand_sub  = _nh_pub.subscribe("left_hand_pose/data", 1, &HumanStateEstimationMPDepth::mpLeftHandCallback, this);
    _mp_right_hand_sub = _nh_pub.subscribe("right_hand_pose/data", 1, &HumanStateEstimationMPDepth::mpRightHandCallback, this);

    _mp_camera_info_sub = _nh_pub.subscribe(camera_prefix + "/rgb/camera_info", 1, &HumanStateEstimationMPDepth::cameraInfoCallback, this);

    _nh_pub.param("/depth_camera_frame", _frame_name_camera, std::string("camera_3d_depth_camera_link"));
    _frame_name_camera = "camera_3d_depth_camera_link";
    // sleep
    // ros::Duration(5).sleep();
    std::cout << "WAiting for transform from world to camera frame"
              << _tf_listener.waitForTransform("world", _frame_name_camera, ros::Time(0), ros::Duration(10.0)) << std::endl;
    if (!_tf_listener.waitForTransform("world", _frame_name_camera, ros::Time(0), ros::Duration(10.0)))
    {
        ROS_ERROR("HumanStateEstimationMPDepth: Could not get transform from world to camera frame");
    }
    return true;
}

void HumanStateEstimationMPDepth::rgbImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_WARN_ONCE("HumanStateEstimationMPDepth: Received RGB image; No callback implemented");
}

void HumanStateEstimationMPDepth::depthImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_INFO_ONCE("HumanStateEstimationMPDepth: Received Depth image");
    try
    {
        _depth_image_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("HumanStateEstimationMPDepth: cv_bridge exception: %s", e.what());
        return;
    }

    if (_show_depth_image)
    {
        // show depth image
        cv::Mat depth_image;

        // normalize depth image for visualization using 16UC1 or 8UC1
        // cv::normalize(_depth_image_ptr->image, depth_image, 0, 65535, cv::NORM_MINMAX, CV_16UC1);
        cv::normalize(_depth_image_ptr->image, depth_image, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        if (_save_depth_image)
        {
            // convert to 8uc1 (only if not already)
            // cv::Mat depth_image_8uc1;
            // cv::normalize(_depth_image_ptr->image, depth_image_8uc1, 0, 255, cv::NORM_MINMAX, CV_8UC1);

            // write depth image to video
            _writer.write(depth_image);
        }

        cv::imshow("Depth Image", depth_image);
        cv::waitKey(1);
    }
}

void HumanStateEstimationMPDepth::mpLeftHandCallback(const human_state_estimation_mediapipe::MsgMediapipeDataConstPtr& msg)
{
    ROS_INFO_ONCE("HumanStateEstimationMPDepth: Received left hand pose");

    if (msg->pixels.poses.size() != msg->coordinates.poses.size())
    {
        ROS_ERROR("HumanStateEstimationMPDepth: Number of pixel and normalized coordinates do not match");
        return;
    }

    // Resize Eigen Matrix
    std::get<0>(_left_hand).resize(3, msg->pixels.poses.size());
    std::get<1>(_left_hand).resize(3, msg->coordinates.poses.size());
    std::get<2>(_left_hand).resize(msg->pixels.poses.size());

    // Iterate over poses
    for (int i = 0; i < msg->pixels.poses.size(); ++i)
    {
        // Place each position element in Eigen Matrix
        std::get<0>(_left_hand)(0, i) = static_cast<int>(msg->pixels.poses[i].position.x);
        std::get<0>(_left_hand)(1, i) = static_cast<int>(msg->pixels.poses[i].position.y);
        std::get<0>(_left_hand)(2, i) = static_cast<int>(msg->pixels.poses[i].position.z);

        if (!getPixelDepth(std::get<0>(_left_hand)(0, i), std::get<0>(_left_hand)(1, i), i, false) && std::get<3>(_left_hand))
        {
            ROS_ERROR_ONCE("HumanStateEstimationMPDepth: Could not get pixel depth (left hand callback); Attention only once");
            std::get<3>(_left_hand) = false;
            return;
        }
        else
        {
            std::get<3>(_left_hand) = true;
        }
        // print depth value at pixel, include left hand as name
        // ROS_INFO("Depth at left hand pixel (%d, %d): %f", std::get<0>(_left_hand)(0, i), std::get<0>(_left_hand)(1, i),
        // std::get<1>(_left_hand)(i));

        std::get<1>(_left_hand)(0, i) = msg->coordinates.poses[i].position.x;
        std::get<1>(_left_hand)(1, i) = msg->coordinates.poses[i].position.y;
        std::get<1>(_left_hand)(2, i) = msg->coordinates.poses[i].position.z;
    }
    _left_hand_actual = true;
}

void HumanStateEstimationMPDepth::mpRightHandCallback(const human_state_estimation_mediapipe::MsgMediapipeDataConstPtr& msg)
{
    ROS_INFO_ONCE("HumanStateEstimationMPDepth: Received right hand pose");

    if (msg->pixels.poses.size() != msg->coordinates.poses.size())
    {
        ROS_ERROR("HumanStateEstimationMPDepth: Number of pixel and normalized coordinates do not match");
        return;
    }

    // Resize Eigen Matrix
    std::get<0>(_right_hand).resize(3, msg->pixels.poses.size());
    std::get<1>(_right_hand).resize(3, msg->coordinates.poses.size());
    std::get<2>(_right_hand).resize(msg->pixels.poses.size());

    // Iterate over poses
    for (int i = 0; i < msg->pixels.poses.size(); ++i)
    {
        // Place each position element in Eigen Matrix
        std::get<0>(_right_hand)(0, i) = static_cast<int>(msg->pixels.poses[i].position.x);
        std::get<0>(_right_hand)(1, i) = static_cast<int>(msg->pixels.poses[i].position.y);
        std::get<0>(_right_hand)(2, i) = static_cast<int>(msg->pixels.poses[i].position.z);

        if (!getPixelDepth(std::get<0>(_right_hand)(0, i), std::get<0>(_right_hand)(1, i), i, true) && std::get<3>(_right_hand))
        {
            std::get<3>(_right_hand) = false;
            ROS_ERROR_ONCE("HumanStateEstimationMPDepth: Could not get pixel depth (right hand callback); Attention only once");
            return;
        }
        else
        {
            std::get<3>(_right_hand) = true;
        }
        // print depth value at pixel, include right hand as name
        // ROS_INFO("Depth at right hand pixel (%d, %d): %f", std::get<0>(_right_hand)(0, i),
        // std::get<0>(_right_hand)(1, i), std::get<1>(_right_hand)(i));

        std::get<1>(_right_hand)(0, i) = msg->coordinates.poses[i].position.x;
        std::get<1>(_right_hand)(1, i) = msg->coordinates.poses[i].position.y;
        std::get<1>(_right_hand)(2, i) = msg->coordinates.poses[i].position.z;
    }
    _right_hand_actual = true;
}

void HumanStateEstimationMPDepth::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
{
    ROS_INFO_ONCE("HumanStateEstimationMPDepth: Received Camera Info; No callback implemented");

    if (_camera_parameters_initialized)
    {
        return;
    }
    else
    {
        _image_width  = msg->width;
        _image_height = msg->height;

        // P is row major
        _image_depth_to_rgb_projection_matrix(0, 0) = msg->P[0];
        _image_depth_to_rgb_projection_matrix(0, 1) = msg->P[1];
        _image_depth_to_rgb_projection_matrix(0, 2) = msg->P[2];
        _image_depth_to_rgb_projection_matrix(0, 3) = msg->P[3];
        _image_depth_to_rgb_projection_matrix(1, 0) = msg->P[4];
        _image_depth_to_rgb_projection_matrix(1, 1) = msg->P[5];
        _image_depth_to_rgb_projection_matrix(1, 2) = msg->P[6];
        _image_depth_to_rgb_projection_matrix(1, 3) = msg->P[7];
        _image_depth_to_rgb_projection_matrix(2, 0) = msg->P[8];
        _image_depth_to_rgb_projection_matrix(2, 1) = msg->P[9];
        _image_depth_to_rgb_projection_matrix(2, 2) = msg->P[10];
        _image_depth_to_rgb_projection_matrix(2, 3) = msg->P[11];
        // print projection matrix
        // std::cout << "Projection Matrix: " << _image_depth_to_rgb_projection_matrix << std::endl;

        _camera_parameters_initialized = true;
    }
}

bool HumanStateEstimationMPDepth::getPixelDepth(int x, int y, int i, bool right_hand_flag)
{
    if (_depth_image_ptr == nullptr)
    {
        ROS_ERROR_ONCE("HumanStateEstimationMPDepth: Depth image not available");
        return -1;
    }

    if (x < 0 || x >= _depth_image_ptr->image.cols || y < 0 || y >= _depth_image_ptr->image.rows)
    {
        ROS_ERROR("HumanStateEstimationMPDepth: Pixel coordinates out of bounds");
        return -1;
    }

    // check if depth value is valid else print warning and check values around pixel
    if (std::isnan(_depth_image_ptr->image.at<float>(y, x)))
    {
        // ROS_WARN("HumanStateEstimationMPDepth: Depth value at pixel (%d, %d) is NaN", x, y);
        // take a window of 10x10 pixels around the pixel with the roi function of opencv

        cv::Rect roi(x - _filter_size / 2, y - _filter_size / 2, _filter_size, _filter_size);
        // check if roi is out of bounds
        if (roi.x < 0)
        {
            roi.x = 0;
        }
        if (roi.y < 0)
        {
            roi.y = 0;
        }
        if (roi.x + roi.width > _depth_image_ptr->image.cols)
        {
            roi.width = _depth_image_ptr->image.cols - roi.x;
        }
        if (roi.y + roi.height > _depth_image_ptr->image.rows)
        {
            roi.height = _depth_image_ptr->image.rows - roi.y;
        }
        // print roi
        cv::Mat depth_roi = _depth_image_ptr->image(roi);

        cv::patchNaNs(depth_roi, 0.0);
        // delete all zero values in depth_roi
        std::vector<cv::Point> mask;
        cv::findNonZero(depth_roi, mask);
        Eigen::VectorXd depth_values(mask.size());
        for (int i = 0; i < mask.size(); i++)
        {
            depth_values[i] = depth_roi.at<float>(mask[i].y, mask[i].x);
        }

        if (depth_values.size() == 0)
        {
            return false;
        }
        // calculate average depth value
        double avg_depth = depth_values.mean();

        // return average depth value
        if (right_hand_flag)
        {
            std::get<2>(_right_hand)(i) = avg_depth;
        }
        else
        {
            std::get<2>(_left_hand)(i) = avg_depth;
        }
        return true;
    }
    // print depth value at pixel
    // ROS_INFO("Depth at pixel (%d, %d): %f", x, y, _depth_image_ptr->image.at<float>(y, x));

    // return depth value at pixel
    if (right_hand_flag)
    {
        std::get<2>(_right_hand)(i) = _depth_image_ptr->image.at<float>(y, x);
    }
    else
    {
        std::get<2>(_left_hand)(i) = _depth_image_ptr->image.at<float>(y, x);
    }
    return true;
}
}  // namespace human_state_estimation_mediapipe
