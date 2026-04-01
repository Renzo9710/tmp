
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

#include <mhp_robot/robot_trajectory_optimization/robot_occlusion_potential.h>

namespace mhp_robot
{
namespace robot_trajectory_optimization
{
void RobotOcclusionPotential::computeGradient(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k,
                                              Eigen::Ref<Eigen::VectorXd> dx)
{
  Eigen::VectorXd diff = Eigen::VectorXd::Zero(x_k.size());
  for (int i = 0; i < x_k.size(); ++i)
  {
    diff(i) = _eps;
    dx(i) = (computePotentials(k, x_k + diff) - computePotentials(k, x_k - diff)) / (2 * _eps);
    diff(i) = 0.0;
  }
}

void RobotOcclusionPotential::computeHessian(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k,
                                             Eigen::Ref<Eigen::MatrixXd> dxdx)
{
  int n = x_k.size();
  Eigen::VectorXd diff = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd ldx(n), rdx(n);
  for (int i = 0; i < n; ++i)
  {
    diff(i) = _eps;
    computeGradient(k, x_k + diff, rdx);
    computeGradient(k, x_k - diff, ldx);
    dxdx.col(i) = (rdx - ldx) / (2 * _eps);
    diff(i) = 0.0;
  }
}

bool RobotOcclusionPotential::initialize(robot_kinematic::RobotKinematic::UPtr robot_kinematic)
{
  if (_initialized) return true;

  _robot_kinematic = std::move(robot_kinematic);

  // TODO(renz): Check option to avoid rosparameter server here
  ros::NodeHandle nh;
  _occlusion_dist_sub = nh.subscribe("/ufomap_server_node/occlusion_dist", 1,
                                     &RobotOcclusionPotential::occlusionDistributionCallback, this);

  _camera_frame = nh.param("/depth_camera_frame", std::string("camera_3d_depth_camera_link"));
  // with real data from bag
  tf::StampedTransform transform;
  tf::TransformListener tf_listener;
  try
  {
    ros::Time now = ros::Time::now();
    Eigen::Affine3d tmp;
    tf_listener.waitForTransform("/ee_link", _camera_frame, ros::Time(0), ros::Duration(10.0));
    tf_listener.lookupTransform("/ee_link", _camera_frame, ros::Time(0), transform);
    tf::transformTFToEigen(transform, tmp);
    _tf_cam_to_ee_link = tmp.matrix();
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }

  _initialized = true;

  return true;
}

bool RobotOcclusionPotential::update(double dt)
{
  _dt = dt;
  _occ_mutex.lock();
  _occlusion_distribution = _occlusion_distribution_cb;
  _occlusion_distribution_cartesian = _occlusion_distribution_cartesian_cb;
  _occlusion_sphere_center = _occlusion_sphere_center_cb;
  _occlusion_predictions = _occlusion_predictions_cb;
  _occ_mutex.unlock();
  return true;
}

bool RobotOcclusionPotential::isInitialized() const
{
  return _initialized;
}

void RobotOcclusionPotential::occlusionDistributionCallback(const mhp_robot::MsgOcclusionDist::ConstPtr& msg)
{
  // Fill occlusion matrix
  Eigen::MatrixXd occlusion_distribution = Eigen::MatrixXd::Zero(size(msg->occlusion), 4);
  Eigen::MatrixXd occlusion_distribution_cartesian = Eigen::MatrixXd::Zero(size(msg->occlusion), 4);

  for (int i = 0; i < size(msg->occlusion); i++)
  {
    occlusion_distribution(i, 0) = msg->radius[i];
    occlusion_distribution(i, 1) = msg->theta[i];
    occlusion_distribution(i, 2) = msg->phi[i];
    occlusion_distribution(i, 3) = msg->occlusion[i];

    Eigen::Vector3d cartesian = sphericalToCartesian(msg->radius[i], msg->theta[i], msg->phi[i]);
    occlusion_distribution_cartesian(i, 0) = cartesian(0);
    occlusion_distribution_cartesian(i, 1) = cartesian(1);
    occlusion_distribution_cartesian(i, 2) = cartesian(2);
    occlusion_distribution_cartesian(i, 3) = msg->occlusion[i];
  }

  Eigen::Vector3d sphere_center = Eigen::Vector3d::Zero();
  sphere_center(0) = msg->center.x;
  sphere_center(1) = msg->center.y;
  sphere_center(2) = msg->center.z;

  // std::cout << "Occlusion Distribution Callback" << std::endl;
  // std::cout << "Occlusion Distribution: " << occlusion_distribution << std::endl;
  // Copy prediction occlusions
  std::vector<double> tmp = msg->pred_occlusion.data;
  Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > occlusion_pred_matrix(
      tmp.data(), msg->pred_occlusion.layout.dim[0].size, msg->pred_occlusion.layout.dim[1].size);
  // Copy mutex saved into class member

  _occ_mutex.lock();
  _occlusion_distribution_cb = occlusion_distribution;
  _occlusion_distribution_cartesian_cb = occlusion_distribution_cartesian;
  _occlusion_sphere_center_cb = sphere_center;
  _occlusion_predictions_cb = occlusion_pred_matrix;
  _occ_mutex.unlock();
}

Eigen::Vector3d RobotOcclusionPotential::sphericalToCartesian(const double& radius, const double& theta,
                                                              const double& phi) const
{
  Eigen::Vector3d cartesian;
  cartesian(0) = radius * sin(theta) * cos(phi);
  cartesian(1) = radius * sin(theta) * sin(phi);
  cartesian(2) = radius * cos(theta);
  return cartesian;
}

Eigen::Vector3d RobotOcclusionPotential::cartesianToSpherical(const double x, const double y, const double z) const
{
  Eigen::Vector3d spherical;
  spherical(0) = sqrt(x * x + y * y + z * z);    // radius
  spherical(1) = atan2(sqrt(x * x + y * y), z);  // theta
  spherical(2) = atan2(y, x);                    // phi

  // wrap phi to [0, 2pi]
  if (spherical(2) < 0)
  {
    spherical(2) += 2 * M_PI;
  }

  return spherical;
}

}  // namespace robot_trajectory_optimization
}  // namespace mhp_robot
