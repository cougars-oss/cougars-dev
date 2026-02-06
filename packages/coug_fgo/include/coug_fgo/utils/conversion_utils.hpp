// Copyright (c) 2026 BYU FROST Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file conversion_utils.hpp
 * @brief Utility functions for converting between ROS 2 and GTSAM data types.
 * @author Nelson Durrant
 * @date Jan 2026
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>

#include <array>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/wrench.hpp>

namespace coug_fgo::utils
{

/**
 * @brief Converts a geometry_msgs Point to a GTSAM Point3.
 * @param msg The input Point message.
 * @return The resulting gtsam::Point3.
 */
gtsam::Point3 toGtsam(const geometry_msgs::msg::Point & msg)
{
  return {msg.x, msg.y, msg.z};
}

/**
 * @brief Converts a geometry_msgs Vector3 to a GTSAM Vector3.
 * @param msg The input Vector3 message.
 * @return The resulting gtsam::Vector3.
 */
gtsam::Vector3 toGtsam(const geometry_msgs::msg::Vector3 & msg)
{
  return {msg.x, msg.y, msg.z};
}

/**
 * @brief Converts a geometry_msgs Quaternion to a GTSAM Rot3.
 * @param msg The input Quaternion message.
 * @return The resulting gtsam::Rot3.
 */
gtsam::Rot3 toGtsam(const geometry_msgs::msg::Quaternion & msg)
{
  return gtsam::Rot3::Quaternion(msg.w, msg.x, msg.y, msg.z);
}

/**
 * @brief Converts a geometry_msgs Pose to a GTSAM Pose3.
 * @param msg The input Pose message.
 * @return The resulting gtsam::Pose3.
 */
gtsam::Pose3 toGtsam(const geometry_msgs::msg::Pose & msg)
{
  return {toGtsam(msg.orientation), toGtsam(msg.position)};
}

/**
 * @brief Converts a geometry_msgs Transform to a GTSAM Pose3.
 * @param msg The input Transform message.
 * @return The resulting gtsam::Pose3.
 */
gtsam::Pose3 toGtsam(const geometry_msgs::msg::Transform & msg)
{
  return gtsam::Pose3(toGtsam(msg.rotation), toGtsam(msg.translation));
}

/**
 * @brief Converts a geometry_msgs Wrench to a GTSAM Vector6.
 * @param msg The input Wrench message.
 * @return The resulting gtsam::Vector6 [fx, fy, fz, tx, ty, tz].
 */
gtsam::Vector6 toGtsam(const geometry_msgs::msg::Wrench & msg)
{
  gtsam::Vector6 v;
  v << msg.force.x, msg.force.y, msg.force.z, msg.torque.x, msg.torque.y, msg.torque.z;
  return v;
}

/**
 * @brief Converts a 9-element covariance array (row-major) to a GTSAM Matrix33.
 * @param cov The input 3x3 covariance array.
 * @return The resulting gtsam::Matrix33.
 */
gtsam::Matrix33 toGtsam(const std::array<double, 9> & cov)
{
  gtsam::Matrix33 m;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      m(i, j) = cov[i * 3 + j];
    }
  }
  return m;
}

/**
 * @brief Converts a 36-element covariance array (row-major) to a GTSAM Matrix66.
 * @param cov The input 6x6 covariance array.
 * @return The resulting gtsam::Matrix66.
 */
gtsam::Matrix66 toGtsam(const std::array<double, 36> & cov)
{
  gtsam::Matrix66 m;
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      m(i, j) = cov[i * 6 + j];
    }
  }
  return m;
}

/**
 * @brief Converts a std::vector to a GTSAM Vector (dynamic size).
 * @param v The input vector.
 * @return The resulting gtsam::Vector.
 */
gtsam::Vector toGtsam(const std::vector<double> & v)
{
  gtsam::Vector gtsam_v(v.size());
  for (size_t i = 0; i < v.size(); ++i) {
    gtsam_v(i) = v[i];
  }
  return gtsam_v;
}

/**
 * @brief Extracts the upper-left 3x3 block from a 36-element covariance array.
 * @param cov The input 6x6 covariance array.
 * @return The resulting gtsam::Matrix33.
 */
gtsam::Matrix33 toGtsam3x3(const std::array<double, 36> & cov)
{
  gtsam::Matrix33 m;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      m(i, j) = cov[i * 6 + j];
    }
  }
  return m;
}

/**
 * @brief Converts a vector of sigmas to a squared diagonal covariance matrix (dynamic size).
 * @param sigmas The input vector of standard deviations.
 * @return The resulting gtsam::Matrix (diagonal with sigmas squared).
 */
gtsam::Matrix toGtsamSquaredDiagonal(const std::vector<double> & sigmas)
{
  gtsam::Vector squared(sigmas.size());
  for (size_t i = 0; i < sigmas.size(); ++i) {
    squared(i) = sigmas[i] * sigmas[i];
  }
  return squared.asDiagonal();
}

/**
 * @brief Converts a vector of values to a diagonal matrix (dynamic size).
 * @param diag_elements The input vector of diagonal elements.
 * @return The resulting gtsam::Matrix.
 */
gtsam::Matrix toGtsamDiagonal(const std::vector<double> & diag_elements)
{
  gtsam::Vector v(diag_elements.size());
  for (size_t i = 0; i < diag_elements.size(); ++i) {
    v(i) = diag_elements[i];
  }
  return v.asDiagonal();
}

/**
 * @brief Converts a GTSAM Point3 to a geometry_msgs Point.
 * @param gtsam_obj The input GTSAM Point3.
 * @return The resulting geometry_msgs::msg::Point.
 */
geometry_msgs::msg::Point toPointMsg(const gtsam::Point3 & gtsam_obj)
{
  geometry_msgs::msg::Point msg;
  msg.x = gtsam_obj.x();
  msg.y = gtsam_obj.y();
  msg.z = gtsam_obj.z();
  return msg;
}

/**
 * @brief Converts a GTSAM Vector3 to a geometry_msgs Vector3.
 * @param gtsam_obj The input GTSAM Vector3.
 * @return The resulting geometry_msgs::msg::Vector3.
 */
geometry_msgs::msg::Vector3 toVectorMsg(const gtsam::Vector3 & gtsam_obj)
{
  geometry_msgs::msg::Vector3 msg;
  msg.x = gtsam_obj.x();
  msg.y = gtsam_obj.y();
  msg.z = gtsam_obj.z();
  return msg;
}

/**
 * @brief Converts a GTSAM Rot3 to a geometry_msgs Quaternion.
 * @param gtsam_obj The input GTSAM Rot3.
 * @return The resulting geometry_msgs::msg::Quaternion.
 */
geometry_msgs::msg::Quaternion toQuatMsg(const gtsam::Rot3 & gtsam_obj)
{
  gtsam::Quaternion q = gtsam_obj.toQuaternion();
  geometry_msgs::msg::Quaternion msg;
  msg.w = q.w();
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
  return msg;
}

/**
 * @brief Converts a GTSAM Pose3 to a geometry_msgs Pose.
 * @param gtsam_obj The input GTSAM Pose3.
 * @return The resulting geometry_msgs::msg::Pose.
 */
geometry_msgs::msg::Pose toPoseMsg(const gtsam::Pose3 & gtsam_obj)
{
  geometry_msgs::msg::Pose msg;
  msg.position = toPointMsg(gtsam_obj.translation());
  msg.orientation = toQuatMsg(gtsam_obj.rotation());
  return msg;
}


/**
 * @brief Converts a GTSAM Matrix33 to a 36-element covariance array (upper-left block).
 * @param cov The input GTSAM Matrix33.
 * @return The resulting std::array<double, 36>.
 */
std::array<double, 36> toCovariance36Msg(const gtsam::Matrix33 & cov)
{
  std::array<double, 36> msg;
  msg.fill(0.0);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      msg[i * 6 + j] = cov(i, j);
    }
  }
  return msg;
}

/**
 * @brief Converts a GTSAM Matrix66 to a 36-element covariance array.
 * @param cov The input GTSAM Matrix66.
 * @return The resulting std::array<double, 36>.
 */
std::array<double, 36> toCovariance36Msg(const gtsam::Matrix66 & cov)
{
  std::array<double, 36> msg;
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      msg[i * 6 + j] = cov(i, j);
    }
  }
  return msg;
}

/**
 * @brief Converts a GTSAM Pose3 covariance (rot, pos) to a ROS Pose covariance (pos, rot).
 * @param cov The input GTSAM Matrix66 (rot, pos order).
 * @return The resulting std::array<double, 36> (pos, rot order).
 */
std::array<double, 36> toPoseCovarianceMsg(const gtsam::Matrix66 & cov)
{
  std::array<double, 36> msg;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      msg[i * 6 + j] = cov(i + 3, j + 3);
      msg[(i + 3) * 6 + (j + 3)] = cov(i, j);
      msg[i * 6 + (j + 3)] = cov(i + 3, j);
      msg[(i + 3) * 6 + j] = cov(i, j + 3);
    }
  }
  return msg;
}

}  // namespace coug_fgo::utils
