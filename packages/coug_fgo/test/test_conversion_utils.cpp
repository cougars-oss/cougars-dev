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
 * @file test_conversion_utils.cpp
 * @brief Unit tests for conversion_utils.hpp.
 * @author Nelson Durrant (w Gemini 3 Pro)
 * @date Jan 2026
 */

#include <gtest/gtest.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Pose3.h>

#include "coug_fgo/utils/conversion_utils.hpp"

/**
 * @brief Verify basic geometry conversions from ROS messages to GTSAM types.
 */
TEST(ConversionUtilsTest, RosToGtsam_Geometry) {
  geometry_msgs::msg::Point pt_msg;
  pt_msg.x = 1.0; pt_msg.y = 2.0; pt_msg.z = 3.0;
  gtsam::Point3 pt = coug_fgo::utils::toGtsam(pt_msg);
  EXPECT_DOUBLE_EQ(pt.x(), 1.0);
  EXPECT_DOUBLE_EQ(pt.y(), 2.0);
  EXPECT_DOUBLE_EQ(pt.z(), 3.0);

  geometry_msgs::msg::Vector3 vec_msg;
  vec_msg.x = 4.0; vec_msg.y = 5.0; vec_msg.z = 6.0;
  gtsam::Vector3 vec = coug_fgo::utils::toGtsam(vec_msg);
  EXPECT_DOUBLE_EQ(vec.x(), 4.0);
  EXPECT_DOUBLE_EQ(vec.y(), 5.0);
  EXPECT_DOUBLE_EQ(vec.z(), 6.0);

  geometry_msgs::msg::Quaternion quat_msg;
  quat_msg.w = 1.0; quat_msg.x = 0.0; quat_msg.y = 0.0; quat_msg.z = 0.0;
  gtsam::Rot3 rot = coug_fgo::utils::toGtsam(quat_msg);
  EXPECT_TRUE(rot.equals(gtsam::Rot3::Identity()));

  geometry_msgs::msg::Pose pose_msg;
  pose_msg.position = pt_msg;
  pose_msg.orientation = quat_msg;
  gtsam::Pose3 pose = coug_fgo::utils::toGtsam(pose_msg);
  EXPECT_DOUBLE_EQ(pose.x(), 1.0);
  EXPECT_TRUE(pose.rotation().equals(gtsam::Rot3::Identity()));

  geometry_msgs::msg::Transform tf_msg;
  tf_msg.translation = vec_msg;
  tf_msg.rotation = quat_msg;
  gtsam::Pose3 tf_pose = coug_fgo::utils::toGtsam(tf_msg);
  EXPECT_DOUBLE_EQ(tf_pose.x(), 4.0);

  geometry_msgs::msg::Wrench wrench_msg;
  wrench_msg.force = vec_msg;
  wrench_msg.torque = vec_msg;
  gtsam::Vector6 wrench = coug_fgo::utils::toGtsam(wrench_msg);
  EXPECT_DOUBLE_EQ(wrench(0), 4.0);
  EXPECT_DOUBLE_EQ(wrench(3), 4.0);
}

/**
 * @brief Verify linear algebra conversions from STL containers to GTSAM types.
 */
TEST(ConversionUtilsTest, RosToGtsam_Matrices) {
  std::array<double, 9> cov9;
  cov9.fill(0.0); cov9[0] = 1.0; cov9[4] = 5.0; cov9[8] = 9.0;
  gtsam::Matrix33 m33 = coug_fgo::utils::toGtsam(cov9);
  EXPECT_DOUBLE_EQ(m33(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(m33(1, 1), 5.0);
  EXPECT_DOUBLE_EQ(m33(2, 2), 9.0);

  std::array<double, 36> cov36;
  cov36.fill(0.0); cov36[0] = 10.0; cov36[35] = 20.0;
  gtsam::Matrix66 m66 = coug_fgo::utils::toGtsam(cov36);
  EXPECT_DOUBLE_EQ(m66(0, 0), 10.0);
  EXPECT_DOUBLE_EQ(m66(5, 5), 20.0);

  std::vector<double> v = {1.0, 2.0, 3.0};
  gtsam::Vector gtsam_v = coug_fgo::utils::toGtsam(v);
  EXPECT_EQ(gtsam_v.size(), 3);
  EXPECT_DOUBLE_EQ(gtsam_v(2), 3.0);
}

/**
 * @brief Verify helper functions for matrix extraction and creation.
 */
TEST(ConversionUtilsTest, Helpers_MatrixLogic) {
  std::array<double, 36> cov36;
  cov36.fill(0.0);
  cov36[0] = 1.0;
  cov36[1] = 0.5;
  cov36[7] = 2.0;
  cov36[14] = 3.0;

  gtsam::Matrix33 m33_full = coug_fgo::utils::toGtsam3x3(cov36);
  EXPECT_DOUBLE_EQ(m33_full(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(m33_full(0, 1), 0.5);
  EXPECT_DOUBLE_EQ(m33_full(1, 1), 2.0);

  std::vector<double> sigmas = {2.0, 3.0};
  gtsam::Matrix mat_sq = coug_fgo::utils::toGtsamSquaredDiagonal(sigmas);
  EXPECT_EQ(mat_sq.rows(), 2);
  EXPECT_EQ(mat_sq.cols(), 2);
  EXPECT_DOUBLE_EQ(mat_sq(0, 0), 4.0);
  EXPECT_DOUBLE_EQ(mat_sq(1, 1), 9.0);

  std::vector<double> diag = {2.0, 3.0};
  gtsam::Matrix mat_diag = coug_fgo::utils::toGtsamDiagonal(diag);
  EXPECT_EQ(mat_diag.rows(), 2);
  EXPECT_EQ(mat_diag.cols(), 2);
  EXPECT_DOUBLE_EQ(mat_diag(0, 0), 2.0);
  EXPECT_DOUBLE_EQ(mat_diag(1, 1), 3.0);
}

/**
 * @brief Verify conversions from GTSAM types back to ROS messages.
 */
TEST(ConversionUtilsTest, GtsamToRos_Geometry) {
  gtsam::Point3 pt(1.0, 2.0, 3.0);
  gtsam::Vector3 vec(4.0, 5.0, 6.0);
  gtsam::Rot3 rot = gtsam::Rot3::Identity();
  gtsam::Pose3 pose(rot, pt);

  geometry_msgs::msg::Point pt_msg = coug_fgo::utils::toPointMsg(pt);
  EXPECT_DOUBLE_EQ(pt_msg.x, 1.0);

  geometry_msgs::msg::Vector3 vec_msg = coug_fgo::utils::toVectorMsg(vec);
  EXPECT_DOUBLE_EQ(vec_msg.x, 4.0);

  geometry_msgs::msg::Quaternion quat_msg = coug_fgo::utils::toQuatMsg(rot);
  EXPECT_DOUBLE_EQ(quat_msg.w, 1.0);

  geometry_msgs::msg::Pose ros_pose = coug_fgo::utils::toPoseMsg(pose);
  EXPECT_DOUBLE_EQ(ros_pose.position.x, 1.0);
}

/**
 * @brief Verify GTSAM to ROS covariance conversions.
 */
TEST(ConversionUtilsTest, GtsamToRos_Covariance) {
  gtsam::Matrix33 m33 = gtsam::Matrix33::Identity() * 5.0;
  std::array<double, 36> arr36_padded = coug_fgo::utils::toCovariance36Msg(m33);
  EXPECT_DOUBLE_EQ(arr36_padded[0], 5.0);
  EXPECT_DOUBLE_EQ(arr36_padded[7], 5.0);
  EXPECT_DOUBLE_EQ(arr36_padded[35], 0.0);

  gtsam::Matrix66 m66 = gtsam::Matrix66::Identity() * 2.0;
  std::array<double, 36> arr36_copy = coug_fgo::utils::toCovariance36Msg(m66);
  EXPECT_DOUBLE_EQ(arr36_copy[0], 2.0);
  EXPECT_DOUBLE_EQ(arr36_copy[35], 2.0);

  gtsam::Matrix66 m_pose = gtsam::Matrix66::Zero();
  m_pose(0, 0) = 99.0;
  m_pose(3, 3) = 77.0;

  std::array<double, 36> arr36_swapped = coug_fgo::utils::toPoseCovarianceMsg(m_pose);

  EXPECT_DOUBLE_EQ(arr36_swapped[0], 77.0);
  EXPECT_DOUBLE_EQ(arr36_swapped[21], 99.0);
}
