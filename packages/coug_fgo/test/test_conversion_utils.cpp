// Copyright (c) 2026 BYU FRoSt Lab
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

#include "coug_fgo/utils/conversion_utils.hpp"

/**
 * @brief Test basic ROS message to GTSAM conversions.
 */
TEST(ConversionUtilsTest, BasicToGtsam) {
  geometry_msgs::msg::Point ros_point;
  ros_point.x = 1.0; ros_point.y = 2.0; ros_point.z = 3.0;
  gtsam::Point3 gtsam_point = coug_fgo::utils::toGtsam(ros_point);
  EXPECT_DOUBLE_EQ(gtsam_point.x(), 1.0);
  EXPECT_DOUBLE_EQ(gtsam_point.y(), 2.0);
  EXPECT_DOUBLE_EQ(gtsam_point.z(), 3.0);

  geometry_msgs::msg::Vector3 ros_vec;
  ros_vec.x = 0.5; ros_vec.y = -0.5; ros_vec.z = 10.0;
  gtsam::Vector3 gtsam_vec = coug_fgo::utils::toGtsam(ros_vec);
  EXPECT_DOUBLE_EQ(gtsam_vec.x(), 0.5);
  EXPECT_DOUBLE_EQ(gtsam_vec.y(), -0.5);
  EXPECT_DOUBLE_EQ(gtsam_vec.z(), 10.0);

  geometry_msgs::msg::Quaternion ros_quat;
  ros_quat.w = 1.0; ros_quat.x = 0.0; ros_quat.y = 0.0; ros_quat.z = 0.0;
  gtsam::Rot3 gtsam_rot = coug_fgo::utils::toGtsam(ros_quat);
  EXPECT_TRUE(gtsam_rot.equals(gtsam::Rot3::Identity()));

  geometry_msgs::msg::Pose ros_pose;
  ros_pose.position.x = 5.0; ros_pose.position.y = 6.0; ros_pose.position.z = 7.0;
  ros_pose.orientation.w = 1.0;
  gtsam::Pose3 gtsam_pose = coug_fgo::utils::toGtsam(ros_pose);
  EXPECT_DOUBLE_EQ(gtsam_pose.x(), 5.0);
  EXPECT_TRUE(gtsam_pose.rotation().equals(gtsam::Rot3::Identity()));

  geometry_msgs::msg::Transform ros_tf;
  ros_tf.translation.x = 10.0; ros_tf.translation.y = 20.0; ros_tf.translation.z = 30.0;
  ros_tf.rotation.w = 0.70710678; ros_tf.rotation.z = 0.70710678;
  gtsam::Pose3 gtsam_tf = coug_fgo::utils::toGtsam(ros_tf);
  EXPECT_DOUBLE_EQ(gtsam_tf.x(), 10.0);
  EXPECT_NEAR(gtsam_tf.rotation().yaw(), M_PI_2, 1e-4);

  geometry_msgs::msg::Wrench ros_wrench;
  ros_wrench.force.x = 1.0; ros_wrench.force.y = 2.0; ros_wrench.force.z = 3.0;
  ros_wrench.torque.x = 0.1; ros_wrench.torque.y = 0.2; ros_wrench.torque.z = 0.3;
  gtsam::Vector6 gtsam_wrench = coug_fgo::utils::toGtsam(ros_wrench);
  EXPECT_DOUBLE_EQ(gtsam_wrench(0), 1.0);
  EXPECT_DOUBLE_EQ(gtsam_wrench(3), 0.1);
}

/**
 * @brief Test ROS covariance array to GTSAM Matrix conversions.
 */
TEST(ConversionUtilsTest, CovarianceToGtsam) {
  std::array<double, 9> cov3x3;
  for (int i = 0; i < 9; ++i) {
    cov3x3[i] = static_cast<double>(i);
  }
  gtsam::Matrix33 m3 = coug_fgo::utils::toGtsam(cov3x3);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      EXPECT_DOUBLE_EQ(m3(i, j), static_cast<double>(i * 3 + j));
    }
  }

  std::array<double, 36> cov6x6;
  for (int i = 0; i < 36; ++i) {
    cov6x6[i] = static_cast<double>(i);
  }
  gtsam::Matrix66 m6 = coug_fgo::utils::toGtsam(cov6x6);
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      EXPECT_DOUBLE_EQ(m6(i, j), static_cast<double>(i * 6 + j));
    }
  }
}

/**
 * @brief Test extraction of sub-matrices and scalar conversions.
 */
TEST(ConversionUtilsTest, GtsamExtraction) {
  std::array<double, 36> cov6x6;
  for (int i = 0; i < 36; ++i) {
    cov6x6[i] = static_cast<double>(i);
  }

  gtsam::Matrix33 m3_ext = coug_fgo::utils::toGtsam3x3(cov6x6);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      EXPECT_DOUBLE_EQ(m3_ext(i, j), static_cast<double>(i * 6 + j));
    }
  }

  gtsam::Matrix33 m3_diag = coug_fgo::utils::toGtsam3x3Diagonal(cov6x6);
  EXPECT_DOUBLE_EQ(m3_diag(0, 0), cov6x6[0]);
  EXPECT_DOUBLE_EQ(m3_diag(1, 1), cov6x6[7]);
  EXPECT_DOUBLE_EQ(m3_diag(2, 2), cov6x6[14]);
  EXPECT_DOUBLE_EQ(m3_diag(0, 1), 0.0);

  std::array<double, 9> cov3x3_val;
  cov3x3_val.fill(0.0); cov3x3_val[8] = 42.0;
  gtsam::Matrix11 m11_yaw = coug_fgo::utils::toGtsamYawCovariance(cov3x3_val);
  EXPECT_DOUBLE_EQ(m11_yaw(0, 0), 42.0);

  std::array<double, 36> cov6x6_val;
  cov6x6_val.fill(0.0); cov6x6_val[14] = 84.0;
  gtsam::Matrix11 m11_depth = coug_fgo::utils::toGtsamDepthCovariance(cov6x6_val);
  EXPECT_DOUBLE_EQ(m11_depth(0, 0), 84.0);
}

/**
 * @brief Test basic GTSAM to ROS message conversions.
 */
TEST(ConversionUtilsTest, BasicToRos) {
  gtsam::Point3 gtsam_point(1.0, 2.0, 3.0);
  geometry_msgs::msg::Point ros_point = coug_fgo::utils::toPointMsg(gtsam_point);
  EXPECT_DOUBLE_EQ(ros_point.x, 1.0);

  gtsam::Vector3 gtsam_vec(0.5, -0.5, 10.0);
  geometry_msgs::msg::Vector3 ros_vec = coug_fgo::utils::toVectorMsg(gtsam_vec);
  EXPECT_DOUBLE_EQ(ros_vec.x, 0.5);

  gtsam::Rot3 gtsam_rot = gtsam::Rot3::Identity();
  geometry_msgs::msg::Quaternion ros_quat = coug_fgo::utils::toQuatMsg(gtsam_rot);
  EXPECT_DOUBLE_EQ(ros_quat.w, 1.0);

  gtsam::Pose3 gtsam_pose(gtsam::Rot3::Identity(), gtsam::Point3(5.0, 6.0, 7.0));
  geometry_msgs::msg::Pose ros_pose = coug_fgo::utils::toPoseMsg(gtsam_pose);
  EXPECT_DOUBLE_EQ(ros_pose.position.x, 5.0);

  geometry_msgs::msg::Transform ros_tf = coug_fgo::utils::toTransformMsg(gtsam_pose);
  EXPECT_DOUBLE_EQ(ros_tf.translation.x, 5.0);
  EXPECT_DOUBLE_EQ(ros_tf.rotation.w, 1.0);

  gtsam::Vector6 v_wrench; v_wrench << 1, 2, 3, 4, 5, 6;
  geometry_msgs::msg::Wrench ros_wrench = coug_fgo::utils::toWrenchMsg(v_wrench);
  EXPECT_DOUBLE_EQ(ros_wrench.force.x, 1.0);
  EXPECT_DOUBLE_EQ(ros_wrench.torque.z, 6.0);
}

/**
 * @brief Test GTSAM matrix to ROS covariance array conversions.
 */
TEST(ConversionUtilsTest, MatrixToRos) {
  gtsam::Matrix33 m3;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      m3(i, j) = static_cast<double>(i * 3 + j);
    }
  }

  std::array<double, 9> ros_cov9 = coug_fgo::utils::toCovariance9Msg(m3);
  for (int i = 0; i < 9; ++i) {
    EXPECT_DOUBLE_EQ(ros_cov9[i], static_cast<double>(i));
  }

  std::array<double, 36> ros_cov36_m3 = coug_fgo::utils::toCovariance36Msg(m3);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      EXPECT_DOUBLE_EQ(ros_cov36_m3[i * 6 + j], static_cast<double>(i * 3 + j));
    }
  }

  gtsam::Matrix66 m6;
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      m6(i, j) = static_cast<double>(i * 6 + j);
    }
  }

  std::array<double, 36> ros_cov36_m6 = coug_fgo::utils::toCovariance36Msg(m6);
  for (int i = 0; i < 36; ++i) {
    EXPECT_DOUBLE_EQ(ros_cov36_m6[i], static_cast<double>(i));
  }

  gtsam::Matrix66 pos_rot_cov = gtsam::Matrix66::Zero();
  pos_rot_cov(0, 0) = 1.0;
  pos_rot_cov(3, 3) = 2.0;
  pos_rot_cov(3, 0) = 0.5;
  pos_rot_cov(0, 3) = 0.5;

  std::array<double, 36> ros_pose_cov = coug_fgo::utils::toPoseCovarianceMsg(pos_rot_cov);
  EXPECT_DOUBLE_EQ(ros_pose_cov[0], 2.0);
  EXPECT_DOUBLE_EQ(ros_pose_cov[21], 1.0);
  EXPECT_DOUBLE_EQ(ros_pose_cov[3], 0.5);
  EXPECT_DOUBLE_EQ(ros_pose_cov[18], 0.5);
}
