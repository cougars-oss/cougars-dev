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
 * @file test_mag_factor.cpp
 * @brief Unit tests for mag_factor.hpp.
 * @author Nelson Durrant (w Gemini 3 Pro)
 * @date Jan 2026
 */

#include <gtest/gtest.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>

#include <boost/bind/bind.hpp>

#include "coug_fgo/factors/mag_factor.hpp"

/**
 * @brief Verify error evaluation logic and lever arm correction.
 */
TEST(MagFactorArmTest, ErrorEvaluation) {
  gtsam::Key poseKey = gtsam::symbol_shorthand::X(1);
  gtsam::SharedNoiseModel model = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
  gtsam::Vector3 reference_field_world(1.0, 0.0, 0.0);

  // Case 1: Identity
  coug_fgo::factors::MagFactorArm factor1(poseKey, reference_field_world, reference_field_world,
    gtsam::Rot3::Identity(), model);
  EXPECT_TRUE(
    gtsam::assert_equal(
      gtsam::Vector3::Zero(),
      factor1.evaluateError(gtsam::Pose3::Identity()), 1e-9));

  // Case 2: Rotation
  gtsam::Pose3 pose_90 = gtsam::Pose3(gtsam::Rot3::Yaw(M_PI_2), gtsam::Point3());
  gtsam::Vector3 measured_90 = pose_90.rotation().unrotate(reference_field_world);
  coug_fgo::factors::MagFactorArm factor2(poseKey, measured_90, reference_field_world,
    gtsam::Rot3::Identity(), model);
  EXPECT_TRUE(
    gtsam::assert_equal(
      gtsam::Vector3::Zero(),
      factor2.evaluateError(pose_90), 1e-9));

  // Case 3: Mounting/Lever Arm
  gtsam::Rot3 base_R_sensor = gtsam::Rot3::Yaw(M_PI_2);
  gtsam::Vector3 measured_mount = base_R_sensor.unrotate(reference_field_world);
  coug_fgo::factors::MagFactorArm factor3(poseKey, measured_mount, reference_field_world,
    base_R_sensor, model);
  EXPECT_TRUE(
    gtsam::assert_equal(
      gtsam::Vector3::Zero(),
      factor3.evaluateError(gtsam::Pose3::Identity()), 1e-9));

  // Case 4: Combined
  gtsam::Pose3 pose_comb = gtsam::Pose3(gtsam::Rot3::Yaw(M_PI_4), gtsam::Point3());
  gtsam::Rot3 base_R_sensor_comb = gtsam::Rot3::Yaw(M_PI_4);
  gtsam::Vector3 measured_comb =
    base_R_sensor_comb.unrotate(pose_comb.rotation().unrotate(reference_field_world));
  coug_fgo::factors::MagFactorArm factor4(poseKey, measured_comb, reference_field_world,
    base_R_sensor_comb, model);
  EXPECT_TRUE(
    gtsam::assert_equal(
      gtsam::Vector3::Zero(),
      factor4.evaluateError(pose_comb), 1e-9));

  // Case 5: Error Check
  EXPECT_TRUE(
    gtsam::assert_equal(
      gtsam::Vector3(-2, 0, 0),
      factor1.evaluateError(gtsam::Pose3(gtsam::Rot3::Yaw(M_PI), gtsam::Point3(1, 2, 3))), 1e-9));
}

/**
 * @brief Verify Jacobians against numerical differentiation.
 */
TEST(MagFactorArmTest, Jacobians) {
  gtsam::Key poseKey = gtsam::symbol_shorthand::X(1);
  gtsam::Vector3 reference_field(0.5, 0.8, -0.2);
  gtsam::Vector3 measured_field(0.4, 0.7, -0.1);
  gtsam::Rot3 R_bs = gtsam::Rot3::Rx(0.1);
  gtsam::SharedNoiseModel model = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);

  coug_fgo::factors::MagFactorArm factor(poseKey, measured_field, reference_field, R_bs,
    model);
  gtsam::Pose3 pose = gtsam::Pose3(gtsam::Rot3::Ypr(0.1, -0.2, 0.3), gtsam::Point3(1, 2, 3));

  gtsam::Matrix expectedH = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(
    boost::bind(
      &coug_fgo::factors::MagFactorArm::evaluateError, &factor,
      boost::placeholders::_1, boost::none), pose, 1e-5);

  gtsam::Matrix actualH;
  factor.evaluateError(pose, actualH);
  EXPECT_TRUE(gtsam::assert_equal(expectedH, actualH, 1e-5));
  EXPECT_EQ(actualH.rows(), 3);
  EXPECT_EQ(actualH.cols(), 6);
}
