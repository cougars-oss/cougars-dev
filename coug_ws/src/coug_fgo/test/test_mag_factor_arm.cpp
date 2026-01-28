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
 * @file test_mag_factor_arm.cpp
 * @brief Unit tests for mag_factor_arm.hpp.
 * @author Nelson Durrant (w Gemini 3 Pro)
 * @date Jan 2026
 */

#include <gtest/gtest.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>

#include <boost/bind/bind.hpp>

#include "coug_fgo/factors/mag_factor_arm.hpp"

/**
 * @brief Test the error evaluation logic of the CustomMagFactorArm.
 *
 * Verifies that the factor correctly accounts for the mounting rotation (offset)
 * between the vehicle's body frame and the magnetometer frame.
 *
 * Cases tested:
 * 1.  **Identity**: Everything aligned. Zero error.
 * 2.  **Rotation**: Vehicle rotated, sensor aligned.
 * 3.  **Mounting/Lever Arm**: Sensor offset relative to body.
 * 4.  **Combined**: Vehicle rotated + Sensor offset.
 * 5.  **Error Check**: Verifies non-zero error magnitude.
 */
TEST(CustomMagFactorArmTest, ErrorEvaluation) {
  gtsam::Key poseKey = gtsam::symbol_shorthand::X(1);
  gtsam::SharedNoiseModel model = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
  gtsam::Vector3 reference_field(1.0, 0.0, 0.0);

  // Case 1: Identity
  coug_fgo::factors::CustomMagFactorArm factor1(poseKey, reference_field,
    reference_field, gtsam::Rot3::Identity(), model);
  EXPECT_TRUE(
    gtsam::assert_equal(
      gtsam::Vector3::Zero(),
      factor1.evaluateError(gtsam::Pose3::Identity()), 1e-9));

  // Case 2: Rotation
  gtsam::Pose3 pose_90 = gtsam::Pose3(gtsam::Rot3::Yaw(M_PI_2), gtsam::Point3());
  gtsam::Vector3 b_body_90 = pose_90.rotation().unrotate(reference_field);
  coug_fgo::factors::CustomMagFactorArm factor2(poseKey, b_body_90,
    reference_field, gtsam::Rot3::Identity(), model);
  EXPECT_TRUE(
    gtsam::assert_equal(
      gtsam::Vector3::Zero(),
      factor2.evaluateError(pose_90), 1e-9));

  // Case 3: Mounting/Lever Arm
  gtsam::Vector3 measured_rot_mount(0.0, -1.0, 0.0);
  coug_fgo::factors::CustomMagFactorArm factor3(poseKey, measured_rot_mount,
    reference_field, gtsam::Rot3::Yaw(M_PI_2), model);
  EXPECT_TRUE(
    gtsam::assert_equal(
      gtsam::Vector3::Zero(),
      factor3.evaluateError(gtsam::Pose3::Identity()), 1e-9));

  // Case 4: Combined
  coug_fgo::factors::CustomMagFactorArm factor4(poseKey, gtsam::Vector3(-1.0, 0.0, 0.0),
    reference_field, gtsam::Rot3::Yaw(M_PI_2), model);
  EXPECT_TRUE(
    gtsam::assert_equal(
      gtsam::Vector3::Zero(),
      factor4.evaluateError(gtsam::Pose3(gtsam::Rot3::Yaw(M_PI_2), gtsam::Point3())), 1e-9));

  // Case 5: Error Check
  gtsam::Pose3 pose_small_rot = gtsam::Pose3(gtsam::Rot3::Yaw(0.1), gtsam::Point3());
  gtsam::Vector error_small = factor1.evaluateError(pose_small_rot);
  EXPECT_NEAR(error_small[1], -sin(0.1), 1e-5);
}

/**
 * @brief Test the error evaluation logic of the CustomMagFactorArm (Yaw Only).
 *
 * Verifies that the factor correctly accounts for yaw differences, wrapping angles appropriately.
 *
 * Cases tested:
 * 1.  **Identity**: Everything aligned. Zero error.
 * 2.  **Rotation**: Vehicle rotated, sensor aligned.
 * 3.  **Mounting/Lever Arm**: Sensor offset relative to body.
 * 4.  **Combined**: Vehicle rotated + Sensor offset.
 * 5.  **Error Check**: Verifies non-zero error magnitude.
 * 6.  **Ignore Dip/Mag**: Measurement matches Yaw but differs in Dip/Mag.
 */
TEST(CustomMagFactorArmTest, ErrorEvaluationYawOnly) {
  gtsam::Key poseKey = gtsam::symbol_shorthand::X(1);
  gtsam::SharedNoiseModel model = gtsam::noiseModel::Isotropic::Sigma(1, 0.1);
  gtsam::Vector3 ref(1.0, 0.0, 0.0);

  // Case 1: Identity
  coug_fgo::factors::CustomMagFactorArm factor1(poseKey, ref, ref,
    gtsam::Rot3::Identity(), model, true);
  EXPECT_TRUE(
    gtsam::assert_equal(
      gtsam::Vector1::Zero(),
      factor1.evaluateError(gtsam::Pose3::Identity()), 1e-9));

  // Case 2: Rotation
  gtsam::Pose3 pose_90 = gtsam::Pose3(gtsam::Rot3::Yaw(M_PI_2), gtsam::Point3());
  gtsam::Vector3 b_body_90 = pose_90.rotation().unrotate(ref);
  coug_fgo::factors::CustomMagFactorArm factor2(poseKey, b_body_90, ref,
    gtsam::Rot3::Identity(), model, true);
  EXPECT_TRUE(
    gtsam::assert_equal(
      gtsam::Vector1::Zero(),
      factor2.evaluateError(pose_90), 1e-9));

  // Case 3: Mounting/Lever Arm
  gtsam::Vector3 meas_mount(0.0, -1.0, 0.0);
  coug_fgo::factors::CustomMagFactorArm factor3(poseKey, meas_mount, ref,
    gtsam::Rot3::Yaw(M_PI_2), model, true);
  EXPECT_TRUE(
    gtsam::assert_equal(
      gtsam::Vector1::Zero(),
      factor3.evaluateError(gtsam::Pose3::Identity()), 1e-9));

  // Case 4: Combined
  coug_fgo::factors::CustomMagFactorArm factor4(poseKey, gtsam::Vector3(-1.0, 0.0, 0.0),
    ref, gtsam::Rot3::Yaw(M_PI_2), model, true);
  EXPECT_TRUE(
    gtsam::assert_equal(
      gtsam::Vector1::Zero(),
      factor4.evaluateError(gtsam::Pose3(gtsam::Rot3::Yaw(M_PI_2), gtsam::Point3())), 1e-9));

  // Case 5: Error Check
  gtsam::Pose3 pose_small = gtsam::Pose3(gtsam::Rot3::Yaw(0.1), gtsam::Point3());
  gtsam::Vector error_small = factor1.evaluateError(pose_small);
  EXPECT_NEAR(error_small[0], -0.1, 1e-5);

  // Case 6: Ignore Dip/Mag
  gtsam::Vector3 meas_mag_dip(2.0, 0.0, 1.0);
  coug_fgo::factors::CustomMagFactorArm factor6(poseKey, meas_mag_dip, ref,
    gtsam::Rot3::Identity(), model, true);
  EXPECT_TRUE(
    gtsam::assert_equal(
      gtsam::Vector1::Zero(),
      factor6.evaluateError(gtsam::Pose3::Identity()), 1e-9));
}

/**
 * @brief Verify Jacobians of the CustomMagFactorArm using numerical differentiation.
 *
 * Validates analytical derivatives for magnetic field error, ensuring proper handling of SE(3) manifolds.
 */
TEST(CustomMagFactorArmTest, Jacobians) {
  gtsam::Key poseKey = gtsam::symbol_shorthand::X(1);
  gtsam::Vector3 reference_field(0.5, 0.8, -0.2);
  gtsam::Vector3 measured_field(0.4, 0.7, -0.1);
  gtsam::Rot3 R_bs = gtsam::Rot3::Rx(0.1);
  gtsam::SharedNoiseModel model = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);

  coug_fgo::factors::CustomMagFactorArm factor(poseKey, measured_field, reference_field, R_bs,
    model);
  gtsam::Pose3 pose = gtsam::Pose3(gtsam::Rot3::Ypr(0.1, -0.2, 0.3), gtsam::Point3(1, 2, 3));

  gtsam::Matrix expectedH = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(
    boost::bind(
      &coug_fgo::factors::CustomMagFactorArm::evaluateError, &factor,
      boost::placeholders::_1, boost::none), pose, 1e-5);

  gtsam::Matrix actualH;
  factor.evaluateError(pose, actualH);
  EXPECT_TRUE(gtsam::assert_equal(expectedH, actualH, 1e-5));
}

/**
 * @brief Verify Jacobians of the CustomMagFactorArm (Yaw Only) using numerical differentiation.
 *
 * Validates analytical derivatives for yaw error, ensuring proper handling of SE(3) manifolds.
 */
TEST(CustomMagFactorArmTest, JacobiansYawOnly) {
  gtsam::Key poseKey = gtsam::symbol_shorthand::X(1);
  gtsam::Vector3 reference_field(0.5, 0.8, -0.2);
  gtsam::Vector3 measured_field(0.4, 0.7, -0.1);
  gtsam::Rot3 R_bs = gtsam::Rot3::Rx(0.1);
  gtsam::SharedNoiseModel model = gtsam::noiseModel::Isotropic::Sigma(1, 0.1);

  coug_fgo::factors::CustomMagFactorArm factor(poseKey, measured_field, reference_field, R_bs,
    model, true);
  gtsam::Pose3 pose = gtsam::Pose3(gtsam::Rot3::Ypr(0.1, -0.2, 0.3), gtsam::Point3(1, 2, 3));

  gtsam::Matrix expectedH = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(
    boost::bind(
      &coug_fgo::factors::CustomMagFactorArm::evaluateError, &factor,
      boost::placeholders::_1, boost::none), pose, 1e-5);

  gtsam::Matrix actualH;
  factor.evaluateError(pose, actualH);
  EXPECT_TRUE(gtsam::assert_equal(expectedH, actualH, 1e-5));
}
