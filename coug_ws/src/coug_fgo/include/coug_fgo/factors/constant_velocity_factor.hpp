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
 * @file constant_velocity_factor.hpp
 * @brief GTSAM factor for enforcing constant body-frame velocity.
 * @author Nelson Durrant
 * @date Jan 2026
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

using gtsam::symbol_shorthand::V;  // Velocity (x,y,z)
using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

namespace coug_fgo::factors
{

/**
 * @class CustomConstantVelocityFactor
 * @brief GTSAM factor for enforcing constant body velocity between two poses.
 *
 * This factor constrains the body-frame velocity of the AUV to be constant
 * between two consecutive keyframes.
 */
class CustomConstantVelocityFactor : public gtsam::NoiseModelFactor4<gtsam::Pose3, gtsam::Vector3,
    gtsam::Pose3, gtsam::Vector3>
{
public:
  /**
   * @brief Constructor for CustomConstantVelocityFactor.
   * @param pose_key1 GTSAM key for the first pose (state i).
   * @param vel_key1 GTSAM key for the first velocity (state i).
   * @param pose_key2 GTSAM key for the second pose (state j).
   * @param vel_key2 GTSAM key for the second velocity (state j).
   * @param noise_model The noise model for the constraint.
   */
  CustomConstantVelocityFactor(
    gtsam::Key pose_key1, gtsam::Key vel_key1,
    gtsam::Key pose_key2, gtsam::Key vel_key2,
    const gtsam::SharedNoiseModel & noise_model)
  : NoiseModelFactor4<gtsam::Pose3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3>(
      noise_model, pose_key1, vel_key1, pose_key2, vel_key2) {}

  ~CustomConstantVelocityFactor() override {}

  /**
   * @brief Evaluates the error and Jacobians for the factor.
   * @param pose1 The first pose estimate.
   * @param vel1 The first velocity estimate.
   * @param pose2 The second pose estimate.
   * @param vel2 The second velocity estimate.
   * @param H1 Optional Jacobian matrix.
   * @param H2 Optional Jacobian matrix.
   * @param H3 Optional Jacobian matrix.
   * @param H4 Optional Jacobian matrix.
   * @return The 3D error vector.
   */
  gtsam::Vector evaluateError(
    const gtsam::Pose3 & pose_key1, const gtsam::Vector3 & vel_key1,
    const gtsam::Pose3 & pose_key2, const gtsam::Vector3 & vel_key2,
    boost::optional<gtsam::Matrix &> H1 = boost::none,
    boost::optional<gtsam::Matrix &> H2 = boost::none,
    boost::optional<gtsam::Matrix &> H3 = boost::none,
    boost::optional<gtsam::Matrix &> H4 = boost::none) const override
  {
    // Predict the velocity measurements
    gtsam::Matrix33 J_R1_v1, J_v1, J_R2_v2, J_v2;
    gtsam::Vector3 v_body1 = pose_key1.rotation().unrotate(
      vel_key1, H1 ? &J_R1_v1 : 0,
      H2 ? &J_v1 : 0);
    gtsam::Vector3 v_body2 = pose_key2.rotation().unrotate(
      vel_key2, H3 ? &J_R2_v2 : 0,
      H4 ? &J_v2 : 0);

    // 3D velocity difference residual
    gtsam::Vector3 error = v_body1 - v_body2;

    if (H1) {
      // Jacobian with respect to pose1 (3x6)
      H1->setZero(3, 6);
      H1->block<3, 3>(0, 0) = J_R1_v1;
    }
    if (H2) {
      // Jacobian with respect to velocity1 (3x3)
      *H2 = J_v1;
    }
    if (H3) {
      // Jacobian with respect to pose2 (3x6)
      H3->setZero(3, 6);
      H3->block<3, 3>(0, 0) = -J_R2_v2;
    }
    if (H4) {
      // Jacobian with respect to velocity2 (3x3)
      *H4 = -J_v2;
    }

    return error;
  }
};

}  // namespace coug_fgo::factors
