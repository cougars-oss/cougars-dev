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
 * @file hydrodynamic_factor_arm.hpp
 * @brief GTSAM factor for enforcing Fossen's hydrodynamic equations (F=ma) with lever arm.
 * @author Nelson Durrant
 * @date Jan 2026
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

using gtsam::symbol_shorthand::V;  // Velocity (x,y,z)
using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

namespace coug_fgo::factors
{

/**
 * @class CustomHydrodynamicFactorArm
 * @brief GTSAM factor for enforcing hydrodynamic dynamics between two poses.
 *
 * This factor constrains the velocity evolution of the AUV based on a simplified
 * Fossen model, accounting for thruster inputs and drag.
 *
 * Model: V_next = V_curr + (dt/m) * (F_thrust - (linear_drag * V + quad_drag * |V| * V))
 */
class CustomHydrodynamicFactorArm : public gtsam::NoiseModelFactor4<gtsam::Pose3, gtsam::Vector3,
    gtsam::Pose3, gtsam::Vector3>
{
private:
  double dt_;
  gtsam::Vector3 control_force_;
  gtsam::Pose3 body_P_sensor_;
  double mass_;
  double linear_drag_;
  double quad_drag_;

public:
  /**
   * @brief Constructor for CustomHydrodynamicFactorArm.
   * @param poseKey1 GTSAM key for the first pose (state i).
   * @param velKey1 GTSAM key for the first velocity (state i).
   * @param poseKey2 GTSAM key for the second pose (state j).
   * @param velKey2 GTSAM key for the second velocity (state j).
   * @param dt The time interval between the two states.
   * @param control_force The sensor-frame force vector from thrusters.
   * @param body_P_sensor The transform from sensor frame to body frame.
   * @param mass Combined mass (Rigid body + Added mass).
   * @param linear_drag Linear damping coefficient.
   * @param quad_drag Quadratic damping coefficient.
   * @param model The noise model for the constraint.
   */
  CustomHydrodynamicFactorArm(
    gtsam::Key poseKey1, gtsam::Key velKey1,
    gtsam::Key poseKey2, gtsam::Key velKey2,
    double dt,
    const gtsam::Vector3 & control_force,
    const gtsam::Pose3 & body_P_sensor,
    double mass,
    double linear_drag,
    double quad_drag,
    const gtsam::SharedNoiseModel & model)
  : NoiseModelFactor4<gtsam::Pose3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3>(
      model, poseKey1, velKey1, poseKey2, velKey2),
    dt_(dt),
    control_force_(control_force),
    body_P_sensor_(body_P_sensor),
    mass_(mass),
    linear_drag_(linear_drag),
    quad_drag_(quad_drag)
  {}

  ~CustomHydrodynamicFactorArm() override {}

  /**
   * @brief Helper function to predict the next body velocity based on physics.
   * @param pose1 The current pose estimate.
   * @param vel1 The current world velocity estimate.
   * @return The predicted body-frame velocity at the next time step.
   */
  gtsam::Vector3 predictBodyVelocity(
    const gtsam::Pose3 & pose1,
    const gtsam::Vector3 & vel1) const
  {
    gtsam::Vector3 v_body = pose1.rotation().unrotate(vel1);
    gtsam::Vector3 force_body = body_P_sensor_.rotation() * control_force_;

    gtsam::Vector3 accel_body;
    for (int i = 0; i < 3; i++) {
      double u = v_body(i);
      double drag = -(linear_drag_ * u + quad_drag_ * std::abs(u) * u);
      double net_force = force_body(i) + drag;
      accel_body(i) = net_force / mass_;
    }

    return v_body + accel_body * dt_;
  }

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
   * @return The 3D error vector (difference between measured and predicted body velocity).
   */
  gtsam::Vector evaluateError(
    const gtsam::Pose3 & pose1, const gtsam::Vector3 & vel1,
    const gtsam::Pose3 & pose2, const gtsam::Vector3 & vel2,
    boost::optional<gtsam::Matrix &> H1 = boost::none,
    boost::optional<gtsam::Matrix &> H2 = boost::none,
    boost::optional<gtsam::Matrix &> H3 = boost::none,
    boost::optional<gtsam::Matrix &> H4 = boost::none) const override
  {
    auto errorFunc =
      [this](
      const gtsam::Pose3 & p1, const gtsam::Vector3 & v1,
      const gtsam::Pose3 & p2, const gtsam::Vector3 & v2) -> gtsam::Vector {
        // 3D velocity difference residual
        gtsam::Vector3 v_body_pred = this->predictBodyVelocity(p1, v1);
        gtsam::Vector3 v_body_meas = p2.rotation().unrotate(v2);
        return v_body_meas - v_body_pred;
      };

    if (H1) {
      *H1 = gtsam::numericalDerivative41<gtsam::Vector, gtsam::Pose3, gtsam::Vector3,
          gtsam::Pose3, gtsam::Vector3>(errorFunc, pose1, vel1, pose2, vel2);
    }
    if (H2) {
      *H2 = gtsam::numericalDerivative42<gtsam::Vector, gtsam::Pose3, gtsam::Vector3,
          gtsam::Pose3, gtsam::Vector3>(errorFunc, pose1, vel1, pose2, vel2);
    }
    if (H3) {
      *H3 = gtsam::numericalDerivative43<gtsam::Vector, gtsam::Pose3, gtsam::Vector3,
          gtsam::Pose3, gtsam::Vector3>(errorFunc, pose1, vel1, pose2, vel2);
    }
    if (H4) {
      *H4 = gtsam::numericalDerivative44<gtsam::Vector, gtsam::Pose3, gtsam::Vector3,
          gtsam::Pose3, gtsam::Vector3>(errorFunc, pose1, vel1, pose2, vel2);
    }

    return errorFunc(pose1, vel1, pose2, vel2);
  }
};

}  // namespace coug_fgo::factors
