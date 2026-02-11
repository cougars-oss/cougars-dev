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
 * @file test_factor_graph_node.cpp
 * @brief Unit tests for factor_graph_node.hpp.
 * @author Nelson Durrant (w Gemini 3 Pro)
 * @date Jan 2026
 */

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "coug_fgo/factor_graph_node.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"

using coug_fgo::FactorGraphNode;

/**
 * @class TestFactorGraphNode
 * @brief Harness to expose protected FactorGraphNode members for testing.
 */
class TestFactorGraphNode : public FactorGraphNode
{
public:
  using FactorGraphNode::FactorGraphNode;

  // Expose protected methods
  using FactorGraphNode::incrementAverages;
  using FactorGraphNode::computeInitialOrientation;
  using FactorGraphNode::computeInitialPosition;
  using FactorGraphNode::computeInitialVelocity;
  using FactorGraphNode::computeInitialBias;
  using FactorGraphNode::getInterpolatedOrientation;

  // Expose protected state
  using FactorGraphNode::initial_ahrs_count_;
  using FactorGraphNode::initial_imu_;
  using FactorGraphNode::initial_ahrs_;
  using FactorGraphNode::initial_dvl_;
  using FactorGraphNode::initial_gps_;
  using FactorGraphNode::initial_depth_;
  using FactorGraphNode::initial_mag_;

  using FactorGraphNode::ahrs_queue_;

  using FactorGraphNode::params_;
  using FactorGraphNode::dvl_to_base_tf_;
  using FactorGraphNode::imu_to_dvl_tf_;
  using FactorGraphNode::gps_to_dvl_tf_;
  using FactorGraphNode::depth_to_dvl_tf_;
  using FactorGraphNode::mag_to_dvl_tf_;
};

/**
 * @class FactorGraphNodeTest
 * @brief Test fixture for FactorGraphNode tests.
 */
class FactorGraphNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void SetUp() override
  {
    rclcpp::NodeOptions options;
    node = std::make_shared<TestFactorGraphNode>(options);
  }

  std::shared_ptr<TestFactorGraphNode> node;

  sensor_msgs::msg::Imu::SharedPtr createImuMsg(double t, double r, double p, double y)
  {
    auto msg = std::make_shared<sensor_msgs::msg::Imu>();
    msg->header.stamp = rclcpp::Time(static_cast<uint64_t>(t * 1e9));
    tf2::Quaternion q;
    q.setRPY(r, p, y);
    gtsam::Rot3 gtsam_q(q.w(), q.x(), q.y(), q.z());
    msg->orientation = coug_fgo::utils::toQuatMsg(gtsam_q);
    msg->linear_acceleration.z = 9.81;
    return msg;
  }
};

/**
 * @brief Verify orientation wrapping and average accumulation.
 */
TEST_F(FactorGraphNodeTest, IncrementAverages) {
  node->params_.ahrs.enable_ahrs = true;
  node->initial_ahrs_count_ = 0;

  double deg2rad = M_PI / 180.0;
  node->ahrs_queue_.push(createImuMsg(1.0, 0.0, 0.0, 350.0 * deg2rad));
  node->ahrs_queue_.push(createImuMsg(1.1, 0.0, 0.0, 10.0 * deg2rad));

  node->incrementAverages();

  double avg_yaw = coug_fgo::utils::toGtsam(node->initial_ahrs_->orientation).yaw();
  EXPECT_NEAR(std::abs(avg_yaw), 0.0, 1e-4);
}

/**
 * @brief Verify gravity alignment and magnetometer logic.
 */
TEST_F(FactorGraphNodeTest, ComputeInitialOrientation) {
  // Case 1: Tilted Gravity
  auto imu = std::make_shared<sensor_msgs::msg::Imu>();
  double g = 9.81;
  imu->linear_acceleration.x = -g * sin(M_PI_4);
  imu->linear_acceleration.z = g * cos(M_PI_4);
  node->initial_imu_ = imu;
  node->dvl_to_base_tf_.transform.rotation.w = 1.0;
  node->imu_to_dvl_tf_.transform.rotation.w = 1.0;
  node->params_.prior.use_parameter_priors = false;

  gtsam::Rot3 R_tilt = node->computeInitialOrientation();
  EXPECT_NEAR(R_tilt.pitch(), M_PI_4, 1e-6);

  // Case 2: Magnetometer with Mounting Rotation
  node->params_.ahrs.enable_ahrs = false;
  node->params_.mag.enable_mag = true;
  node->params_.mag.reference_field = {1.0, 0.0, 0.0};

  tf2::Quaternion q_mag;
  q_mag.setRPY(0, 0, M_PI_2);
  node->mag_to_dvl_tf_.transform.rotation =
    coug_fgo::utils::toQuatMsg(gtsam::Rot3(q_mag.w(), q_mag.x(), q_mag.y(), q_mag.z()));

  auto mag = std::make_shared<sensor_msgs::msg::MagneticField>();
  mag->magnetic_field.x = 1.0;
  node->initial_mag_ = mag;
  node->initial_imu_ = createImuMsg(1.0, 0.0, 0.0, 0.0);

  gtsam::Rot3 R_mag = node->computeInitialOrientation();
  EXPECT_NEAR(R_mag.yaw(), -M_PI_2, 1e-6);
}

/**
 * @brief Verify lever arm rotation and depth override.
 */
TEST_F(FactorGraphNodeTest, ComputeInitialPosition) {
  node->params_.prior.use_parameter_priors = false;
  node->params_.gps.enable_gps = true;

  auto gps = std::make_shared<nav_msgs::msg::Odometry>();
  gps->pose.pose.position.x = 10.0;
  gps->pose.pose.position.y = 10.0;
  gps->pose.pose.position.z = 100.0;
  node->initial_gps_ = gps;

  auto depth = std::make_shared<nav_msgs::msg::Odometry>();
  depth->pose.pose.position.z = -5.0;
  node->initial_depth_ = depth;

  node->gps_to_dvl_tf_.transform.translation.x = 2.0;
  node->depth_to_dvl_tf_.transform.translation.z = -1.0;

  gtsam::Point3 P = node->computeInitialPosition(gtsam::Rot3::Yaw(M_PI_2));

  EXPECT_NEAR(P.x(), 10.0, 1e-6);
  EXPECT_NEAR(P.y(), 8.0, 1e-6);
  EXPECT_NEAR(P.z(), -4.0, 1e-6);
}

/**
 * @brief Verify velocity frame rotation.
 */
TEST_F(FactorGraphNodeTest, ComputeInitialVelocity) {
  node->params_.prior.use_parameter_priors = false;

  auto dvl = std::make_shared<geometry_msgs::msg::TwistWithCovarianceStamped>();
  dvl->twist.twist.linear.x = 1.0;
  node->initial_dvl_ = dvl;

  gtsam::Vector3 V = node->computeInitialVelocity(gtsam::Rot3::Yaw(M_PI_2));
  EXPECT_NEAR(V.x(), 0.0, 1e-6);
  EXPECT_NEAR(V.y(), 1.0, 1e-6);
}

/**
 * @brief Verify bias initialization and parameter priority.
 */
TEST_F(FactorGraphNodeTest, ComputeInitialBias) {
  node->params_.prior.use_parameter_priors = true;
  node->params_.prior.parameter_priors.initial_gyro_bias = {0.5, 0.5, 0.5};

  auto imu = std::make_shared<sensor_msgs::msg::Imu>();
  imu->angular_velocity.x = 0.01;
  node->initial_imu_ = imu;

  gtsam::imuBias::ConstantBias bias = node->computeInitialBias();
  EXPECT_NEAR(bias.gyroscope().x(), 0.5, 1e-6);
}

/**
 * @brief Verify orientation interpolation.
 */
TEST_F(FactorGraphNodeTest, GetInterpolatedOrientation) {
  std::deque<sensor_msgs::msg::Imu::SharedPtr> msgs;
  msgs.push_back(createImuMsg(1.0, 0.0, 0.0, 0.0));
  msgs.push_back(createImuMsg(2.0, 0.0, 0.0, 1.0));

  gtsam::Rot3 result = node->getInterpolatedOrientation(msgs, 1.5);
  EXPECT_NEAR(result.yaw(), 0.5, 1e-6);
}
