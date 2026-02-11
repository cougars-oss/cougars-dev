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
 * @file test_dvl_a50_twist_node.cpp
 * @brief Unit tests for dvl_a50_twist_node.hpp.
 * @author Nelson Durrant (w Gemini 3 Pro)
 * @date Jan 2026
 */

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <dvl_msgs/msg/dvl.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

#include "coug_fgo/dvl_a50_twist_node.hpp"

using coug_fgo::DvlA50TwistNode;

/**
 * @class TestDvlA50TwistNode
 * @brief Harness to expose protected DvlA50TwistNode members for testing.
 */
class TestDvlA50TwistNode : public DvlA50TwistNode
{
public:
  using DvlA50TwistNode::DvlA50TwistNode;

  // Expose protected methods
  using DvlA50TwistNode::convertToTwist;

  // Expose protected state
  using DvlA50TwistNode::params_;
};

/**
 * @class DvlA50TwistNodeTest
 * @brief Test fixture for DvlA50TwistNode tests.
 */
class DvlA50TwistNodeTest : public ::testing::Test
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
    node = std::make_shared<TestDvlA50TwistNode>(options);
  }

  std::shared_ptr<TestDvlA50TwistNode> node;
};

/**
 * @brief Verify conversion of DVL data.
 */
TEST_F(DvlA50TwistNodeTest, ConvertToTwist) {
  auto msg = std::make_shared<dvl_msgs::msg::DVL>();
  msg->header.frame_id = "dvl_link";
  msg->velocity_valid = true;
  msg->velocity.x = 1.0;
  msg->velocity.y = 2.0;
  msg->velocity.z = 3.0;
  msg->time_of_validity = 1234567;
  msg->covariance = {0.1, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.3};

  node->params_.use_parameter_frame = false;
  node->params_.use_fom_covariance = false;

  auto twist_msg = node->convertToTwist(msg);

  EXPECT_STREQ(twist_msg.header.frame_id.c_str(), "dvl_link");
  EXPECT_EQ(twist_msg.header.stamp.sec, 1);
  EXPECT_EQ(twist_msg.header.stamp.nanosec, 234567000);

  EXPECT_DOUBLE_EQ(twist_msg.twist.twist.linear.x, 1.0);
  EXPECT_DOUBLE_EQ(twist_msg.twist.twist.linear.y, 2.0);
  EXPECT_DOUBLE_EQ(twist_msg.twist.twist.linear.z, 3.0);

  EXPECT_DOUBLE_EQ(twist_msg.twist.covariance[0], 0.1);
  EXPECT_DOUBLE_EQ(twist_msg.twist.covariance[7], 0.2);
  EXPECT_DOUBLE_EQ(twist_msg.twist.covariance[14], 0.3);

  node->params_.use_fom_covariance = true;
  node->params_.fom_covariance_scale = 2.0;
  msg->fom = 0.5;

  twist_msg = node->convertToTwist(msg);
  EXPECT_DOUBLE_EQ(twist_msg.twist.covariance[0], 1.0);
  EXPECT_DOUBLE_EQ(twist_msg.twist.covariance[7], 1.0);
  EXPECT_DOUBLE_EQ(twist_msg.twist.covariance[14], 1.0);
}
