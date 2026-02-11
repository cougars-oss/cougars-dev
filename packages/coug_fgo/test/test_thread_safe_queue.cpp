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
 * @file test_thread_safe_queue.cpp
 * @brief Unit tests for thread_safe_queue.hpp.
 * @author Nelson Durrant (w Gemini 3 Pro)
 * @date Jan 2026
 */

#include <gtest/gtest.h>
#include <thread>
#include <vector>

#include "coug_fgo/utils/thread_safe_queue.hpp"
#include "sensor_msgs/msg/imu.hpp"

using coug_fgo::utils::ThreadSafeQueue;

/**
 * @class ThreadSafeQueueTest
 * @brief Test fixture for ThreadSafeQueue tests.
 */
class ThreadSafeQueueTest : public ::testing::Test
{
protected:
  ThreadSafeQueue<sensor_msgs::msg::Imu::SharedPtr> queue;

  sensor_msgs::msg::Imu::SharedPtr createMsg(double t)
  {
    auto msg = std::make_shared<sensor_msgs::msg::Imu>();
    msg->header.stamp.sec = static_cast<int32_t>(t);
    msg->header.stamp.nanosec = static_cast<uint32_t>((t - floor(t)) * 1e9);
    return msg;
  }
};

/**
 * @brief Verify initialization state.
 */
TEST_F(ThreadSafeQueueTest, Initialization) {
  EXPECT_TRUE(queue.empty());
  EXPECT_EQ(queue.size(), 0u);
  EXPECT_DOUBLE_EQ(queue.getLastTime(), 0.0);
}

/**
 * @brief Verify push and size functionality.
 */
TEST_F(ThreadSafeQueueTest, PushAndSize) {
  queue.push(createMsg(1.0));
  EXPECT_FALSE(queue.empty());
  EXPECT_EQ(queue.size(), 1u);
  EXPECT_DOUBLE_EQ(queue.getLastTime(), 1.0);

  queue.push(createMsg(2.5));
  EXPECT_EQ(queue.size(), 2u);
  EXPECT_DOUBLE_EQ(queue.getLastTime(), 2.5);
}

/**
 * @brief Verify drain functionality.
 */
TEST_F(ThreadSafeQueueTest, Drain) {
  queue.push(createMsg(1.0));
  queue.push(createMsg(2.0));
  queue.push(createMsg(3.0));

  auto drained = queue.drain();
  EXPECT_EQ(drained.size(), 3u);
  EXPECT_TRUE(queue.empty());
  EXPECT_EQ(queue.size(), 0u);

  EXPECT_DOUBLE_EQ(rclcpp::Time(drained[0]->header.stamp).seconds(), 1.0);
  EXPECT_DOUBLE_EQ(rclcpp::Time(drained[1]->header.stamp).seconds(), 2.0);
  EXPECT_DOUBLE_EQ(rclcpp::Time(drained[2]->header.stamp).seconds(), 3.0);
}

/**
 * @brief Verify restore functionality.
 */
TEST_F(ThreadSafeQueueTest, Restore) {
  queue.push(createMsg(3.0));

  std::deque<sensor_msgs::msg::Imu::SharedPtr> items;
  items.push_back(createMsg(1.0));
  items.push_back(createMsg(2.0));

  queue.restore(items);
  EXPECT_EQ(queue.size(), 3u);

  auto drained = queue.drain();
  EXPECT_DOUBLE_EQ(rclcpp::Time(drained[0]->header.stamp).seconds(), 1.0);
  EXPECT_DOUBLE_EQ(rclcpp::Time(drained[1]->header.stamp).seconds(), 2.0);
  EXPECT_DOUBLE_EQ(rclcpp::Time(drained[2]->header.stamp).seconds(), 3.0);
}

/**
 * @brief Verify back and pop_back functionality.
 */
TEST_F(ThreadSafeQueueTest, BackAndPopBack) {
  queue.push(createMsg(1.0));
  queue.push(createMsg(2.0));

  EXPECT_DOUBLE_EQ(rclcpp::Time(queue.back()->header.stamp).seconds(), 2.0);

  queue.pop_back();
  EXPECT_EQ(queue.size(), 1u);
  EXPECT_DOUBLE_EQ(rclcpp::Time(queue.back()->header.stamp).seconds(), 1.0);

  queue.pop_back();
  EXPECT_TRUE(queue.empty());
  EXPECT_THROW(queue.back(), std::runtime_error);
}

/**
 * @brief Verify thread safety with concurrent pushes.
 */
TEST_F(ThreadSafeQueueTest, ConcurrentPush) {
  const int num_threads = 10;
  const int pushes_per_thread = 100;
  std::vector<std::thread> threads;

  for (int i = 0; i < num_threads; ++i) {
    threads.emplace_back(
      [this, pushes_per_thread]() {
        for (int j = 0; j < pushes_per_thread; ++j) {
          queue.push(createMsg(1.0));
        }
      });
  }

  for (auto & t : threads) {
    t.join();
  }

  EXPECT_EQ(queue.size(), static_cast<size_t>(num_threads * pushes_per_thread));
}

/**
 * @brief Verify thread safety with concurrent push and drain.
 */
TEST_F(ThreadSafeQueueTest, ConcurrentPushAndDrain) {
  const int num_threads = 5;
  const int ops_per_thread = 200;
  std::vector<std::thread> producers;
  std::vector<std::thread> consumers;

  std::atomic<int> total_drained{0};

  for (int i = 0; i < num_threads; ++i) {
    producers.emplace_back(
      [this, ops_per_thread]() {
        for (int j = 0; j < ops_per_thread; ++j) {
          queue.push(createMsg(1.0));
        }
      });

    consumers.emplace_back(
      [this, ops_per_thread, &total_drained]() {
        for (int j = 0; j < ops_per_thread; ++j) {
          auto drained = queue.drain();
          total_drained += drained.size();
          std::this_thread::yield();
        }
      });
  }

  for (auto & t : producers) {
    t.join();
  }
  for (auto & t : consumers) {
    t.join();
  }

  total_drained += queue.drain().size();
  EXPECT_EQ(total_drained, num_threads * ops_per_thread);
}
