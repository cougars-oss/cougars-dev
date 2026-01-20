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
 * @file factor_graph_node.hpp
 * @brief ROS 2 node for factor graph optimization using GTSAM and fixed-lag smoothing.
 * @author Nelson Durrant
 * @date Jan 2026
 */

#pragma once

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "coug_fgo/utils/conversion_utils.hpp"
#include "coug_fgo/utils/dvl_preintegrator.hpp"


namespace coug_fgo
{

/**
 * @class FactorGraphNode
 * @brief Performs factor graph optimization for AUV navigation.
 *
 * This node integrates measurements from IMU, DVL, GPS, depth, magnetometer, and AHRS
 * sensors into a global factor graph. It uses fixed-lag smoothing (ISAM2) to estimate
 * the AUV's pose, velocity, and IMU biases in real-time.
 */
class FactorGraphNode : public rclcpp::Node
{
public:
  /**
   * @brief FactorGraphNode constructor.
   */
  FactorGraphNode();

  /** @name Parameter Structures */
  /// @{
  struct ImuParams
  {
    /// If true, use the frame ID parameter below instead of the header frame.
    bool use_parameter_frame;
    /// The frame ID to use if use_parameter_frame is true.
    std::string parameter_frame;
    /// Use fixed sigma values instead of message covariance.
    bool use_parameter_covariance;
    /// Accelerometer noise sigma (continuous-time) [m/s^2].
    double accel_noise_sigma;
    /// Gyroscope noise sigma (continuous-time) [rad/s].
    double gyro_noise_sigma;
    /// Accelerometer bias random walk sigma [m/s^3].
    double accel_bias_rw_sigma;
    /// Gyroscope bias random walk sigma [rad/s^2].
    double gyro_bias_rw_sigma;
    /// Covariance for IMU integration step.
    double integration_covariance;
    /// Gravity vector in world frame [m/s^2].
    std::vector<double> gravity;
  };

  struct GpsParams
  {
    /// Enable/disable GPS factor processing.
    bool enable;
    /// If true, use the frame ID parameter below instead of the header frame.
    bool use_parameter_frame;
    /// The frame ID to use if use_parameter_frame is true.
    std::string parameter_frame;
    /// Use fixed sigma values instead of message covariance.
    bool use_parameter_covariance;
    /// Horizontal position noise sigma [m].
    double position_noise_sigma;
    /// Vertical position noise sigma [m] (can be set high to ignore GPS altitude).
    double altitude_noise_sigma;
    /// Type of robust cost function (e.g., "Huber").
    std::string robust_kernel;
    /// Threshold for the robust kernel.
    double robust_k;
  };

  struct DepthParams
  {
    /// If true, use the frame ID parameter below instead of the header frame.
    bool use_parameter_frame;
    /// The frame ID to use if use_parameter_frame is true.
    std::string parameter_frame;
    /// Use fixed sigma values instead of message covariance.
    bool use_parameter_covariance;
    /// Depth (Z) noise sigma [m].
    double position_z_noise_sigma;
    /// Type of robust cost function.
    std::string robust_kernel;
    /// Threshold for the robust kernel.
    double robust_k;
  };

  struct MagParams
  {
    /// Enable/disable magnetic factor processing.
    bool enable;
    /// If true, use the frame ID parameter below instead of the header frame.
    bool use_parameter_frame;
    /// The frame ID to use if use_parameter_frame is true.
    std::string parameter_frame;
    /// Flag to constrain only the yaw angle.
    bool constrain_yaw_only;
    /// Use fixed sigma values instead of message covariance.
    bool use_parameter_covariance;
    /// Magnetic field noise sigma.
    double magnetic_field_noise_sigma;
    /// Type of robust cost function.
    std::string robust_kernel;
    /// Threshold for the robust kernel.
    double robust_k;
    /// Reference magnetic field vector (world frame).
    std::vector<double> reference_field;
  };

  struct AhrsParams
  {
    /// Enable/disable AHRS factor processing.
    bool enable_ahrs;
    /// If true, use the frame ID parameter below instead of the header frame.
    bool use_parameter_frame;
    /// The frame ID to use if use_parameter_frame is true.
    std::string parameter_frame;
    /// Use fixed sigma values instead of message covariance.
    bool use_parameter_covariance;
    /// Yaw noise sigma [rad].
    double yaw_noise_sigma;
    /// Roll/Pitch noise sigma [rad] (can be set high to ignore).
    double roll_pitch_noise_sigma;
    /// Type of robust cost function.
    std::string robust_kernel;
    /// Threshold for the robust kernel.
    double robust_k;
  };

  struct DvlParams
  {
    /// Use fixed sigma values instead of message covariance.
    bool use_parameter_covariance;
    /// If true, use the frame ID parameter below instead of the header frame.
    bool use_parameter_frame;
    /// The frame ID to use if use_parameter_frame is true.
    std::string parameter_frame;
    /// Velocity noise sigma [m/s].
    double velocity_noise_sigma;
    /// Timeout threshold (DVL dropout detection) [s].
    double timeout_threshold;
    /// Type of robust cost function.
    std::string robust_kernel;
    /// Threshold for the robust kernel.
    double robust_k;
  };

  struct PriorParams
  {
    /// Warm-up period for sensor data averaging (seconds).
    double initialization_duration;

    /// If true, use parameter values for priors instead of initial sensor data.
    bool use_parameter_priors;
    /// Static initial position prior [x,y,z].
    std::vector<double> initial_position;
    /// Static initial orientation prior [r,p,y].
    std::vector<double> initial_orientation;
    /// Static initial velocity prior [x,y,z].
    std::vector<double> initial_velocity;
    /// Static initial accelerometer bias prior [x,y,z].
    std::vector<double> initial_accel_bias;
    /// Static initial gyroscope bias prior [x,y,z].
    std::vector<double> initial_gyro_bias;
    /// Uncertainty sigma for initial position.
    double initial_position_sigma;
    /// Uncertainty sigma for initial orientation.
    double initial_orientation_sigma;
    /// Uncertainty sigma for initial velocity.
    double initial_velocity_sigma;

    /// Uncertainty sigma for initial accelerometer bias.
    double initial_accel_bias_sigma;
    /// Uncertainty sigma for initial gyroscope bias.
    double initial_gyro_bias_sigma;
  };

  struct ExperimentalParams
  {
    /// Enable iterative preintegration of DVL measurements between IMU poses.
    bool enable_dvl_preintegration;
  };

  /// Container for averaged sensor data used during initialization.
  struct AveragedMeasurements
  {
    sensor_msgs::msg::Imu::SharedPtr imu;
    nav_msgs::msg::Odometry::SharedPtr gps;
    nav_msgs::msg::Odometry::SharedPtr depth;
    sensor_msgs::msg::Imu::SharedPtr ahrs;
    sensor_msgs::msg::MagneticField::SharedPtr mag;
    geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr dvl;
  };

private:
  // --- Main Logic ---
  /**
   * @brief Initializes the factor graph using averaged sensor data or parameters.
   */
  void initializeGraph();
  /**
   * @brief Main optimization loop; adds factors to the graph and invokes the smoother.
   */
  void optimizeGraph();

  // --- Setup & Helpers ---
  /**
   * @brief Loads ROS 2 parameters into local structures.
   */
  void loadParameters();
  /**
   * @brief Initializes publishers, subscribers, and timers.
   */
  void setupRosInterfaces();
  /**
   * @brief Looks up required static transforms (e.g., imu->dvl).
   * @return True if all transforms were found.
   */
  bool lookupInitialTransforms();
  /**
   * @brief Configures GTSAM combined IMU preintegration parameters.
   * @return GTSAM preintegration parameters.
   */
  boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params>
  configureImuPreintegration();

  /**
   * @brief Averages sensor queues over a specified duration.
   * @param imu_msgs Queue of IMU messages.
   * @param gps_msgs Queue of GPS messages.
   * @param depth_msgs Queue of depth messages.
   * @param mag_msgs Queue of magnetic field messages.
   * @param ahrs_msgs Queue of AHRS messages.
   * @param dvl_msgs Queue of DVL messages.
   * @return The averaged measurements.
   */
  AveragedMeasurements computeAveragedMeasurements(
    const std::deque<sensor_msgs::msg::Imu::SharedPtr> & imu_msgs,
    const std::deque<nav_msgs::msg::Odometry::SharedPtr> & gps_msgs,
    const std::deque<nav_msgs::msg::Odometry::SharedPtr> & depth_msgs,
    const std::deque<sensor_msgs::msg::MagneticField::SharedPtr> & mag_msgs,
    const std::deque<sensor_msgs::msg::Imu::SharedPtr> & ahrs_msgs,
    const std::deque<geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr> & dvl_msgs);

  /**
   * @brief Computes initial orientation using IMU, magnetometer, and AHRS sensor.
   * @return The initial GTSAM rotation.
   */
  gtsam::Rot3 computeInitialOrientation();
  /**
   * @brief Computes initial position using GPS and depth sensor.
   * @param initial_orientation The previously computed initial orientation.
   * @return The initial GTSAM translation.
   */
  gtsam::Point3 computeInitialPosition(const gtsam::Rot3 & initial_orientation);
  /**
   * @brief Computes initial velocity using DVL measurements.
   * @param initial_orientation The previously computed initial orientation.
   * @return The initial GTSAM velocity.
   */
  gtsam::Vector3 computeInitialVelocity(const gtsam::Rot3 & initial_orientation);
  /**
   * @brief Computes initial IMU biases from averaged data or parameters.
   * @return The initial GTSAM bias estimate.
   */
  gtsam::imuBias::ConstantBias computeInitialBias();

  // --- Factor Management ---
  /**
   * @brief Adds prior factors to the GTSAM graph.
   * @param graph The target factor graph.
   * @param values The initial value estimates.
   */
  void addPriorFactors(gtsam::NonlinearFactorGraph & graph, gtsam::Values & values);
  /**
   * @brief Adds a GPS position factor to the graph.
   * @param graph The target factor graph.
   * @param gps_msgs Queue of GPS messages to process.
   */
  void addGpsFactor(
    gtsam::NonlinearFactorGraph & graph,
    const std::deque<nav_msgs::msg::Odometry::SharedPtr> & gps_msgs);
  /**
   * @brief Adds a depth (Z) factor to the graph.
   * @param graph The target factor graph.
   * @param depth_msgs Queue of depth messages to process.
   */
  void addDepthFactor(
    gtsam::NonlinearFactorGraph & graph,
    const std::deque<nav_msgs::msg::Odometry::SharedPtr> & depth_msgs);
  /**
   * @brief Adds a magnetic orientation factor to the graph.
   * @param graph The target factor graph.
   * @param mag_msgs Queue of Mag messages.
   */
  void addMagFactor(
    gtsam::NonlinearFactorGraph & graph,
    const std::deque<sensor_msgs::msg::MagneticField::SharedPtr> & mag_msgs);
  /**
   * @brief Adds a AHRS orientation factor to the graph.
   * @param graph The target factor graph.
   * @param ahrs_msgs Queue of AHRS messages to process.
   */
  void addAhrsFactor(
    gtsam::NonlinearFactorGraph & graph,
    const std::deque<sensor_msgs::msg::Imu::SharedPtr> & ahrs_msgs);
  /**
   * @brief Adds a velocity (DVL) factor to the graph.
   * @param graph The target factor graph.
   * @param dvl_msgs Queue of DVL messages to process.
   */
  void addDvlFactor(
    gtsam::NonlinearFactorGraph & graph,
    const std::deque<geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr> & dvl_msgs);
  /**
   * @brief Integrates and adds a combined IMU factor to the graph.
   * @param graph The target factor graph.
   * @param imu_msgs Queue of IMU messages since the last pose.
   * @param target_time The timestamp for the new pose key.
   */
  void addPreintegratedImuFactor(
    gtsam::NonlinearFactorGraph & graph,
    const std::deque<sensor_msgs::msg::Imu::SharedPtr> & imu_msgs,
    double target_time);
  /**
   * @brief Interpolates orientation between IMU messages.
   * @param imu_msgs IMU message history.
   * @param target_time Integration target time.
   * @return The interpolated GTSAM rotation.
   */
  gtsam::Rot3 getInterpolatedOrientation(
    const std::deque<sensor_msgs::msg::Imu::SharedPtr> & imu_msgs, double target_time);
  /**
   * @brief [EXPERIMENTAL] Adds a preintegrated DVL translation factor.
   * @param graph The target factor graph.
   * @param dvl_msgs Queue of DVL measurements.
   * @param imu_msgs Queue of IMU measurements for orientation.
   * @param target_time The timestamp for the new pose key.
   */
  void addPreintegratedDvlFactor(
    gtsam::NonlinearFactorGraph & graph,
    const std::deque<geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr> & dvl_msgs,
    const std::deque<sensor_msgs::msg::Imu::SharedPtr> & imu_msgs, double target_time);

  // --- Publishing ---
  /**
   * @brief Publishes the optimized global odometry.
   * @param current_pose The estimated pose.
   * @param pose_covariance The estimation error covariance.
   * @param timestamp The message timestamp.
   */
  void publishGlobalOdom(
    const gtsam::Pose3 & current_pose, const gtsam::Matrix & pose_covariance,
    const rclcpp::Time & timestamp);
  /**
   * @brief Broadcasts the map-to-odom transform.
   * @param current_pose The estimated pose.
   * @param timestamp The transform timestamp.
   */
  void broadcastGlobalTf(const gtsam::Pose3 & current_pose, const rclcpp::Time & timestamp);
  /**
   * @brief Publishes the full optimized trajectory path.
   * @param results The final optimized values.
   * @param timestamp The path timestamp.
   */
  void publishSmoothedPath(const gtsam::Values & results, const rclcpp::Time & timestamp);
  /**
   * @brief Publishes the optimized body-frame velocity.
   * @param current_vel The estimated velocity.
   * @param vel_covariance The estimation error covariance.
   * @param timestamp The message timestamp.
   */
  void publishVelocity(
    const gtsam::Vector3 & current_vel, const gtsam::Matrix & vel_covariance,
    const rclcpp::Time & timestamp);
  /**
   * @brief Publishes the optimized IMU biases.
   * @param current_imu_bias The estimated biases.
   * @param imu_bias_covariance The estimation error covariance.
   * @param timestamp The message timestamp.
   */
  void publishImuBias(
    const gtsam::imuBias::ConstantBias & current_imu_bias,
    const gtsam::Matrix & imu_bias_covariance, const rclcpp::Time & timestamp);

  // --- Graph State ---
  /// True if all required sensors have published initial data.
  bool sensors_ready_ = false;
  /// True if the initialization averaging phase is complete.
  bool data_averaged_ = false;
  /// True if the smoother and graph have been initialized.
  bool graph_initialized_ = false;
  /// Start time of the initialization averaging phase.
  double start_avg_time_ = 0.0;

  /// Index of the previous optimization step/key.
  size_t prev_step_ = 0;
  /// Index of the current optimization step/key.
  size_t current_step_ = 1;
  /// Timestamp of the last processed data (seconds).
  double prev_time_ = 0.0;
  /// Timestamp of the last DVL message (seconds).
  double last_dvl_time_ = 0.0;
  /// Mapping between timestamps and GTSAM pose keys.
  std::map<rclcpp::Time, gtsam::Key> time_to_key_;

  // --- GTSAM Objects ---
  /// Core GTSAM smoother (fixed-lag ISAM2).
  std::unique_ptr<gtsam::IncrementalFixedLagSmoother> smoother_;
  /// GTSAM preintegrator for IMU measurements.
  std::unique_ptr<gtsam::PreintegratedCombinedMeasurements> imu_preintegrator_;
  /// Helper for DVL preintegration.
  std::unique_ptr<utils::DVLPreintegrator> dvl_preintegrator_;

  /// Most recent estimated pose.
  gtsam::Pose3 prev_pose_;
  /// Most recent estimated velocity.
  gtsam::Vector3 prev_vel_;
  /// Most recent estimated IMU bias.
  gtsam::imuBias::ConstantBias prev_imu_bias_;

  // --- Initial Averaged Measurements ---
  /// Averaged IMU data used for orientation initialization.
  sensor_msgs::msg::Imu::SharedPtr initial_imu_;
  /// Averaged GPS data used for position initialization.
  nav_msgs::msg::Odometry::SharedPtr initial_gps_;
  /// Averaged depth data used for position initialization.
  nav_msgs::msg::Odometry::SharedPtr initial_depth_;
  /// Averaged AHRS data used for orientation initialization.
  sensor_msgs::msg::Imu::SharedPtr initial_ahrs_;
  /// Averaged magnetic data used for orientation initialization.
  sensor_msgs::msg::MagneticField::SharedPtr initial_mag_;
  /// Averaged DVL data used for velocity initialization.
  geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr initial_dvl_;

  // --- Message Queues ---
  /// Queue of incoming IMU messages.
  std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_queue_;
  /// Queue of incoming GPS messages.
  std::deque<nav_msgs::msg::Odometry::SharedPtr> gps_queue_;
  /// Queue of incoming depth messages.
  std::deque<nav_msgs::msg::Odometry::SharedPtr> depth_queue_;
  /// Queue of incoming Mag messages.
  std::deque<sensor_msgs::msg::MagneticField::SharedPtr> mag_queue_;
  /// Queue of incoming AHRS messages.
  std::deque<sensor_msgs::msg::Imu::SharedPtr> ahrs_queue_;
  /// Queue of incoming DVL messages.
  std::deque<geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr> dvl_queue_;

  // --- Multithreading ---
  /// Separate callback group for sensor subscribers.
  rclcpp::CallbackGroup::SharedPtr sensor_cb_group_;

  /// Lock for the main optimization loop.
  std::mutex optimization_mutex_;
  /// Lock for the IMU message queue.
  std::mutex imu_queue_mutex_;
  /// Lock for the GPS message queue.
  std::mutex gps_queue_mutex_;
  /// Lock for the depth message queue.
  std::mutex depth_queue_mutex_;
  /// Lock for the Mag message queue.
  std::mutex mag_queue_mutex_;
  /// Lock for the AHRS message queue.
  std::mutex ahrs_queue_mutex_;
  /// Lock for the DVL message queue.
  std::mutex dvl_queue_mutex_;

  // --- Transformations ---
  /// Frame ID of the DVL sensor.
  std::string dvl_frame_;
  /// Frame ID of the IMU sensor.
  std::string imu_frame_;

  /// True if the DVL-to-base transform has been successfully looked up.
  bool have_dvl_to_base_tf_ = false;
  /// True if the IMU-to-DVL transform has been successfully looked up.
  bool have_imu_to_dvl_tf_ = false;
  /// True if the GPS-to-DVL transform has been successfully looked up.
  bool have_gps_to_dvl_tf_ = false;
  /// True if the depth-to-DVL transform has been successfully looked up.
  bool have_depth_to_dvl_tf_ = false;
  /// True if the Mag-to-DVL transform has been successfully looked up.
  bool have_mag_to_dvl_tf_ = false;
  /// True if the AHRS-to-DVL transform has been successfully looked up.
  bool have_ahrs_to_dvl_tf_ = false;

  /// Transformation from the DVL frame to the base link frame.
  geometry_msgs::msg::TransformStamped dvl_to_base_tf_;
  /// Transformation from the IMU frame to the DVL frame.
  geometry_msgs::msg::TransformStamped imu_to_dvl_tf_;
  /// Transformation from the GPS frame to the DVL frame.
  geometry_msgs::msg::TransformStamped gps_to_dvl_tf_;
  /// Transformation from the depth sensor frame to the DVL frame.
  geometry_msgs::msg::TransformStamped depth_to_dvl_tf_;
  /// Transformation from the Mag sensor frame to the DVL frame.
  geometry_msgs::msg::TransformStamped mag_to_dvl_tf_;
  /// Transformation from the AHRS sensor frame to the DVL frame.
  geometry_msgs::msg::TransformStamped ahrs_to_dvl_tf_;

  // --- ROS Interfaces ---
  /// Optimized global odometry publisher.
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr global_odom_pub_;
  /// Optimized path/trajectory publisher.
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr smoothed_path_pub_;
  /// Body-frame velocity publisher.
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr velocity_pub_;
  /// IMU bias estimates publisher.
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr imu_bias_pub_;

  /// Raw IMU data subscriber.
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  /// Preprocessed GPS ENU subscriber.
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gps_odom_sub_;
  /// Depth odom subscriber.
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr depth_odom_sub_;
  /// Magnetic field subscriber.
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
  /// AHRS IMU subscriber.
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr ahrs_sub_;
  /// Body-frame DVL twist subscriber.
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr dvl_sub_;
  /// Timer for the main factor graph optimization loop.
  rclcpp::TimerBase::SharedPtr factor_graph_timer_;

  /// ROS 2 TF transform broadcaster.
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  /// ROS 2 TF transform listener.
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  /// ROS 2 TF buffer.
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  // --- Parameters ---
  /// Enable/disable broadcasting the map->odom transform.
  bool publish_global_tf_;
  /// Enable/disable publishing the optimized trajectory.
  bool publish_smoothed_path_;
  /// Enable/disable publishing body-frame velocity.
  bool publish_velocity_;
  /// Enable/disable publishing IMU bias estimates.
  bool publish_imu_bias_;

  /// Include pose covariance in odometry messages.
  bool publish_pose_cov_;
  /// Include velocity covariance in twist messages.
  bool publish_velocity_cov_;
  /// Include bias covariance in bias messages.
  bool publish_imu_bias_cov_;

  /// Fixed-lag smoothing window (seconds).
  double smoother_lag_;
  /// ISAM2 relinearization threshold.
  double gtsam_relinearize_threshold_;
  /// ISAM2 relinearization skip count.
  int gtsam_relinearize_skip_;

  /// Raw IMU data topic name.
  std::string imu_topic_;
  /// Preprocessed GPS ENU topic name.
  std::string gps_odom_topic_;
  /// Depth odom topic name.
  std::string depth_odom_topic_;
  /// Magnetic field topic name.
  std::string mag_topic_;
  /// AHRS IMU topic name.
  std::string ahrs_topic_;
  /// Body-frame DVL twist topic name.
  std::string dvl_topic_;
  /// Optimized global odometry topic name.
  std::string global_odom_topic_;
  /// Optimized trajectory topic name.
  std::string smoothed_path_topic_;
  /// Optimized velocity topic name.
  std::string velocity_topic_;
  /// Optimized IMU bias topic name.
  std::string imu_bias_topic_;

  /// Global world frame (ENU).
  std::string map_frame_;
  /// Dead-reckoning frame.
  std::string odom_frame_;
  /// AUV body-fixed frame.
  std::string base_frame_;

  /// Configuration for IMU processing.
  ImuParams imu_params_;
  /// Configuration for GPS processing.
  GpsParams gps_params_;
  /// Configuration for depth processing.
  DepthParams depth_params_;
  /// Configuration for Mag processing.
  MagParams mag_params_;
  /// Configuration for AHRS processing.
  AhrsParams ahrs_params_;
  /// Configuration for DVL processing.
  DvlParams dvl_params_;
  /// Configuration for priors and initialization.
  PriorParams prior_params_;
  /// Configuration for experimental features.
  ExperimentalParams experimental_params_;
};

}  // namespace coug_fgo
