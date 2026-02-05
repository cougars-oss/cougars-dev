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
 * @file coug_waypoints_plugin.hpp
 * @brief MapViz plugin for multi-agent waypoint mission planning.
 * @author Nelson Durrant
 * @date Jan 2026
 */

#pragma once

#include <ui_coug_waypoints_config.h>

#include <mapviz/map_canvas.h>
#include <mapviz/mapviz_plugin.h>
#include <QGLWidget>
#include <QObject>
#include <QTimer>
#include <QWidget>

#include <string>
#include <vector>
#include <map>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/transform_datatypes.hpp>

namespace coug_gui
{

/**
 * @class CougWaypointsPlugin
 * @brief A MapViz plugin that allows users to place, edit, and publish waypoints for multiple agents.
 *
 * This plugin supports:
 * - Geometric placement of waypoints on a map.
 * - Multi-agent support via ROS 2 topic discovery.
 * - Saving and loading mission plans to/from JSON.
 * - Modifying waypoint depth.
 * - Visualization of active and ghost (non-selected agent) paths.
 */
class CougWaypointsPlugin : public mapviz::MapvizPlugin
{
  Q_OBJECT

public:
  /**
   * @brief Constructor.
   */
  CougWaypointsPlugin();

  /**
   * @brief Destructor.
   */
  ~CougWaypointsPlugin() override;

  /**
   * @brief Initializes the plugin with the MapViz canvas.
   * @param canvas The OpenGL widget used for drawing.
   * @return True if initialization was successful.
   */
  bool Initialize(QGLWidget * canvas) override;

  /**
   * @brief Shuts down the plugin.
   */
  void Shutdown() override {}

  /**
   * @brief Draws the waypoints and paths on the map using OpenGL.
   * @param x Camera X position.
   * @param y Camera Y position.
   * @param scale Camera zoom scale.
   */
  void Draw(double x, double y, double scale) override;

  /**
   * @brief Paints 2D overlays (text labels) on top of the map.
   * @param painter The QPainter object.
   * @param x Camera X position.
   * @param y Camera Y position.
   * @param scale Camera zoom scale.
   */
  void Paint(QPainter * painter, double x, double y, double scale) override;

  /**
   * @brief Transforms coordinates (unused).
   */
  void Transform() override {}

  /**
   * @brief Loads configuration from YAML (unused, auto-discovery used instead).
   * @param node The YAML node.
   * @param path The path to the config file.
   */
  void LoadConfig(const YAML::Node & node, const std::string & path) override;

  /**
   * @brief Saves configuration to YAML (unused).
   * @param emitter The YAML emitter.
   * @param path The path to the config file.
   */
  void SaveConfig(YAML::Emitter & emitter, const std::string & path) override;

  /**
   * @brief Gets the configuration widget for the plugin.
   * @param parent The parent widget.
   * @return The configuration widget.
   */
  QWidget * GetConfigWidget(QWidget * parent) override;

  /**
   * @brief Checks if the plugin supports painting.
   * @return True.
   */
  bool SupportsPainting() override {return true;}

protected:
  /**
   * @brief Prints an error message to the status label.
   * @param message The message to print.
   */
  void PrintError(const std::string & message) override;

  /**
   * @brief Prints an info message to the status label.
   * @param message The message to print.
   */
  void PrintInfo(const std::string & message) override;

  /**
   * @brief Prints a warning message to the status label.
   * @param message The message to print.
   */
  void PrintWarning(const std::string & message) override;

  /**
   * @brief Event filter for handling mouse events on the canvas.
   * @param object The object receiving the event.
   * @param event The event.
   * @return True if the event was handled.
   */
  bool eventFilter(QObject * object, QEvent * event) override;

  /**
   * @brief Handles mouse press events for adding/selecting waypoints.
   * @param event The mouse event.
   * @return True if handled.
   */
  bool handleMousePress(QMouseEvent * event);

  /**
   * @brief Handles mouse release events.
   * @param event The mouse event.
   * @return True if handled.
   */
  bool handleMouseRelease(QMouseEvent * event);

  /**
   * @brief Handles mouse move events for dragging waypoints.
   * @param event The mouse event.
   * @return True if handled.
   */
  bool handleMouseMove(QMouseEvent * event);

protected Q_SLOTS:
  /**
   * @brief Publishes the current agent's waypoints to ROS.
   */
  void PublishWaypoints();

  /**
   * @brief Publishes an empty path to stop the current agent.
   */
  void Stop();

  /**
   * @brief Clears the current agent's waypoints (or all if "Apply to All" is checked).
   */
  void Clear();

  /**
   * @brief Saves all mission plans to a JSON file.
   */
  void SaveWaypoints();

  /**
   * @brief Loads mission plans from a JSON file.
   */
  void LoadWaypoints();

  /**
   * @brief Handles agent selection changes.
   * @param text The new topic name.
   */
  void TopicChanged(const QString & text);

  /**
   * @brief Discovers new PoseArray topics and adds them to the selector.
   */
  void DiscoverTopics();

  /**
   * @brief Publishes waypoints for all discovered agents.
   */
  void PublishAll();

  /**
   * @brief Publishes empty paths for all discovered agents to stop them.
   */
  void StopAll();

  /**
   * @brief Handles visibility changes for the plugin.
   * @param visible True if visible.
   */
  void VisibilityChanged(bool visible);

  /**
   * @brief Updates the depth of the selected waypoint.
   * @param value The new depth value.
   */
  void DepthChanged(double value);

private:
  Ui::coug_waypoints_config ui_;
  QWidget * config_widget_;
  mapviz::MapCanvas * map_canvas_;

  std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr> publishers_;
  std::map<std::string, std::vector<geometry_msgs::msg::Pose>> waypoint_map_;
  std::string current_topic_;
  std::vector<geometry_msgs::msg::Pose> waypoints_;

  int selected_point_;
  int dragged_point_;
  bool is_mouse_down_;
  QPointF mouse_down_pos_;
  qint64 mouse_down_time_;

  QTimer * discovery_timer_;

  /**
   * @brief Helper to publish waypoints to a specific topic.
   * @param topic The topic name.
   * @param wps The waypoints to publish.
   */
  void PublishTopic(
    const std::string & topic,
    const std::vector<geometry_msgs::msg::Pose> & wps);

  /**
   * @brief Finds the closest waypoint to a point on the screen.
   * @param point The screen point.
   * @param distance Output reference for the distance to the closest point.
   * @return The index of the closest point, or -1 if none found.
   */
  int GetClosestPoint(const QPointF & point, double & distance);

  /**
   * @brief Helper to draw lines connecting waypoints.
   * @param wps Waypoints to draw.
   * @param r Red component.
   * @param g Green component.
   * @param b Blue component.
   * @param a Alpha component.
   * @param transform The geometric transform to use.
   */
  void DrawLines(
    const std::vector<geometry_msgs::msg::Pose> & wps,
    float r, float g, float b, float a,
    const swri_transform_util::Transform & transform);

  /**
   * @brief Helper to draw waypoint points.
   * @param wps Waypoints to draw.
   * @param r Red component.
   * @param g Green component.
   * @param b Blue component.
   * @param a Alpha component.
   * @param transform The geometric transform to use.
   * @param selected_index Index of a selected point to highlight (optional).
   */
  void DrawPoints(
    const std::vector<geometry_msgs::msg::Pose> & wps,
    float r, float g, float b, float a,
    const swri_transform_util::Transform & transform,
    int selected_index = -1);

  /**
   * @brief Helper to paint text labels (indices and depths) for waypoints.
   * @param painter The QPainter.
   * @param wps Waypoints to label.
   * @param transform The geometric transform to use.
   * @param color THe color of the text/box.
   */
  void PaintLabels(
    QPainter * painter,
    const std::vector<geometry_msgs::msg::Pose> & wps,
    const swri_transform_util::Transform & transform,
    const QColor & color);

  /**
   * @brief Transforms local waypoints to the target frame for publishing.
   * @param in Input waypoints.
   * @param out Output transformed waypoints.
   * @param transform The transform to apply.
   */
  void TransformWaypoints(
    const std::vector<geometry_msgs::msg::Pose> & in,
    std::vector<geometry_msgs::msg::Pose> & out,
    const swri_transform_util::Transform & transform);
};

}  // namespace coug_gui
