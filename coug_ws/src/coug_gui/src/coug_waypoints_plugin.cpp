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
 * @file coug_waypoints_plugin.cpp
 * @brief Implementation of the CougWaypointsPlugin class.
 * @author Nelson Durrant
 * @date Jan 2026
 */

#include <coug_gui/coug_waypoints_plugin.hpp>

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QGLWidget>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMouseEvent>
#include <QPainter>
#include <QComboBox>
#include <QLabel>
#include <swri_transform_util/frames.h>

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

PLUGINLIB_EXPORT_CLASS(coug_gui::CougWaypointsPlugin, mapviz::MapvizPlugin)


namespace coug_gui
{

CougWaypointsPlugin::CougWaypointsPlugin()
: MapvizPlugin(),
  ui_(),
  config_widget_(new QWidget()),
  map_canvas_(nullptr),
  selected_point_(-1),
  dragged_point_(-1),
  is_mouse_down_(false),
  mouse_down_time_(0)
{
  ui_.setupUi(config_widget_);

  QPalette p(config_widget_->palette());
  p.setColor(QPalette::Window, Qt::white);
  config_widget_->setPalette(p);
  QPalette p3(ui_.status->palette());
  p3.setColor(QPalette::Text, Qt::darkGreen);
  ui_.status->setPalette(p3);

  discovery_timer_ = new QTimer(this);
  QObject::connect(discovery_timer_, SIGNAL(timeout()), this, SLOT(DiscoverTopics()));

  QObject::connect(
    ui_.topic_selector, SIGNAL(currentTextChanged(const QString&)), this,
    SLOT(TopicChanged(const QString&)));
  QObject::connect(ui_.publish, SIGNAL(clicked()), this, SLOT(PublishWaypoints()));
  QObject::connect(ui_.stop, SIGNAL(clicked()), this, SLOT(Stop()));
  QObject::connect(ui_.clear, SIGNAL(clicked()), this, SLOT(Clear()));

  QObject::connect(ui_.save, SIGNAL(clicked()), this, SLOT(SaveWaypoints()));
  QObject::connect(ui_.load, SIGNAL(clicked()), this, SLOT(LoadWaypoints()));

  QObject::connect(this, SIGNAL(VisibleChanged(bool)), this, SLOT(VisibilityChanged(bool)));
  QObject::connect(
    ui_.depth_editor, SIGNAL(valueChanged(double)), this,
    SLOT(DepthChanged(double)));
}

CougWaypointsPlugin::~CougWaypointsPlugin()
{
  if (map_canvas_) {
    map_canvas_->removeEventFilter(this);
  }
}

void CougWaypointsPlugin::VisibilityChanged(bool visible)
{
  if (visible) {
    map_canvas_->installEventFilter(this);
  } else {
    map_canvas_->removeEventFilter(this);
  }
}

void CougWaypointsPlugin::DiscoverTopics()
{
  auto topics_and_types = node_->get_topic_names_and_types();
  for (const auto & [topic, types] : topics_and_types) {
    for (const auto & type : types) {
      if (type == "geometry_msgs/msg/PoseArray") {
        if (ui_.topic_selector->findText(QString::fromStdString(topic)) == -1) {
          ui_.topic_selector->addItem(QString::fromStdString(topic));

          if (waypoint_map_.find(topic) == waypoint_map_.end()) {
            waypoint_map_[topic] = std::vector<geometry_msgs::msg::Pose>();
          }
        }
        break;
      }
    }
  }
}

void CougWaypointsPlugin::TopicChanged(const QString & text)
{
  if (!current_topic_.empty()) {
    waypoint_map_[current_topic_] = waypoints_;
  }

  current_topic_ = text.toStdString();
  waypoints_ = waypoint_map_[current_topic_];

  selected_point_ = -1;
  ui_.depth_editor->setEnabled(false);

  if (waypoints_.empty()) {
    PrintInfo("Click to add waypoints");
  } else {
    PrintInfo(current_topic_ + " (" + std::to_string(waypoints_.size()) + " waypoints)");
  }
}

void CougWaypointsPlugin::PublishWaypoints()
{
  if (ui_.apply_all->isChecked()) {
    PublishAll();
  } else if (!current_topic_.empty()) {
    waypoint_map_[current_topic_] = waypoints_;
    PublishTopic(current_topic_, waypoints_);

    if (waypoints_.empty()) {
      PrintWarning("Stopped");
    } else {
      PrintInfo("Published " + std::to_string(waypoints_.size()) + " waypoints");
    }
  } else {
    PrintError("No topic selected");
  }
}

void CougWaypointsPlugin::Stop()
{
  if (ui_.apply_all->isChecked()) {
    StopAll();
  } else if (!current_topic_.empty()) {
    PublishTopic(current_topic_, {});
    PrintWarning("Stopped");
  }
}

void CougWaypointsPlugin::PublishAll()
{
  if (!current_topic_.empty()) {
    waypoint_map_[current_topic_] = waypoints_;
  }

  int count = 0;
  for (auto const & [topic, wps] : waypoint_map_) {
    PublishTopic(topic, wps);
    count++;
  }
  PrintInfo("Published all (" + std::to_string(count) + " topics)");
}

void CougWaypointsPlugin::StopAll()
{
  for (auto const & [topic, wps] : waypoint_map_) {
    PublishTopic(topic, {});
  }
  PrintWarning("Stopped all (" + std::to_string(waypoint_map_.size()) + " topics)");
}

void CougWaypointsPlugin::PublishTopic(
  const std::string & topic,
  const std::vector<geometry_msgs::msg::Pose> & wps)
{
  if (publishers_.find(topic) == publishers_.end()) {
    publishers_[topic] = node_->create_publisher<geometry_msgs::msg::PoseArray>(
      topic, rclcpp::QoS(1));
  }

  swri_transform_util::Transform transform;
  if (!tf_manager_->GetTransform(target_frame_, swri_transform_util::_wgs84_frame, transform)) {
    PrintError("No transform for " + topic);
    return;
  }

  std::vector<geometry_msgs::msg::Pose> transformed_wps;
  TransformWaypoints(wps, transformed_wps, transform);

  auto pose_array = std::make_unique<geometry_msgs::msg::PoseArray>();
  pose_array->header.frame_id = target_frame_;
  pose_array->header.stamp = node_->now();
  pose_array->poses = transformed_wps;

  publishers_[topic]->publish(*pose_array);
}

void CougWaypointsPlugin::TransformWaypoints(
  const std::vector<geometry_msgs::msg::Pose> & in,
  std::vector<geometry_msgs::msg::Pose> & out,
  const swri_transform_util::Transform & transform)
{
  out = in;
  for (size_t i = 0; i < in.size(); i++) {
    tf2::Vector3 position(in[i].position.x, in[i].position.y, 0.0);
    position = transform * position;
    out[i].position.x = position.x();
    out[i].position.y = position.y();
  }
}

int CougWaypointsPlugin::GetClosestPoint(const QPointF & point, double & distance)
{
  int closest_point = -1;
  distance = std::numeric_limits<double>::max();

  swri_transform_util::Transform transform;
  if (!tf_manager_->GetTransform(target_frame_, swri_transform_util::_wgs84_frame, transform)) {
    return -1;
  }

  for (size_t i = 0; i < waypoints_.size(); i++) {
    tf2::Vector3 waypoint(waypoints_[i].position.x, waypoints_[i].position.y, 0);
    waypoint = transform * waypoint;
    QPointF transformed =
      map_canvas_->FixedFrameToMapGlCoord(QPointF(waypoint.x(), waypoint.y()));

    double d = QLineF(transformed, point).length();
    if (d < distance) {
      distance = d;
      closest_point = static_cast<int>(i);
    }
  }
  return closest_point;
}

void CougWaypointsPlugin::SaveWaypoints()
{
  QString path = QDir::homePath() + "/coug_ws/src/coug_gui/waypoints";

  QDir dir(path);
  if (!dir.exists()) {
    dir.mkpath(".");
  }

  QString filename =
    QFileDialog::getSaveFileName(config_widget_, "Save Mission", path, "JSON Files (*.json)");
  if (filename.isEmpty()) {return;}
  if (!filename.endsWith(".json", Qt::CaseInsensitive)) {filename += ".json";}

  QJsonObject multi_topic_obj;

  if (!current_topic_.empty()) {
    waypoint_map_[current_topic_] = waypoints_;
  }

  for (auto const & [topic, wps] : waypoint_map_) {
    QJsonArray waypoints_array;
    for (const auto & wp : wps) {
      QJsonObject wp_obj;
      
      wp_obj["lon"] = wp.position.x;
      wp_obj["lat"] = wp.position.y;
      wp_obj["z"] = wp.position.z;

      swri_transform_util::Transform transform;
      if (tf_manager_->GetTransform(target_frame_, swri_transform_util::_wgs84_frame, transform)) {
        tf2::Vector3 wgs_point(wp.position.x, wp.position.y, 0.0);
        tf2::Vector3 map_point = transform * wgs_point;
        wp_obj["map_x"] = map_point.x();
        wp_obj["map_y"] = map_point.y();
      } else {
         PrintError("Transform failed");
      }

      waypoints_array.append(wp_obj);
    }
    multi_topic_obj[QString::fromStdString(topic)] = waypoints_array;
  }
  PrintInfo("Saved mission (" + std::to_string(waypoint_map_.size()) + " topics)");


  QJsonDocument doc(multi_topic_obj);
  QFile file(filename);
  if (file.open(QIODevice::WriteOnly)) {
    file.write(doc.toJson());
    file.close();
  } else {
    PrintError("Failed to save file");
  }
}

void CougWaypointsPlugin::LoadWaypoints()
{
  QString path = QDir::homePath() + "/coug_ws/src/coug_gui/waypoints";

  QString filename =
    QFileDialog::getOpenFileName(config_widget_, "Load Mission", path, "JSON Files (*.json)");
  if (filename.isEmpty()) {return;}

  QFile file(filename);
  if (!file.open(QIODevice::ReadOnly)) {
    PrintError("Failed to open file");
    return;
  }

  QByteArray data = file.readAll();
  QJsonDocument doc = QJsonDocument::fromJson(data);

  if (!doc.isObject()) {
    PrintError("Invalid mission file format");
    return;
  }

  QJsonObject obj = doc.object();

  ui_.topic_selector->blockSignals(true);

  for (auto & [topic, wps] : waypoint_map_) {
    wps.clear();
  }

  int loaded_count = 0;
  for (const QString & topic_key : obj.keys()) {
    std::string topic_str = topic_key.toStdString();

    if (waypoint_map_.find(topic_str) != waypoint_map_.end()) {
      std::vector<geometry_msgs::msg::Pose> wps;
      QJsonArray array = obj[topic_key].toArray();
      
      for (const auto & val : array) {
        QJsonObject wp_obj = val.toObject();
        geometry_msgs::msg::Pose pose;
        
        if (wp_obj.contains("lat") && wp_obj.contains("lon")) {
          pose.position.x = wp_obj["lon"].toDouble();
          pose.position.y = wp_obj["lat"].toDouble();
          pose.position.z = wp_obj["z"].toDouble();
          wps.push_back(pose);
        }
      }
      waypoint_map_[topic_str] = wps;
      loaded_count++;
    }
  }

  if (!current_topic_.empty()) {
    waypoints_ = waypoint_map_[current_topic_];
  }

  ui_.topic_selector->blockSignals(false);

  PrintInfo("Loaded mission (" + std::to_string(loaded_count) + " topics)");
}

void CougWaypointsPlugin::Clear()
{
  waypoints_.clear();
  waypoint_map_[current_topic_].clear();

  if (ui_.apply_all->isChecked()) {
    for (auto & [topic, wps] : waypoint_map_) {
      wps.clear();
    }
  }

  ui_.depth_editor->setValue(0.0);
  selected_point_ = -1;
  dragged_point_ = -1;
  ui_.depth_editor->setEnabled(false);
  PrintInfo("Cleared waypoints");
}

void CougWaypointsPlugin::DepthChanged(double value)
{
  if (selected_point_ >= 0 && static_cast<size_t>(selected_point_) < waypoints_.size()) {
    waypoints_[selected_point_].position.z = value;
  }
}

void CougWaypointsPlugin::PrintError(const std::string & message)
{
  PrintErrorHelper(ui_.status, message, 1.0);
}

void CougWaypointsPlugin::PrintInfo(const std::string & message)
{
  PrintInfoHelper(ui_.status, message, 1.0);
}

void CougWaypointsPlugin::PrintWarning(const std::string & message)
{
  PrintWarningHelper(ui_.status, message, 1.0);
}

QWidget * CougWaypointsPlugin::GetConfigWidget(QWidget * parent)
{
  config_widget_->setParent(parent);

  return config_widget_;
}

bool CougWaypointsPlugin::Initialize(QGLWidget * canvas)
{
  map_canvas_ = dynamic_cast<mapviz::MapCanvas *>(canvas);
  map_canvas_->installEventFilter(this);

  // Initialize discovery
  discovery_timer_->start(1000);
  DiscoverTopics();

  initialized_ = true;
  return true;
}

bool CougWaypointsPlugin::eventFilter(QObject * object, QEvent * event)
{
  switch (event->type()) {
    case QEvent::MouseButtonPress:
      return handleMousePress(dynamic_cast<QMouseEvent *>(event));
    case QEvent::MouseButtonRelease:
      return handleMouseRelease(dynamic_cast<QMouseEvent *>(event));
    case QEvent::MouseMove:
      return handleMouseMove(dynamic_cast<QMouseEvent *>(event));
    default:
      return false;
  }
}

bool CougWaypointsPlugin::handleMousePress(QMouseEvent * event)
{
  dragged_point_ = -1;
  double distance = 0.0;
  QPointF point = event->localPos();
  int closest_point = GetClosestPoint(point, distance);

  if (event->button() == Qt::LeftButton) {
    is_mouse_down_ = true;
    mouse_down_pos_ = event->localPos();
    mouse_down_time_ = QDateTime::currentMSecsSinceEpoch();

    if (distance < 15) {
      dragged_point_ = closest_point;
      return true;
    }
  } else if (event->button() == Qt::RightButton) {
    if (distance < 15) {
      waypoints_.erase(waypoints_.begin() + closest_point);
      if (selected_point_ == closest_point) {
        selected_point_ = -1;
        ui_.depth_editor->setEnabled(false);
      }
      return true;
    }
  }

  return false;
}

bool CougWaypointsPlugin::handleMouseRelease(QMouseEvent * event)
{
  qreal distance = QLineF(mouse_down_pos_, event->localPos()).length();
  qint64 msecsDiff = QDateTime::currentMSecsSinceEpoch() - mouse_down_time_;

  constexpr qint64 kMaxClickDuration = 500;
  constexpr double kMaxClickDistance = 5.0;

  if (dragged_point_ != -1) {
    if (distance <= kMaxClickDistance && msecsDiff < kMaxClickDuration) {
      // This was a click, not a drag. Select the point.
      selected_point_ = dragged_point_;
      ui_.depth_editor->setEnabled(true);
      ui_.depth_editor->blockSignals(true);
      ui_.depth_editor->setValue(waypoints_[selected_point_].position.z);
      ui_.depth_editor->blockSignals(false);
    }
    // If it was a drag, the point is already moved. We just stop dragging.
    dragged_point_ = -1;
    return true;
  }

  if (is_mouse_down_) {
    if (msecsDiff < kMaxClickDuration && distance <= kMaxClickDistance) {
      QPointF transformed = map_canvas_->MapGlCoordToFixedFrame(event->localPos());
      swri_transform_util::Transform transform;
      if (tf_manager_->GetTransform(swri_transform_util::_wgs84_frame, target_frame_, transform)) {
        tf2::Vector3 position(transformed.x(), transformed.y(), 0.0);
        position = transform * position;
        geometry_msgs::msg::Pose pose;
        pose.position.x = position.x();
        pose.position.y = position.y();
        pose.position.z = ui_.depth_editor->value();

        waypoints_.push_back(pose);
        waypoint_map_[current_topic_] = waypoints_;
      }
    }
  }
  is_mouse_down_ = false;
  dragged_point_ = -1;
  return false;
}

bool CougWaypointsPlugin::handleMouseMove(QMouseEvent * event)
{
  if (dragged_point_ >= 0) {
    if (selected_point_ != -1) {
      selected_point_ = -1;
      ui_.depth_editor->setEnabled(false);
    }

    QPointF point = event->localPos();
    swri_transform_util::Transform transform;
    if (tf_manager_->GetTransform(swri_transform_util::_wgs84_frame, target_frame_, transform)) {
      QPointF transformed = map_canvas_->MapGlCoordToFixedFrame(point);
      tf2::Vector3 position(transformed.x(), transformed.y(), 0.0);
      position = transform * position;
      waypoints_[dragged_point_].position.x = position.x();

      waypoints_[dragged_point_].position.y = position.y();
      waypoint_map_[current_topic_] = waypoints_;
    }
    return true;
  }
  return false;
}

void CougWaypointsPlugin::Draw(double x, double y, double scale)
{
  swri_transform_util::Transform transform;
  if (!tf_manager_->GetTransform(target_frame_, swri_transform_util::_wgs84_frame, transform)) {
    return;
  }

  glLineWidth(2);

  // Draw Ghost Paths
  for (auto const & [topic, wps] : waypoint_map_) {
    if (topic == current_topic_) {continue;}
    DrawLines(wps, 1.0, 1.0, 1.0, 1.0, transform);  // White
  }

  // Draw Current Path
  DrawLines(waypoints_, 0.0, 0.0, 1.0, 1.0, transform);  // Blue

  // Draw Ghost Points
  for (auto const & [topic, wps] : waypoint_map_) {
    if (topic == current_topic_) {continue;}
    DrawPoints(wps, 0.5, 0.5, 0.5, 1.0, transform);  // Gray
  }

  // Draw Current Points
  DrawPoints(waypoints_, 0.0, 1.0, 1.0, 1.0, transform, selected_point_);  // Cyan (Yellow selected)
}

void CougWaypointsPlugin::Paint(QPainter * painter, double x, double y, double scale)
{
  swri_transform_util::Transform transform;
  if (!tf_manager_->GetTransform(target_frame_, swri_transform_util::_wgs84_frame, transform)) {
    return;
  }

  painter->save();
  painter->resetTransform();
  painter->setFont(QFont("DejaVu Sans Mono", 10, QFont::Bold));

  // Paint Ghost Labels
  for (auto const & [topic, wps] : waypoint_map_) {
    if (topic == current_topic_) {continue;}
    PaintLabels(painter, wps, transform, QColor(255, 255, 255, 200));
  }

  // Paint Current Labels
  PaintLabels(painter, waypoints_, transform, Qt::white);

  painter->restore();
}

void CougWaypointsPlugin::DrawLines(
  const std::vector<geometry_msgs::msg::Pose> & wps,
  float r, float g, float b, float a,
  const swri_transform_util::Transform & transform)
{
  glColor4f(r, g, b, a);
  glBegin(GL_LINE_STRIP);
  for (const auto & wp : wps) {
    tf2::Vector3 point(wp.position.x, wp.position.y, 0);
    point = transform * point;
    glVertex2d(point.x(), point.y());
  }
  glEnd();
}

void CougWaypointsPlugin::DrawPoints(
  const std::vector<geometry_msgs::msg::Pose> & wps,
  float r, float g, float b, float a,
  const swri_transform_util::Transform & transform,
  int selected_index)
{
  glPointSize(20);
  glBegin(GL_POINTS);
  for (size_t i = 0; i < wps.size(); ++i) {
    if (static_cast<int>(i) == selected_index) {
      glColor4f(1.0, 1.0, 0.0, 1.0);  // Yellow
    } else {
      glColor4f(r, g, b, a);
    }

    tf2::Vector3 point(wps[i].position.x, wps[i].position.y, 0);
    point = transform * point;
    glVertex2d(point.x(), point.y());
  }
  glEnd();
}

void CougWaypointsPlugin::PaintLabels(
  QPainter * painter,
  const std::vector<geometry_msgs::msg::Pose> & wps,
  const swri_transform_util::Transform & transform,
  const QColor & color)
{
  for (size_t i = 0; i < wps.size(); i++) {
    tf2::Vector3 point(wps[i].position.x, wps[i].position.y, 0);
    point = transform * point;
    QPointF gl_point = map_canvas_->FixedFrameToMapGlCoord(QPointF(point.x(), point.y()));

    painter->setPen(QPen(color));

    // Depth Text
    QPointF depth_text_corner(gl_point.x() - 50, gl_point.y() + 15);
    QRectF depth_text_rect(depth_text_corner, QSizeF(100, 20));
    QString depth_text = QString::number(wps[i].position.z, 'f', 1) + "m";
    painter->drawText(depth_text_rect, Qt::AlignHCenter | Qt::AlignTop, depth_text);

    // Number
    if (color == Qt::white) {
      painter->setPen(QPen(Qt::black));
    } else {
      painter->setPen(QPen(color));
    }

    QPointF num_corner(gl_point.x() - 20, gl_point.y() - 20);
    QRectF num_rect(num_corner, QSizeF(40, 40));
    painter->drawText(num_rect, Qt::AlignHCenter | Qt::AlignVCenter, QString::number(i + 1));
  }
}

void CougWaypointsPlugin::LoadConfig(const YAML::Node & node, const std::string & path)
{
  (void)node;
  (void)path;
}

void CougWaypointsPlugin::SaveConfig(YAML::Emitter & emitter, const std::string & path)
{
  (void)emitter;
  (void)path;
}

}  // namespace coug_gui
