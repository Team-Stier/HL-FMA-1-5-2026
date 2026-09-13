#pragma once

#include <diagnostic_msgs/DiagnosticArray.h>
#include <ros/ros.h>
#include <rviz/config.h>
#include <rviz/panel.h>

#include <QColor>
#include <QLabel>
#include <QTimer>
#include <QTreeWidget>

#include <mutex>
#include <string>

namespace localization_rviz {

/** RViz-only observer for the DiagnosticArray emitted by Localization. */
class LocalizationSensorStatusPanel : public rviz::Panel {
  Q_OBJECT

 public:
  explicit LocalizationSensorStatusPanel(QWidget* parent = nullptr);

  void load(const rviz::Config& config) override;
  void save(rviz::Config config) const override;

 private Q_SLOTS:
  void refreshDisplay();

 private:
  void subscribe();
  void diagnosticsCallback(
      const diagnostic_msgs::DiagnosticArrayConstPtr& message);
  static QString componentLabel(const std::string& name);
  static QString levelLabel(uint8_t level);
  static QColor levelColor(uint8_t level);
  static QString value(const diagnostic_msgs::DiagnosticStatus& status,
                       const std::string& key);
  static QString compactDetails(
      const diagnostic_msgs::DiagnosticStatus& status);

  ros::NodeHandle node_;
  ros::Subscriber diagnostics_subscriber_;
  std::string diagnostics_topic_{"/localization/live/diagnostics"};

  QLabel* overall_label_{nullptr};
  QLabel* topic_label_{nullptr};
  QTreeWidget* status_tree_{nullptr};
  QTimer* refresh_timer_{nullptr};

  mutable std::mutex mutex_;
  diagnostic_msgs::DiagnosticArray diagnostics_;
  ros::WallTime last_diagnostics_wall_time_;
  bool have_diagnostics_{false};
  double stale_display_sec_{1.5};
};

}  // namespace localization_rviz
