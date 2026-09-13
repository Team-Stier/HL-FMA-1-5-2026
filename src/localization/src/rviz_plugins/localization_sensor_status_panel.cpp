#include "localization_sensor_status_panel.hpp"

#include <diagnostic_msgs/DiagnosticStatus.h>
#include <pluginlib/class_list_macros.h>

#include <QBrush>
#include <QColor>
#include <QFont>
#include <QHeaderView>
#include <QTreeWidgetItem>
#include <QVBoxLayout>

#include <algorithm>
#include <utility>

namespace localization_rviz {

LocalizationSensorStatusPanel::LocalizationSensorStatusPanel(QWidget* parent)
    : rviz::Panel(parent) {
  setMinimumWidth(380);
  setMaximumWidth(520);

  auto* layout = new QVBoxLayout();
  overall_label_ = new QLabel(QStringLiteral("전체 상태: 수신 대기"));
  QFont title_font = overall_label_->font();
  title_font.setBold(true);
  title_font.setPointSize(title_font.pointSize() + 1);
  overall_label_->setFont(title_font);

  topic_label_ = new QLabel();
  topic_label_->setStyleSheet(QStringLiteral("QLabel { color: #888888; }"));

  status_tree_ = new QTreeWidget();
  status_tree_->setColumnCount(2);
  status_tree_->setHeaderLabels(
      {QStringLiteral("센서/출력"), QStringLiteral("상태 / 현재 값")});
  status_tree_->header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  status_tree_->header()->setSectionResizeMode(1, QHeaderView::Stretch);
  status_tree_->setAlternatingRowColors(true);
  status_tree_->setRootIsDecorated(false);

  layout->addWidget(overall_label_);
  layout->addWidget(topic_label_);
  layout->addWidget(status_tree_);
  setLayout(layout);

  subscribe();
  refresh_timer_ = new QTimer(this);
  connect(refresh_timer_, &QTimer::timeout, this,
          &LocalizationSensorStatusPanel::refreshDisplay);
  refresh_timer_->start(200);
}

void LocalizationSensorStatusPanel::load(const rviz::Config& config) {
  rviz::Panel::load(config);
  QString topic;
  if (config.mapGetString(QStringLiteral("Diagnostics Topic"), &topic) &&
      !topic.trimmed().isEmpty()) {
    diagnostics_topic_ = topic.trimmed().toStdString();
    subscribe();
  }
}

void LocalizationSensorStatusPanel::save(rviz::Config config) const {
  rviz::Panel::save(config);
  config.mapSetValue(QStringLiteral("Diagnostics Topic"),
                     QString::fromStdString(diagnostics_topic_));
}

void LocalizationSensorStatusPanel::subscribe() {
  diagnostics_subscriber_.shutdown();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    have_diagnostics_ = false;
    last_diagnostics_wall_time_ = ros::WallTime();
  }
  diagnostics_subscriber_ = node_.subscribe(
      diagnostics_topic_, 10,
      &LocalizationSensorStatusPanel::diagnosticsCallback, this);
  if (topic_label_ != nullptr) {
    topic_label_->setText(QStringLiteral("토픽: ") +
                          QString::fromStdString(diagnostics_topic_));
  }
}

void LocalizationSensorStatusPanel::diagnosticsCallback(
    const diagnostic_msgs::DiagnosticArrayConstPtr& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  diagnostics_ = *message;
  have_diagnostics_ = true;
  last_diagnostics_wall_time_ = ros::WallTime::now();
}

QString LocalizationSensorStatusPanel::componentLabel(
    const std::string& name) {
  const std::string prefix = "localization/";
  const std::string key =
      name.compare(0, prefix.size(), prefix) == 0 ? name.substr(prefix.size())
                                                  : name;
  if (key == "gps_fix") return QStringLiteral("GPS Fix");
  if (key == "gps_quality") return QStringLiteral("GPS Quality");
  if (key == "imu") return QStringLiteral("IMU");
  if (key == "encoder") return QStringLiteral("Encoder");
  if (key == "gps_pose") return QStringLiteral("GPS + IMU Pose");
  if (key == "local_odom") return QStringLiteral("Local Odometry");
  if (key == "global_odom") return QStringLiteral("Global Odometry");
  return QString::fromStdString(key);
}

QString LocalizationSensorStatusPanel::levelLabel(const uint8_t level) {
  if (level == diagnostic_msgs::DiagnosticStatus::OK) {
    return QStringLiteral("정상");
  }
  if (level == diagnostic_msgs::DiagnosticStatus::WARN) {
    return QStringLiteral("경고");
  }
  if (level == diagnostic_msgs::DiagnosticStatus::ERROR) {
    return QStringLiteral("오류");
  }
  return QStringLiteral("갱신 지연");
}

QColor LocalizationSensorStatusPanel::levelColor(const uint8_t level) {
  if (level == diagnostic_msgs::DiagnosticStatus::OK) return QColor("#24c45a");
  if (level == diagnostic_msgs::DiagnosticStatus::WARN) return QColor("#f2a900");
  return QColor("#ef3e36");
}

QString LocalizationSensorStatusPanel::value(
    const diagnostic_msgs::DiagnosticStatus& status, const std::string& key) {
  const auto found = std::find_if(
      status.values.begin(), status.values.end(),
      [&key](const diagnostic_msgs::KeyValue& item) { return item.key == key; });
  return found == status.values.end() ? QString()
                                      : QString::fromStdString(found->value);
}

QString LocalizationSensorStatusPanel::compactDetails(
    const diagnostic_msgs::DiagnosticStatus& status) {
  QString details;
  if (status.message == "MISSING") {
    details = QStringLiteral("입력 없음");
  } else if (status.message == "NO OUTPUT") {
    details = QStringLiteral("출력 없음");
  } else if (status.message == "STALE") {
    details = QStringLiteral("수신 지연");
  } else if (status.message != "OK") {
    details = QString::fromStdString(status.message);
  }
  const QString name = QString::fromStdString(status.name);
  const auto append = [&details](const QString& text) {
    if (text.isEmpty()) return;
    if (!details.isEmpty()) details += QStringLiteral(" | ");
    details += text;
  };

  if (name.endsWith(QStringLiteral("gps_quality"))) {
    append(value(status, "status_text"));
    const QString sats = value(status, "satellites");
    if (!sats.isEmpty()) append(QStringLiteral("위성 ") + sats);
    const QString accuracy = value(status, "horizontal_accuracy_m");
    if (!accuracy.isEmpty()) append(QStringLiteral("hAcc ") + accuracy + " m");
  } else if (name.endsWith(QStringLiteral("gps_fix"))) {
    const QString latitude = value(status, "latitude");
    const QString longitude = value(status, "longitude");
    if (!latitude.isEmpty() && !longitude.isEmpty()) {
      append(latitude + QStringLiteral(", ") + longitude);
    }
  } else if (name.endsWith(QStringLiteral("imu"))) {
    const QString yaw = value(status, "yaw_deg");
    if (!yaw.isEmpty()) append(QStringLiteral("Yaw ") + yaw + QChar(0x00b0));
  } else if (name.endsWith(QStringLiteral("encoder"))) {
    const QString count = value(status, "count");
    const QString speed = value(status, "speed_raw");
    if (!count.isEmpty()) append(QStringLiteral("count ") + count);
    if (!speed.isEmpty()) append(QStringLiteral("speed ") + speed);
  } else {
    const QString xy = value(status, "xy_m");
    if (!xy.isEmpty()) append(QStringLiteral("x,y ") + xy + QStringLiteral(" m"));
  }

  const QString age = value(status, "receipt_age_sec");
  if (!age.isEmpty()) append(QStringLiteral("age ") + age);
  return details;
}

void LocalizationSensorStatusPanel::refreshDisplay() {
  diagnostic_msgs::DiagnosticArray diagnostics;
  ros::WallTime received_at;
  bool have_diagnostics = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    diagnostics = diagnostics_;
    received_at = last_diagnostics_wall_time_;
    have_diagnostics = have_diagnostics_;
  }

  status_tree_->clear();
  const bool stale =
      !have_diagnostics || received_at.isZero() ||
      (ros::WallTime::now() - received_at).toSec() > stale_display_sec_;
  if (stale) {
    overall_label_->setText(QStringLiteral("전체 상태: 진단 수신 대기/지연"));
    overall_label_->setStyleSheet(
        QStringLiteral("QLabel { color: #ef3e36; font-weight: bold; }"));
    auto* item = new QTreeWidgetItem(status_tree_);
    item->setText(0, QStringLiteral("Localization Diagnostics"));
    item->setText(1, QStringLiteral("갱신 지연 · DiagnosticArray 미수신"));
    item->setForeground(1, QBrush(levelColor(diagnostic_msgs::DiagnosticStatus::STALE)));
    return;
  }

  uint8_t overall_level = diagnostic_msgs::DiagnosticStatus::OK;
  for (const auto& status : diagnostics.status) {
    const uint8_t status_level = static_cast<uint8_t>(status.level);
    overall_level = std::max(overall_level, status_level);
    auto* item = new QTreeWidgetItem(status_tree_);
    item->setText(0, componentLabel(status.name));
    const QString details = compactDetails(status);
    if (status.message == "MISSING" || status.message == "NO OUTPUT") {
      item->setText(1, details);
    } else {
      item->setText(1, levelLabel(status_level) +
                           (details.isEmpty() ? QString()
                                              : QStringLiteral(" · ") + details));
    }
    item->setForeground(1, QBrush(levelColor(status_level)));
    QFont font = item->font(1);
    font.setBold(true);
    item->setFont(1, font);

    QString tooltip;
    for (const auto& field : status.values) {
      tooltip += QString::fromStdString(field.key) + QStringLiteral(": ") +
                 QString::fromStdString(field.value) + QLatin1Char('\n');
    }
    item->setToolTip(1, tooltip.trimmed());
  }

  overall_label_->setText(QStringLiteral("전체 상태: ") +
                          levelLabel(overall_level));
  overall_label_->setStyleSheet(
      QStringLiteral("QLabel { color: %1; font-weight: bold; }")
          .arg(levelColor(overall_level).name()));
}

}  // namespace localization_rviz

PLUGINLIB_EXPORT_CLASS(localization_rviz::LocalizationSensorStatusPanel,
                       rviz::Panel)
