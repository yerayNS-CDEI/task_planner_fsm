#include "task_planner_fsm_rviz_panel/fsm_panel.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include <QDateTime>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTextDocument>
#include <QTextEdit>
#include <QVBoxLayout>

namespace task_planner_fsm_rviz_panel
{

FsmPanel::FsmPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  createUi();

  connect(
    this, &FsmPanel::currentStateReceived, this, &FsmPanel::handleCurrentState,
    Qt::QueuedConnection);
  connect(
    this, &FsmPanel::transitionReceived, this, &FsmPanel::handleTransition,
    Qt::QueuedConnection);
}

void FsmPanel::onInitialize()
{
  auto ros_node_abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!ros_node_abstraction) {
    return;
  }

  raw_node_ = ros_node_abstraction->get_raw_node();
  createSubscriptions();
}

void FsmPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("current_topic", current_topic_);
  config.mapSetValue("transition_topic", transition_topic_);
}

void FsmPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);

  QString saved_topic;
  if (config.mapGetString("current_topic", &saved_topic) && !saved_topic.isEmpty()) {
    current_topic_ = saved_topic;
  }
  if (config.mapGetString("transition_topic", &saved_topic) && !saved_topic.isEmpty()) {
    transition_topic_ = saved_topic;
  }

  current_topic_edit_->setText(current_topic_);
  transition_topic_edit_->setText(transition_topic_);
  createSubscriptions();
}

void FsmPanel::createUi()
{
  auto * root_layout = new QVBoxLayout();

  auto * topic_group = new QGroupBox("FSM Topics");
  auto * topic_form = new QFormLayout(topic_group);
  current_topic_edit_ = new QLineEdit(current_topic_);
  transition_topic_edit_ = new QLineEdit(transition_topic_);
  auto * apply_button = new QPushButton("Apply Topics");

  topic_form->addRow("Current state topic:", current_topic_edit_);
  topic_form->addRow("Transition topic:", transition_topic_edit_);
  topic_form->addRow("", apply_button);

  auto * status_group = new QGroupBox("FSM Status");
  auto * status_form = new QFormLayout(status_group);
  current_state_value_ = new QLabel("-");
  last_transition_value_ = new QLabel("-");
  status_form->addRow("Current state:", current_state_value_);
  status_form->addRow("Last transition:", last_transition_value_);

  auto * log_group = new QGroupBox("Transition Log");
  auto * log_layout = new QVBoxLayout(log_group);
  transition_log_ = new QTextEdit();
  transition_log_->setReadOnly(true);
  transition_log_->document()->setMaximumBlockCount(300);
  log_layout->addWidget(transition_log_);

  root_layout->addWidget(topic_group);
  root_layout->addWidget(status_group);
  root_layout->addWidget(log_group);
  setLayout(root_layout);

  connect(apply_button, &QPushButton::clicked, this, &FsmPanel::updateTopicsFromUi);
  connect(current_topic_edit_, &QLineEdit::returnPressed, this, &FsmPanel::updateTopicsFromUi);
  connect(transition_topic_edit_, &QLineEdit::returnPressed, this, &FsmPanel::updateTopicsFromUi);
}

void FsmPanel::createSubscriptions()
{
  current_state_sub_.reset();
  transition_sub_.reset();

  if (!raw_node_) {
    return;
  }

  auto current_qos = rclcpp::QoS(rclcpp::KeepLast(1));
  current_qos.reliable();
  current_qos.transient_local();
  auto transition_qos = rclcpp::QoS(rclcpp::KeepLast(50));
  transition_qos.reliable();

  current_state_sub_ = raw_node_->create_subscription<StringMsg>(
    current_topic_.toStdString(),
    current_qos,
    [this](const StringMsg::SharedPtr msg) {
      Q_EMIT currentStateReceived(QString::fromStdString(msg->data));
    });

  transition_sub_ = raw_node_->create_subscription<StringMsg>(
    transition_topic_.toStdString(),
    transition_qos,
    [this](const StringMsg::SharedPtr msg) {
      QString from;
      QString to;
      QString reason;
      parseTransitionPayload(msg->data, from, to, reason);
      Q_EMIT transitionReceived(from, to, reason);
    });
}

void FsmPanel::parseTransitionPayload(
  const std::string & payload, QString & from, QString & to, QString & reason)
{
  QJsonParseError parse_error;
  const auto json_doc =
    QJsonDocument::fromJson(QByteArray::fromStdString(payload), &parse_error);
  if (parse_error.error == QJsonParseError::NoError && json_doc.isObject()) {
    const auto obj = json_doc.object();
    from = obj.value("from").toString();
    to = obj.value("to").toString();
    reason = obj.value("reason").toString();
    return;
  }

  const QString raw = QString::fromStdString(payload).trimmed();
  if (raw.contains("->")) {
    const auto parts = raw.split("->");
    if (parts.size() >= 2) {
      from = parts.front().trimmed();
      to = parts.back().trimmed();
      reason.clear();
      return;
    }
  }

  reason = raw;
}

void FsmPanel::handleCurrentState(const QString & state)
{
  current_state_ = state.trimmed();
  current_state_value_->setText(current_state_.isEmpty() ? "-" : current_state_);
}

void FsmPanel::handleTransition(const QString & from, const QString & to, const QString & reason)
{
  QString from_text = from.trimmed();
  QString to_text = to.trimmed();

  if (from_text.isEmpty()) {
    from_text = "?";
  }
  if (to_text.isEmpty()) {
    to_text = "?";
  }

  QString transition_text = QString("%1 -> %2").arg(from_text, to_text);
  if (!reason.trimmed().isEmpty()) {
    transition_text += QString(" [%1]").arg(reason.trimmed());
  }

  last_transition_value_->setText(transition_text);

  const QString stamp = QDateTime::currentDateTime().toString("HH:mm:ss");
  transition_log_->append(QString("[%1] %2").arg(stamp, transition_text));
}

void FsmPanel::updateTopicsFromUi()
{
  const QString current_topic_new = current_topic_edit_->text().trimmed();
  const QString transition_topic_new = transition_topic_edit_->text().trimmed();

  if (!current_topic_new.isEmpty() && current_topic_new != current_topic_) {
    current_topic_ = current_topic_new;
  }
  if (!transition_topic_new.isEmpty() && transition_topic_new != transition_topic_) {
    transition_topic_ = transition_topic_new;
  }

  createSubscriptions();
  configChanged();

  const QString stamp = QDateTime::currentDateTime().toString("HH:mm:ss");
  transition_log_->append(
    QString("[%1] Resubscribed to topics: current='%2', transition='%3'")
      .arg(stamp, current_topic_, transition_topic_));
}

}  // namespace task_planner_fsm_rviz_panel

PLUGINLIB_EXPORT_CLASS(task_planner_fsm_rviz_panel::FsmPanel, rviz_common::Panel)
