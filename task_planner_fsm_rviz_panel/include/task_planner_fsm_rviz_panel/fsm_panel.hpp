#ifndef TASK_PLANNER_FSM_RVIZ_PANEL__FSM_PANEL_HPP_
#define TASK_PLANNER_FSM_RVIZ_PANEL__FSM_PANEL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/string.hpp>

#include <QString>

class QLabel;
class QLineEdit;
class QTextEdit;

namespace task_planner_fsm_rviz_panel
{

class FsmPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit FsmPanel(QWidget * parent = nullptr);

  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config & config) override;

Q_SIGNALS:
  void currentStateReceived(const QString & state);
  void transitionReceived(const QString & from, const QString & to, const QString & reason);

private Q_SLOTS:
  void handleCurrentState(const QString & state);
  void handleTransition(const QString & from, const QString & to, const QString & reason);
  void updateTopicsFromUi();

private:
  using StringMsg = std_msgs::msg::String;

  void createUi();
  void createSubscriptions();
  static void parseTransitionPayload(
    const std::string & payload, QString & from, QString & to, QString & reason);

  QString current_topic_{"/fsm/current_state"};
  QString transition_topic_{"/fsm/transition"};
  QString current_state_{};

  QLabel * current_state_value_{nullptr};
  QLabel * last_transition_value_{nullptr};
  QLineEdit * current_topic_edit_{nullptr};
  QLineEdit * transition_topic_edit_{nullptr};
  QTextEdit * transition_log_{nullptr};

  rclcpp::Node::SharedPtr raw_node_{};
  rclcpp::Subscription<StringMsg>::SharedPtr current_state_sub_{};
  rclcpp::Subscription<StringMsg>::SharedPtr transition_sub_{};
};

}  // namespace task_planner_fsm_rviz_panel

#endif  // TASK_PLANNER_FSM_RVIZ_PANEL__FSM_PANEL_HPP_
