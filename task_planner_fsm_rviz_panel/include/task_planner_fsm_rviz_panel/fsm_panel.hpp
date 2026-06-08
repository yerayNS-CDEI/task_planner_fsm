#ifndef TASK_PLANNER_FSM_RVIZ_PANEL__FSM_PANEL_HPP_
#define TASK_PLANNER_FSM_RVIZ_PANEL__FSM_PANEL_HPP_

#include <vector>

#include <QHash>
#include <QString>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/string.hpp>

class QLabel;
class QGraphicsEllipseItem;
class QGraphicsPathItem;
class QGraphicsPolygonItem;
class QGraphicsScene;
class QGraphicsView;

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
  void graphPayloadReceived(const QString & payload);
  void statusPayloadReceived(const QString & payload);
  void eventPayloadReceived(const QString & payload);

private Q_SLOTS:
  void handleCurrentState(const QString & state);
  void handleTransition(const QString & from, const QString & to, const QString & reason);
  void handleGraphPayload(const QString & payload);
  void handleStatusPayload(const QString & payload);
  void handleEventPayload(const QString & payload);

private:
  struct EdgeItem
  {
    QString from;
    QString to;
    QGraphicsPathItem * item{nullptr};
    QGraphicsPolygonItem * arrow{nullptr};
  };

  using StringMsg = std_msgs::msg::String;

  void createUi();
  void createSubscriptions();
  void appendLogLine(const QString & line);
  void refreshGraphStyles();
  static void parseTransitionPayload(
    const std::string & payload, QString & from, QString & to, QString & reason);

  QString current_topic_{"/fsm/current_state"};
  QString transition_topic_{"/fsm/transition"};
  QString graph_topic_{"/fsm/graph"};
  QString status_topic_{"/fsm/status"};
  QString event_topic_{"/fsm/event"};
  QString current_state_{};
  QString last_transition_from_{};
  QString last_transition_to_{};
  QString last_transition_reason_{};

  QLabel * current_state_value_{nullptr};
  QLabel * last_transition_value_{nullptr};
  QLabel * status_phase_value_{nullptr};
  QLabel * status_summary_value_{nullptr};
  QLabel * elapsed_value_{nullptr};
  QLabel * progress_value_{nullptr};
  QGraphicsScene * graph_scene_{nullptr};
  QGraphicsView * graph_view_{nullptr};

  QHash<QString, QGraphicsEllipseItem *> node_items_{};
  std::vector<EdgeItem> edge_items_{};

  rclcpp::Node::SharedPtr raw_node_{};
  rclcpp::Subscription<StringMsg>::SharedPtr current_state_sub_{};
  rclcpp::Subscription<StringMsg>::SharedPtr transition_sub_{};
  rclcpp::Subscription<StringMsg>::SharedPtr graph_sub_{};
  rclcpp::Subscription<StringMsg>::SharedPtr status_sub_{};
  rclcpp::Subscription<StringMsg>::SharedPtr event_sub_{};
};

}  // namespace task_planner_fsm_rviz_panel

#endif  // TASK_PLANNER_FSM_RVIZ_PANEL__FSM_PANEL_HPP_
