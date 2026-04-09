#include "task_planner_fsm_rviz_panel/fsm_panel.hpp"

#include <cmath>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include <QBrush>
#include <QDateTime>
#include <QFont>
#include <QFormLayout>
#include <QGraphicsEllipseItem>
#include <QGraphicsLineItem>
#include <QGraphicsScene>
#include <QGraphicsSimpleTextItem>
#include <QGraphicsView>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QPainter>
#include <QPen>
#include <QPushButton>
#include <QSizePolicy>
#include <QVBoxLayout>
#include <QWheelEvent>
#include <QWidget>

namespace task_planner_fsm_rviz_panel
{

namespace
{

QColor groupColor(const QString & group)
{
  if (group == "phase_1") {
    return QColor("#DDEAF7");
  }
  if (group == "phase_2") {
    return QColor("#E2F4E7");
  }
  if (group == "terminal") {
    return QColor("#F1E3E3");
  }
  return QColor("#EEE9DD");
}

class ZoomableGraphicsView : public QGraphicsView
{
public:
  explicit ZoomableGraphicsView(QGraphicsScene * scene, QWidget * parent = nullptr)
  : QGraphicsView(scene, parent)
  {
    setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
    setResizeAnchor(QGraphicsView::AnchorUnderMouse);
  }

protected:
  void wheelEvent(QWheelEvent * event) override
  {
    if (event->angleDelta().y() == 0) {
      QGraphicsView::wheelEvent(event);
      return;
    }

    const qreal factor = event->angleDelta().y() > 0 ? 1.15 : (1.0 / 1.15);
    scale(factor, factor);
    event->accept();
  }
};

QPointF ellipseBoundaryPoint(
  const QPointF & center, qreal rx, qreal ry, const QPointF & target)
{
  const qreal dx = target.x() - center.x();
  const qreal dy = target.y() - center.y();
  const qreal denom = std::sqrt((dx * dx) / (rx * rx) + (dy * dy) / (ry * ry));
  if (denom < 1e-6) {
    return center;
  }
  return QPointF(center.x() + dx / denom, center.y() + dy / denom);
}

}  // namespace

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
  connect(
    this, &FsmPanel::graphPayloadReceived, this, &FsmPanel::handleGraphPayload,
    Qt::QueuedConnection);
  connect(
    this, &FsmPanel::statusPayloadReceived, this, &FsmPanel::handleStatusPayload,
    Qt::QueuedConnection);
  connect(
    this, &FsmPanel::eventPayloadReceived, this, &FsmPanel::handleEventPayload,
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
  config.mapSetValue("graph_topic", graph_topic_);
  config.mapSetValue("status_topic", status_topic_);
  config.mapSetValue("event_topic", event_topic_);
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
  if (config.mapGetString("graph_topic", &saved_topic) && !saved_topic.isEmpty()) {
    graph_topic_ = saved_topic;
  }
  if (config.mapGetString("status_topic", &saved_topic) && !saved_topic.isEmpty()) {
    status_topic_ = saved_topic;
  }
  if (config.mapGetString("event_topic", &saved_topic) && !saved_topic.isEmpty()) {
    event_topic_ = saved_topic;
  }

  createSubscriptions();
}

void FsmPanel::createUi()
{
  auto * root_layout = new QVBoxLayout();

  auto * status_group = new QGroupBox("FSM Status");
  auto * status_form = new QFormLayout(status_group);
  current_state_value_ = new QLabel("-");
  last_transition_value_ = new QLabel("-");
  status_phase_value_ = new QLabel("-");
  status_summary_value_ = new QLabel("-");
  status_summary_value_->setWordWrap(true);
  elapsed_value_ = new QLabel("-");
  progress_value_ = new QLabel("-");
  status_form->addRow("Current state:", current_state_value_);
  status_form->addRow("Last transition:", last_transition_value_);
  status_form->addRow("Phase:", status_phase_value_);
  status_form->addRow("Summary:", status_summary_value_);
  status_form->addRow("Elapsed:", elapsed_value_);
  status_form->addRow("Progress:", progress_value_);

  auto * graph_group = new QGroupBox("FSM Graph");
  auto * graph_layout = new QVBoxLayout(graph_group);
  graph_scene_ = new QGraphicsScene(this);
  auto * zoom_row = new QHBoxLayout();
  auto * zoom_hint = new QLabel("Mouse wheel to zoom");
  auto * zoom_out_button = new QPushButton("-");
  auto * zoom_reset_button = new QPushButton("Reset");
  auto * zoom_in_button = new QPushButton("+");
  graph_view_ = new ZoomableGraphicsView(graph_scene_);
  graph_view_->setRenderHint(QPainter::Antialiasing, true);
  graph_view_->setDragMode(QGraphicsView::ScrollHandDrag);
  graph_view_->setMinimumSize(360, 420);
  graph_view_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  graph_group->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  zoom_row->addWidget(zoom_hint);
  zoom_row->addStretch(1);
  zoom_row->addWidget(zoom_out_button);
  zoom_row->addWidget(zoom_reset_button);
  zoom_row->addWidget(zoom_in_button);
  graph_layout->addLayout(zoom_row);
  graph_layout->addWidget(graph_view_);

  root_layout->addWidget(status_group);
  root_layout->addWidget(graph_group, 1);
  setLayout(root_layout);

  connect(zoom_in_button, &QPushButton::clicked, this, [this]() {
    graph_view_->scale(1.15, 1.15);
  });
  connect(zoom_out_button, &QPushButton::clicked, this, [this]() {
    graph_view_->scale(1.0 / 1.15, 1.0 / 1.15);
  });
  connect(zoom_reset_button, &QPushButton::clicked, this, [this]() {
    if (!graph_scene_) {
      return;
    }
    graph_view_->resetTransform();
    graph_view_->fitInView(graph_scene_->sceneRect(), Qt::KeepAspectRatio);
  });
}

void FsmPanel::createSubscriptions()
{
  current_state_sub_.reset();
  transition_sub_.reset();
  graph_sub_.reset();
  status_sub_.reset();
  event_sub_.reset();

  if (!raw_node_) {
    return;
  }

  auto current_qos = rclcpp::QoS(rclcpp::KeepLast(1));
  current_qos.reliable();
  current_qos.transient_local();

  auto transition_qos = rclcpp::QoS(rclcpp::KeepLast(100));
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

  graph_sub_ = raw_node_->create_subscription<StringMsg>(
    graph_topic_.toStdString(),
    current_qos,
    [this](const StringMsg::SharedPtr msg) {
      Q_EMIT graphPayloadReceived(QString::fromStdString(msg->data));
    });

  status_sub_ = raw_node_->create_subscription<StringMsg>(
    status_topic_.toStdString(),
    current_qos,
    [this](const StringMsg::SharedPtr msg) {
      Q_EMIT statusPayloadReceived(QString::fromStdString(msg->data));
    });

  event_sub_ = raw_node_->create_subscription<StringMsg>(
    event_topic_.toStdString(),
    transition_qos,
    [this](const StringMsg::SharedPtr msg) {
      Q_EMIT eventPayloadReceived(QString::fromStdString(msg->data));
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

void FsmPanel::appendLogLine(const QString & line)
{
  Q_UNUSED(line);
}

void FsmPanel::refreshGraphStyles()
{
  for (auto it = node_items_.cbegin(); it != node_items_.cend(); ++it) {
    const QString & state_id = it.key();
    auto * item = it.value();
    QColor fill = groupColor(item->data(1).toString());
    QColor border("#56616F");
    qreal pen_width = 1.5;

    if (state_id == current_state_) {
      fill = QColor("#84D39A");
      border = QColor("#1E6F43");
      pen_width = 3.0;
    } else if (state_id == "Error" && current_state_ == "Error") {
      fill = QColor("#F3B4B4");
      border = QColor("#9D2323");
      pen_width = 3.0;
    } else if (state_id == last_transition_from_ || state_id == last_transition_to_) {
      fill = QColor("#F6E2A1");
      border = QColor("#8C6A12");
      pen_width = 2.0;
    }

    item->setBrush(QBrush(fill));
    item->setPen(QPen(border, pen_width));
  }

  for (auto & edge : edge_items_) {
    QColor color("#A0A8B2");
    qreal width = 1.5;
    if (edge.from == last_transition_from_ && edge.to == last_transition_to_) {
      color = QColor("#1E6F43");
      width = 3.0;
    }
    edge.item->setPen(QPen(color, width));
  }
}

void FsmPanel::handleCurrentState(const QString & state)
{
  current_state_ = state.trimmed();
  current_state_value_->setText(current_state_.isEmpty() ? "-" : current_state_);
  refreshGraphStyles();
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

  last_transition_from_ = from_text;
  last_transition_to_ = to_text;
  last_transition_reason_ = reason.trimmed();

  QString transition_text = QString("%1 -> %2").arg(from_text, to_text);
  if (!last_transition_reason_.isEmpty()) {
    transition_text += QString(" [%1]").arg(last_transition_reason_);
  }

  last_transition_value_->setText(transition_text);
  appendLogLine(transition_text);
  refreshGraphStyles();
}

void FsmPanel::handleGraphPayload(const QString & payload)
{
  QJsonParseError parse_error;
  const auto json_doc = QJsonDocument::fromJson(payload.toUtf8(), &parse_error);
  if (parse_error.error != QJsonParseError::NoError || !json_doc.isObject()) {
    appendLogLine(QString("Ignoring invalid /fsm/graph payload: %1").arg(parse_error.errorString()));
    return;
  }

  graph_scene_->clear();
  node_items_.clear();
  edge_items_.clear();

  constexpr qreal kNodeWidth = 220.0;
  constexpr qreal kNodeHeight = 92.0;
  constexpr qreal kRadiusX = kNodeWidth / 2.0;
  constexpr qreal kRadiusY = kNodeHeight / 2.0;

  const auto root = json_doc.object();
  const auto states = root.value("states").toArray();
  for (const auto & state_value : states) {
    const auto state_obj = state_value.toObject();
    const QString id = state_obj.value("id").toString();
    if (id.isEmpty()) {
      continue;
    }

    const QString label = state_obj.value("label").toString(id);
    const QString group = state_obj.value("group").toString();
    const qreal x = state_obj.value("x").toDouble();
    const qreal y = state_obj.value("y").toDouble();

    auto * rect = graph_scene_->addEllipse(
      QRectF(x, y, kNodeWidth, kNodeHeight),
      QPen(QColor("#56616F"), 1.5),
      QBrush(groupColor(group)));
    rect->setData(0, id);
    rect->setData(1, group);
    rect->setZValue(1.0);

    auto * text = graph_scene_->addSimpleText(label, QFont("Sans Serif", 12, QFont::Bold));
    text->setBrush(QBrush(Qt::black));
    const QRectF text_bounds = text->boundingRect();
    text->setPos(
      x + (kNodeWidth - text_bounds.width()) / 2.0,
      y + (kNodeHeight - text_bounds.height()) / 2.0);
    text->setZValue(2.0);

    node_items_.insert(id, rect);
  }

  const auto edges = root.value("edges").toArray();
  for (const auto & edge_value : edges) {
    const auto edge_obj = edge_value.toObject();
    const QString from = edge_obj.value("from").toString();
    const QString to = edge_obj.value("to").toString();
    if (!node_items_.contains(from) || !node_items_.contains(to)) {
      continue;
    }

    const QPointF from_center = node_items_.value(from)->rect().center();
    const QPointF to_center = node_items_.value(to)->rect().center();
    const QPointF start = ellipseBoundaryPoint(from_center, kRadiusX, kRadiusY, to_center);
    const QPointF end = ellipseBoundaryPoint(to_center, kRadiusX, kRadiusY, from_center);
    const QLineF line(start, end);

    auto * line_item = graph_scene_->addLine(line, QPen(QColor("#A0A8B2"), 1.5));
    line_item->setZValue(0.0);
    edge_items_.push_back({from, to, line_item});
  }

  refreshGraphStyles();
  graph_scene_->setSceneRect(graph_scene_->itemsBoundingRect().adjusted(-80.0, -80.0, 80.0, 80.0));
  graph_view_->resetTransform();
  graph_view_->fitInView(graph_scene_->sceneRect(), Qt::KeepAspectRatio);
}

void FsmPanel::handleStatusPayload(const QString & payload)
{
  QJsonParseError parse_error;
  const auto json_doc = QJsonDocument::fromJson(payload.toUtf8(), &parse_error);
  if (parse_error.error != QJsonParseError::NoError || !json_doc.isObject()) {
    appendLogLine(QString("Ignoring invalid /fsm/status payload: %1").arg(parse_error.errorString()));
    return;
  }

  const auto obj = json_doc.object();
  const QString state = obj.value("state").toString();
  if (!state.isEmpty()) {
    current_state_ = state;
    current_state_value_->setText(current_state_);
  }

  const QString phase = obj.value("phase").toString("-");
  const QString summary = obj.value("summary").toString("-");
  status_phase_value_->setText(phase);
  status_summary_value_->setText(summary);

  const auto elapsed_value = obj.value("elapsed_s");
  if (elapsed_value.isDouble()) {
    elapsed_value_->setText(QString("%1 s").arg(elapsed_value.toDouble(), 0, 'f', 1));
  } else {
    elapsed_value_->setText("-");
  }

  const auto progress_obj = obj.value("progress").toObject();
  if (!progress_obj.isEmpty()) {
    const QString current = progress_obj.value("current").toVariant().toString();
    const QString total = progress_obj.value("total").toVariant().toString();
    progress_value_->setText(
      current.isEmpty() && total.isEmpty() ? "-" : QString("%1 / %2").arg(current, total));
  } else {
    progress_value_->setText("-");
  }

  refreshGraphStyles();
}

void FsmPanel::handleEventPayload(const QString & payload)
{
  QJsonParseError parse_error;
  const auto json_doc = QJsonDocument::fromJson(payload.toUtf8(), &parse_error);
  if (parse_error.error != QJsonParseError::NoError || !json_doc.isObject()) {
    appendLogLine(QString("Ignoring invalid /fsm/event payload: %1").arg(parse_error.errorString()));
    return;
  }

  const auto obj = json_doc.object();
  const QString state = obj.value("state").toString("?");
  const QString event = obj.value("event").toString("event");
  const QString summary = obj.value("summary").toString(event);
  const QString level = obj.value("level").toString("info").toUpper();
  appendLogLine(QString("%1 %2: %3").arg(level, state, summary));
}

}  // namespace task_planner_fsm_rviz_panel

PLUGINLIB_EXPORT_CLASS(task_planner_fsm_rviz_panel::FsmPanel, rviz_common::Panel)
