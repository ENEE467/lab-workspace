// #include <rviz_common/display_context.hpp>
#include "lab_rviz_panels/hand_eye_calib_panel.hpp"

namespace lab7 {

HandEyeCalibPanel::HandEyeCalibPanel(QWidget * parent)
: Panel {parent},
  ui_ {std::make_unique<Ui::HandEyeCalibPanel>()}
{
  ui_->setupUi(this);

  connect(ui_->captureFrameButton, SIGNAL(clicked()), this, SLOT(captureFrame()));
  connect(ui_->startOverButton, SIGNAL(clicked()), this, SLOT(startOver()));
  connect(ui_->calibrateButton, SIGNAL(clicked()), this, SLOT(calibrate()));
  connect(ui_->saveButton, SIGNAL(clicked()), this, SLOT(save()));

  connect(this, SIGNAL(panelDisabledChanged(bool)), ui_->framesCapturedProgressBar, SLOT(setDisabled(bool)));
  connect(this, SIGNAL(panelDisabledChanged(bool)), ui_->captureFrameButton, SLOT(setDisabled(bool)));
  connect(this, SIGNAL(panelDisabledChanged(bool)), ui_->startOverButton, SLOT(setDisabled(bool)));
  connect(this, SIGNAL(panelDisabledChanged(bool)), ui_->calibrateButton, SLOT(setDisabled(bool)));
  connect(this, SIGNAL(panelDisabledChanged(bool)), ui_->saveButton, SLOT(setDisabled(bool)));

  connect(this, SIGNAL(statusLabelChanged(QString)), ui_->statusLabel, SLOT(setText(QString)));
  connect(this, SIGNAL(framesCapturedValueChanged(int)), ui_->framesCapturedProgressBar, SLOT(setValue(int)));
  connect(this, SIGNAL(maxFramesCapturedChanged(int)), ui_->framesCapturedProgressBar, SLOT(setMaximum(int)));
}

HandEyeCalibPanel::~HandEyeCalibPanel()
{
  spin_thread_->join();
  executor_->cancel();
}

void HandEyeCalibPanel::onInitialize()
{
  // auto node_ptr {getDisplayContext()->getRosNodeAbstraction().lock()};
  // node_ = node_ptr->get_raw_node();

  node_ = std::make_shared<rclcpp::Node>("hand_eye_calib_panel_node");

  timer_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  client_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  timer_ =
    node_->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&HandEyeCalibPanel::checkServiceAvailability, this),
      timer_cb_group_);

  hand_eye_calib_client_ =
    node_->create_client<lab7::srv::HandEyeCalib>(
      "hand_eye_calib",
      rmw_qos_profile_services_default,
      client_cb_group_);

  service_request_ = std::make_shared<lab7::srv::HandEyeCalib::Request>();

  executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor_->add_node(node_);
  spin_thread_ = std::make_unique<std::thread>( [this](){ executor_->spin(); } );

  service_available_ = hand_eye_calib_client_->wait_for_service(std::chrono::seconds(1));

  if (service_available_)
    enablePanel();
  else
    disablePanel();

  RCLCPP_INFO(node_->get_logger(), "Panel initialized!");
}

void HandEyeCalibPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
}

void HandEyeCalibPanel::load(const rviz_common::Config &config)
{
  rviz_common::Panel::load(config);
}

void HandEyeCalibPanel::enablePanel()
{
  Q_EMIT statusLabelChanged("Frames Captured");
  Q_EMIT panelDisabledChanged(false);

  service_request_->set__action(lab7::srv::HandEyeCalib::Request::STATUS);
  sendServiceRequest(service_request_);
}

void HandEyeCalibPanel::disablePanel()
{
  Q_EMIT statusLabelChanged("Service Unavailable!");
  Q_EMIT panelDisabledChanged(true);
}

void HandEyeCalibPanel::updatePanel(const lab7::srv::HandEyeCalib::Response::SharedPtr response)
{
  if (ui_->framesCapturedProgressBar->value() != response->measurements_captured)
    Q_EMIT framesCapturedValueChanged(response->measurements_captured);

  if (ui_->framesCapturedProgressBar->maximum() != response->measurements_required)
    Q_EMIT maxFramesCapturedChanged(response->measurements_required);
}

void HandEyeCalibPanel::checkServiceAvailability()
{
  service_available_ = hand_eye_calib_client_->wait_for_service(std::chrono::seconds(2));

  if (service_available_)
    enablePanel();
  else
    disablePanel();
}

void HandEyeCalibPanel::sendServiceRequest(
  const lab7::srv::HandEyeCalib::Request::SharedPtr& request)
{
  // if (!service_available_) {
  //   RCLCPP_WARN(node_->get_logger(), "Request failed because the service is unavailable.");

  //   return;
  // }

  auto result_future {hand_eye_calib_client_->async_send_request(request).share()};

  // RCLCPP_INFO(node_->get_logger(), "Command sent, waiting for result...");

  // auto result_status {result_future.wait_for(std::chrono::seconds(1))};
  result_future.wait_for(std::chrono::seconds(1));

  auto response {result_future.get()};
  updatePanel(response);

  // if (response->success && result_status == std::future_status::ready)
  //   RCLCPP_INFO(node_->get_logger(), "Command request fulfilled!");
  // else
  //   RCLCPP_INFO(node_->get_logger(), "Command request failed to execute! Try again.");
}

void HandEyeCalibPanel::captureFrame()
{
  service_request_->set__action(lab7::srv::HandEyeCalib::Request::CAPTURE);
  sendServiceRequest(service_request_);
}

void HandEyeCalibPanel::startOver()
{
  service_request_->set__action(lab7::srv::HandEyeCalib::Request::RESET);
  sendServiceRequest(service_request_);
}

void HandEyeCalibPanel::calibrate()
{
  service_request_->set__action(lab7::srv::HandEyeCalib::Request::CALIBRATE);
  sendServiceRequest(service_request_);
}

void HandEyeCalibPanel::save()
{
  service_request_->set__action(lab7::srv::HandEyeCalib::Request::SAVE);
  sendServiceRequest(service_request_);
}

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(lab7::HandEyeCalibPanel, rviz_common::Panel)
