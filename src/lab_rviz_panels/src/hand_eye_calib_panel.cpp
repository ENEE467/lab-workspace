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
  ui_->statusLabel->setText("Frames Captured");
  ui_->framesCapturedProgressBar->setDisabled(false);
  ui_->captureFrameButton->setDisabled(false);
  ui_->startOverButton->setDisabled(false);
  ui_->calibrateButton->setDisabled(false);
  ui_->saveButton->setDisabled(false);

  service_request_->set__action(lab7::srv::HandEyeCalib::Request::STATUS);
  auto& response {sendServiceRequest(service_request_)};
  updatePanel(response);
}

void HandEyeCalibPanel::disablePanel()
{
  ui_->statusLabel->setText("Service Unavailable!");
  ui_->framesCapturedProgressBar->setDisabled(true);
  ui_->captureFrameButton->setDisabled(true);
  ui_->startOverButton->setDisabled(true);
  ui_->calibrateButton->setDisabled(true);
  ui_->saveButton->setDisabled(true);
}

void HandEyeCalibPanel::updatePanel(const lab7::srv::HandEyeCalib::Response::SharedPtr& response)
{
  ui_->framesCapturedProgressBar->setMaximum(response->measurements_required);
  ui_->framesCapturedProgressBar->setValue(response->measurements_captured);
}

void HandEyeCalibPanel::checkServiceAvailability()
{
  if (service_available_)
    return;

  service_available_ = hand_eye_calib_client_->wait_for_service(std::chrono::seconds(2));

  if (service_available_)
    enablePanel();
  else
    disablePanel();
}

const lab7::srv::HandEyeCalib::Response::SharedPtr& HandEyeCalibPanel::sendServiceRequest(
  const lab7::srv::HandEyeCalib::Request::SharedPtr& request)
{
  // if (!service_available_) {
  //   RCLCPP_WARN(node_->get_logger(), "Request failed because the service is unavailable.");

  //   return;
  // }

  auto result_future {hand_eye_calib_client_->async_send_request(request).share()};

  RCLCPP_INFO(node_->get_logger(), "Command sent, waiting for result...");
  auto result_status {result_future.wait_for(std::chrono::seconds(1))};

  auto& response {result_future.get()};

  if (response->success && result_status == std::future_status::ready)
    RCLCPP_INFO(node_->get_logger(), "Command request fulfilled!");
  else
    RCLCPP_INFO(node_->get_logger(), "Command request failed to execute! Try again.");

  return response;
}

void HandEyeCalibPanel::captureFrame()
{
  service_request_->set__action(lab7::srv::HandEyeCalib::Request::CAPTURE);
  auto& response {sendServiceRequest(service_request_)};
  updatePanel(response);
}

void HandEyeCalibPanel::startOver()
{
  service_request_->set__action(lab7::srv::HandEyeCalib::Request::RESET);
  auto& response {sendServiceRequest(service_request_)};
  updatePanel(response);
}

void HandEyeCalibPanel::calibrate()
{
  service_request_->set__action(lab7::srv::HandEyeCalib::Request::CALIBRATE);
  auto& response {sendServiceRequest(service_request_)};
  updatePanel(response);
}

void HandEyeCalibPanel::save()
{
  service_request_->set__action(lab7::srv::HandEyeCalib::Request::SAVE);
  auto& response {sendServiceRequest(service_request_)};
  updatePanel(response);
}

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(lab7::HandEyeCalibPanel, rviz_common::Panel)
