#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include <QtWidgets>
#include <ui_hand_eye_calib_panel.h>

#include <lab7/srv/hand_eye_calib.hpp>

namespace lab7 {

class HandEyeCalibPanel : public rviz_common::Panel {

Q_OBJECT
public:
  HandEyeCalibPanel(QWidget * parent = nullptr);
  ~HandEyeCalibPanel();

  void onInitialize() override;
  void load(const rviz_common::Config &config) override;
  void save(rviz_common::Config config) const override;

public Q_SLOTS:
  void captureFrame();
  void startOver();
  void calibrate();
  void save();

private:
  void enablePanel();
  void disablePanel();
  void updatePanel(const lab7::srv::HandEyeCalib::Response::SharedPtr& response);
  void checkServiceAvailability();

  const lab7::srv::HandEyeCalib::Response::SharedPtr& sendServiceRequest(
    const lab7::srv::HandEyeCalib::Request::SharedPtr& request);

  bool service_available_ {false};

  std::unique_ptr<Ui::HandEyeCalibPanel> ui_ {nullptr};

  rclcpp::Node::SharedPtr node_ {nullptr};

  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_ {nullptr};
  std::unique_ptr<std::thread> spin_thread_ {nullptr};

  rclcpp::CallbackGroup::SharedPtr timer_cb_group_ {nullptr};
  rclcpp::CallbackGroup::SharedPtr client_cb_group_ {nullptr};

  rclcpp::TimerBase::SharedPtr timer_ {nullptr};

  rclcpp::Client<lab7::srv::HandEyeCalib>::SharedPtr hand_eye_calib_client_ {nullptr};
  lab7::srv::HandEyeCalib::Request::SharedPtr service_request_ {nullptr};

};

}
