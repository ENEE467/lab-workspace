#pragma once

#include <rclcpp/rclcpp.hpp>

#include "lab7/srv/capture_measurements.hpp"

class Lab7CommandInterface : public rclcpp::Node {

public:
  Lab7CommandInterface();
  void sendCaptureMeasurementsRequest(const lab7::srv::CaptureMeasurements::Request::SharedPtr& request);
  void stopNode() {rclcpp::shutdown();}

private:
  bool is_capture_measurements_service_available_ {false};

  rclcpp::Client<lab7::srv::CaptureMeasurements>::SharedPtr capture_measurements_client_ {nullptr};

};
