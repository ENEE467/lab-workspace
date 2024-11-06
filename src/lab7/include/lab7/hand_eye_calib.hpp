#pragma once

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <aruco_opencv_msgs/msg/aruco_detection.hpp>

#include <Eigen/Dense>

#include <opencv2/core/eigen.hpp>

#include "lab7/srv/capture_measurements.hpp"
#include "lab7/msg/status.hpp"
#include "lab7/measurements.hpp"


class HandEyeCalibNode : public rclcpp::Node {

public:
  HandEyeCalibNode();

private:
  void generalTimerCallback();
  void measurementsCaptureTimerCallback();

  void captureMeasurementsServiceCallback(
    const std::shared_ptr<lab7::srv::CaptureMeasurements::Request> request,
    std::shared_ptr<lab7::srv::CaptureMeasurements::Response> response);

  void arucoDetectionsSubscriptionCallback(const aruco_opencv_msgs::msg::ArucoDetection& msg);

  // Naming convention for these functions: getPose<Object>_<Reference Frame>

  bool getPoseEndEffector_RobotBase(Eigen::Affine3d& pose_out);
  bool getPoseMarker_Camera(Eigen::Affine3d& pose_out);
  bool getPoseMarker_RobotBase(Eigen::Affine3d& pose_out);

  bool captureCalibrationMeasurements(Measurements::Calibration& measurements_out);
  bool captureCalibrationCheckMeasurements(Measurements::CalibrationCheck& measurements_out);
  bool calibrateHandEye(const Measurements::Calibration& measurements_in, Eigen::Affine3d& pose_out);
  // void verifyCalibration();
  void broadcastFrameCamera_RobotBase(const Eigen::Affine3d& pose_in);
  void saveCalibrationOutput(std::ofstream& txt_file_out, const Eigen::Affine3d& camera_pose_in);
  // void saveVerificationOutput();

  void resetMeasurements();

  std::string createTimeStamp();

  // bool is_base2eef_frame_available_ {false};
  // bool is_cam2eef_frame_available_ {false};
  bool should_capture_measurements_ {false};
  bool captured_calibration_measurements_ {false};
  bool captured_calibration_check_measurements_ {false};
  bool is_calibration_complete_ {false};
  bool saved_calibration_check_measurements_ {false};
  // bool is_verification_complete_ {false};

  int target_marker_id_ {};
  int measurements_required_ {};
  // int measures_captured_quantity_ {};

  std::string robot_base_frame_name_ {};
  std::string robot_eef_frame_name_ {};
  std::string workspace_dir_ {};

  // geometry_msgs::msg::Transform base2eef_transform_ {};
  // geometry_msgs::msg::Pose cam2eef_pose_ {};
  aruco_opencv_msgs::msg::ArucoDetection detected_markers_ {};

  // Eigen::Affine3d base2eef_frame_ {Eigen::Affine3d::Identity()};
  // Eigen::Affine3d cam2eef_frame_ {Eigen::Affine3d::Identity()};
  // Eigen::Affine3d base2cam_frame_ {Eigen::Affine3d::Identity()};
  Eigen::Affine3d camera_pose_robot_base_ {Eigen::Affine3d::Identity()};

  // Stores exactly the same matrix of base2cam_frame_ but in cv::Affine3d format.
  // cv::Affine3d base2cam_frame_mat_ {cv::Affine3d::Identity()};

  Measurements::Calibration calibration_measurements_ {};
  Measurements::CalibrationCheck calibration_check_measurements_ {};

  // std::vector<cv::Mat> base2eef_frame_tvecs_ {};
  // std::vector<cv::Mat> base2eef_frame_rmatxs_ {};
  // std::vector<cv::Mat> cam2eef_frame_tvecs_ {};
  // std::vector<cv::Mat> cam2eef_frame_rmatxs_ {};

  // std::vector<Eigen::Vector3d> estimated_eef_positions_ {};
  // std::vector<Eigen::Vector3d> actual_eef_positions_ {};

  // std::vector<Eigen::Quaterniond> estimated_eef_orientations_ {};
  // std::vector<Eigen::Quaterniond> actual_eef_orientations_ {};

  // Eigen::Vector<double, 7> mean_error_vector_ {Eigen::Vector<double, 7>::Zero()};
  // Eigen::Vector<double, 7> sum_of_squared_errors_vector_ {Eigen::Vector<double, 7>::Zero()};
  // Eigen::Vector<double, 7> root_sum_of_squared_errors_vector_ {Eigen::Vector<double, 7>::Zero()};

  // Eigen::Matrix<double, 7, 7> covariance_matrix_ {Eigen::Matrix<double, 7, 7>::Zero()};

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ {nullptr};
  // geometry_msgs::msg::TransformStamped tf_static_transform_ {};

  std::unique_ptr<tf2_ros::TransformListener> tf_listener_ {nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_ {nullptr};

  rclcpp::TimerBase::SharedPtr general_timer_ {nullptr};
  rclcpp::TimerBase::SharedPtr measurements_capture_timer_ {nullptr};

  rclcpp::Subscription<aruco_opencv_msgs::msg::ArucoDetection>::SharedPtr marker_pose_sub_ {nullptr};
  rclcpp::Publisher<lab7::msg::Status>::SharedPtr status_pub_ {nullptr};
  rclcpp::Service<lab7::srv::CaptureMeasurements>::SharedPtr capture_measurements_service_ {nullptr};

};
