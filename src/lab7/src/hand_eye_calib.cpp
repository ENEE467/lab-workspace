#include <filesystem>
#include <fstream>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "lab7/hand_eye_calib.hpp"

HandEyeCalibNode::HandEyeCalibNode()
: rclcpp::Node("hand_eye_calib_node")
{
  using namespace std::chrono_literals;

  timer_ = this->create_wall_timer(500ms, std::bind(&HandEyeCalibNode::timerCallback, this));
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

  marker_pose_sub_ = this->create_subscription<aruco_opencv_msgs::msg::ArucoDetection>(
    "aruco_detections", 10,
    std::bind(&HandEyeCalibNode::getEndEffector2CameraFrame, this, std::placeholders::_1));

  hand_eye_calib_service_ = this->create_service<lab7::srv::HandEyeCalib>(
    "hand_eye_calib",
    std::bind(&HandEyeCalibNode::serviceCallback, this, std::placeholders::_1, std::placeholders::_2));

  auto param_description {rcl_interfaces::msg::ParameterDescriptor()};

  param_description.description = "Sets the workspace directory for saving the plots and CSV files";
  this->declare_parameter("workspace_dir", "", param_description);

  param_description.description = "ID of the fiducial marker used on the end-effector";
  this->declare_parameter("marker_id", 0, param_description);

  param_description.description = "Name of the robot base frame";
  this->declare_parameter("robot_base_frame", "base_link", param_description);

  param_description.description = "Name of the robot end-effector frame";
  this->declare_parameter("robot_eef_frame", "tool0", param_description);

  param_description.description = "Minimum number of measurements for calibration";
  this->declare_parameter("measurements_required", 15, param_description);

  workspace_dir_ = this->get_parameter("workspace_dir").as_string();

  if (!std::filesystem::is_directory(workspace_dir_)) {
    RCLCPP_FATAL_STREAM(
      this->get_logger(), "Given workspace directory: " << workspace_dir_ << " does not exist!");

    rclcpp::shutdown();
  }

  robot_base_frame_ = this->get_parameter("robot_base_frame").as_string();
  robot_eef_frame_ = this->get_parameter("robot_eef_frame").as_string();

  RCLCPP_INFO(this->get_logger(), "Hand-Eye Calibration node initialized.");
}

void HandEyeCalibNode::timerCallback()
{
  marker_id_ = this->get_parameter("marker_id").as_int();

  getBase2EndEffectorFrame();
  broadcastBase2CameraFrame();
}

void HandEyeCalibNode::serviceCallback(
  const std::shared_ptr<lab7::srv::HandEyeCalib::Request> request,
  std::shared_ptr<lab7::srv::HandEyeCalib::Response> response)
{
  switch (request->action) {

  case (lab7::srv::HandEyeCalib::Request::STATUS):
    break;

  case (lab7::srv::HandEyeCalib::Request::CAPTURE):
    if (!is_calibration_complete_)
      captureCalibrationMeasure();
    else
      captureVerificationMeasure();

    break;

  case (lab7::srv::HandEyeCalib::Request::CALIBRATE):
    calibrateHandEye();

    if (!is_calibration_complete_) {
      response->set__success(false);

      return;
    }

    break;

  case (lab7::srv::HandEyeCalib::Request::VERIFY):
    verifyCalibration();

    if (!is_verification_complete_) {
      response->set__success(false);

      return;
    }

    break;

  case (lab7::srv::HandEyeCalib::Request::RESET):
    resetMeasurements();
    break;

  case (lab7::srv::HandEyeCalib::Request::SAVE):
    if (!is_verification_complete_)
      saveCalibrationOutput();
    else
      saveVerificationOutput();
    break;

  default:
    response->set__success(false);
    return;

  }

  response->measurements_captured = measures_captured_quantity_;
  response->measurements_required = this->get_parameter("measurements_required").as_int();
  response->set__success(true);
}

void HandEyeCalibNode::resetMeasurements()
{
  base2eef_frame_tvecs_.clear();
  base2eef_frame_rmatxs_.clear();
  cam2eef_frame_tvecs_.clear();
  cam2eef_frame_rmatxs_.clear();

  estimated_eef_positions_.clear();
  actual_eef_positions_.clear();
  estimated_eef_orientations_.clear();
  actual_eef_orientations_.clear();

  measures_captured_quantity_ = 0;

  is_calibration_complete_ = false;

  RCLCPP_INFO(this->get_logger(), "Measurements have been reset, you can start over now.");
}

void HandEyeCalibNode::getBase2EndEffectorFrame()
{
  try {
    base2eef_transform_ =
      tf_buffer_->lookupTransform(
        robot_eef_frame_, robot_base_frame_, tf2::TimePointZero).transform;
  }
  catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_STREAM(
      this->get_logger(),
      "Tried to transform " << robot_base_frame_ << " to " << robot_eef_frame_  << " : "
                            << ex.what());

    is_base2eef_frame_available_ = false;

    return;
  }

  Eigen::Vector3d translation {
    base2eef_transform_.translation.x,
    base2eef_transform_.translation.y,
    base2eef_transform_.translation.z};

  Eigen::Quaterniond rotation {
    base2eef_transform_.rotation.w,
    base2eef_transform_.rotation.x,
    base2eef_transform_.rotation.y,
    base2eef_transform_.rotation.z};

  rotation.normalize();

  base2eef_frame_.translation() = translation;
  base2eef_frame_.matrix().topLeftCorner<3, 3>() = rotation.toRotationMatrix();

  is_base2eef_frame_available_ = true;
}

void HandEyeCalibNode::getEndEffector2CameraFrame(const aruco_opencv_msgs::msg::ArucoDetection& msg)
{
  for (const auto& marker_pose: msg.markers) {
    if (marker_pose.marker_id != marker_id_)
      continue;

    cam2eef_pose_ = marker_pose.pose;

    Eigen::Vector3d translation {
      cam2eef_pose_.position.x,
      cam2eef_pose_.position.y,
      cam2eef_pose_.position.z};

    Eigen::Quaterniond rotation {
      cam2eef_pose_.orientation.w,
      cam2eef_pose_.orientation.x,
      cam2eef_pose_.orientation.y,
      cam2eef_pose_.orientation.z};

    rotation.normalize();

    cam2eef_frame_.translation() = translation;
    cam2eef_frame_.matrix().topLeftCorner<3, 3>() = rotation.toRotationMatrix();

    is_cam2eef_frame_available_ = true;

    return;
  }

  is_cam2eef_frame_available_ = false;
}

void HandEyeCalibNode::captureCalibrationMeasure()
{
  if (!is_base2eef_frame_available_ || !is_cam2eef_frame_available_) {
    RCLCPP_WARN(
      this->get_logger(),
      "Measure capture failed: One/Both of the frames is unavailable, try again.");

    return;
  }

  cv::Affine3d base2eef_frame_mat {};
  cv::eigen2cv(base2eef_frame_.matrix(), base2eef_frame_mat.matrix);

  cv::Affine3d cam2eef_frame_mat {};
  cv::eigen2cv(cam2eef_frame_.matrix(), cam2eef_frame_mat.matrix);

  base2eef_frame_tvecs_.emplace_back(base2eef_frame_mat.translation());
  base2eef_frame_rmatxs_.emplace_back(base2eef_frame_mat.rotation());

  cam2eef_frame_tvecs_.emplace_back(cam2eef_frame_mat.translation());
  cam2eef_frame_rmatxs_.emplace_back(cam2eef_frame_mat.rotation());

  measures_captured_quantity_++;

  RCLCPP_INFO(this->get_logger(), "Measure captured successfully.");
  RCLCPP_INFO_STREAM(this->get_logger(), "Measures captured: " << measures_captured_quantity_);
}

void HandEyeCalibNode::captureVerificationMeasure()
{
  if (!is_calibration_complete_) {
    RCLCPP_WARN(
      this->get_logger(),
      "Calibration needs to be done first before capturing measures for verification.");

    return;
  }

  if (!is_base2eef_frame_available_ || !is_cam2eef_frame_available_) {
    RCLCPP_WARN(
      this->get_logger(),
      "Measure capture failed: One/Both of the frames is unavailable, try again.");

    return;
  }

  auto estimated_eef_pose {base2cam_frame_ * cam2eef_frame_};

  estimated_eef_positions_.emplace_back(estimated_eef_pose.translation());
  estimated_eef_orientations_.emplace_back(estimated_eef_pose.rotation());

  actual_eef_positions_.emplace_back(base2eef_frame_.translation());
  actual_eef_orientations_.emplace_back(base2eef_frame_.rotation());

  measures_captured_quantity_++;

  RCLCPP_INFO(this->get_logger(), "Measure captured successfully.");
  RCLCPP_INFO_STREAM(this->get_logger(), "Measures captured: " << measures_captured_quantity_);
}

void HandEyeCalibNode::calibrateHandEye()
{
  int required_measures = this->get_parameter("measurements_required").as_int();

  if (measures_captured_quantity_ < required_measures) {
    RCLCPP_WARN(this->get_logger(), "Calibration failed: Insufficient measurements.");
    RCLCPP_INFO_STREAM(
      this->get_logger(),
      "Get " << required_measures - measures_captured_quantity_ << " more measurements and try again.");

    is_calibration_complete_ = false;

    return;
  }

  RCLCPP_INFO(this->get_logger(), "Calibrating... This might take some while.");

  cv::Mat base2cam_rotation_matrix {};
  cv::Vec3d base2cam_translation_vector {};

  try {
    cv::calibrateHandEye(
      base2eef_frame_rmatxs_, base2eef_frame_tvecs_,
      cam2eef_frame_rmatxs_, cam2eef_frame_tvecs_,
      base2cam_rotation_matrix, base2cam_translation_vector,
      cv::CALIB_HAND_EYE_PARK);
  }
  catch (const cv::Exception& exception) {
    std::cerr << exception.what();

    is_calibration_complete_ = false;
    RCLCPP_WARN(this->get_logger(), "Calibration failed, try again.");

    resetMeasurements();

    return;
  }

  base2cam_frame_mat_.rotation(base2cam_rotation_matrix);
  base2cam_frame_mat_.translation(base2cam_translation_vector);
  cv::cv2eigen(base2cam_frame_mat_.matrix, base2cam_frame_.matrix());

  is_calibration_complete_ = true;
  measures_captured_quantity_ = 0;

  RCLCPP_INFO(
    this->get_logger(), "Hand-eye calibration complete, now you can capture frames for verification");

  RCLCPP_INFO(this->get_logger(), "Estimated frame will now be broadcasted.");

}

void HandEyeCalibNode::broadcastBase2CameraFrame()
{
  if (!is_calibration_complete_)
    return;

  tf_static_transform_.header.stamp = this->get_clock()->now();
  tf_static_transform_.header.frame_id = robot_base_frame_;
  tf_static_transform_.child_frame_id = "camera";

  tf_static_transform_.transform.translation.x = base2cam_frame_.translation()(0);
  tf_static_transform_.transform.translation.y = base2cam_frame_.translation()(1);
  tf_static_transform_.transform.translation.z = base2cam_frame_.translation()(2);

  Eigen::Quaterniond rotation {base2cam_frame_.rotation()};
  tf_static_transform_.transform.rotation.w = rotation.w();
  tf_static_transform_.transform.rotation.x = rotation.x();
  tf_static_transform_.transform.rotation.y = rotation.y();
  tf_static_transform_.transform.rotation.z = rotation.z();

  tf_static_broadcaster_->sendTransform(tf_static_transform_);
}

std::string HandEyeCalibNode::createTimeStamp()
{
  std::stringstream timeStamp;

  std::time_t t {std::time(nullptr)};
  std::tm tm {*std::localtime(&t)};
  timeStamp << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");

  return timeStamp.str();
}

void HandEyeCalibNode::saveCalibrationOutput()
{
  if (!is_calibration_complete_) {
    RCLCPP_WARN(this->get_logger(), "Calibration is incomplete, not saving the file.");

    return;
  }

  if (!std::filesystem::is_directory(workspace_dir_ + "/output/lab7"))
    std::filesystem::create_directories(workspace_dir_ + "/output/lab7");

  std::string output_file_name {
    workspace_dir_ + "/output/lab7/frame-" + createTimeStamp()};

  cv::FileStorage output_file {output_file_name + ".yaml", cv::FileStorage::WRITE};

  if (!output_file.isOpened()) {
    RCLCPP_FATAL(this->get_logger(), "Failed to write output, unable to write a new output file.");

    return;
  }

  output_file.writeComment("\nTransformation from base to camera frame");
  output_file << "estimated_transformation" << base2cam_frame_mat_.matrix;
  output_file.release();

  std::ofstream output_file_txt {output_file_name + ".txt"};

  if (!output_file_txt.is_open())
    return;

  output_file_txt << "Estimated Transformation Matrix: \n"
                  << base2cam_frame_.matrix() << '\n';

  Eigen::Vector<double, 7> pose_vector;
  Eigen::Quaterniond rotation_q {base2cam_frame_.rotation()};
  pose_vector << base2cam_frame_.translation(), rotation_q.coeffs();

  output_file_txt << "Estimated transform in pose vector format: \n"
                  << pose_vector << '\n';

  output_file_txt.close();
}

void HandEyeCalibNode::saveVerificationOutput()
{
  if (!is_calibration_complete_ || !is_verification_complete_) {
    RCLCPP_WARN(this->get_logger(), "Calibration/verification is incomplete, not saving the file.");

    return;
  }

  std::string output_file_name {
    workspace_dir_ + "/output/lab7/verification-" + createTimeStamp() + ".txt"};

  std::ofstream output_txt {output_file_name};

  if (!output_txt.is_open()) {
    RCLCPP_FATAL(this->get_logger(), "Failed to write output, unable to write a new output file.");

    return;
  }

  output_txt << "Mean error vector: \n"
             << mean_error_vector_ << '\n' << '\n'
             << "Covariance matrix: " << '\n'
             << covariance_matrix_ << '\n' << '\n'
             << "Sum of squares error vector: " << '\n'
             << sum_of_squared_errors_vector_ << '\n' << '\n'
             << "Root sum of squares error vector: " << '\n'
             << root_sum_of_squared_errors_vector_ << '\n';

  output_txt.close();
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto hand_eye_calib_node {std::make_shared<HandEyeCalibNode>()};

  rclcpp::spin(hand_eye_calib_node);

  rclcpp::shutdown();

  return 0;
}
