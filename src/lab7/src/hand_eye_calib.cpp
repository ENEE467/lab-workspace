#include <filesystem>
#include <fstream>

// #include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/affine.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "lab7/hand_eye_calib.hpp"

HandEyeCalibNode::HandEyeCalibNode()
: rclcpp::Node("hand_eye_calib_node")
{
  using namespace std::chrono_literals;

  general_timer_ =
    this->create_wall_timer(500ms, std::bind(&HandEyeCalibNode::generalTimerCallback, this));

  measurements_capture_timer_ =
    this->create_wall_timer(250ms, std::bind(&HandEyeCalibNode::measurementsCaptureTimerCallback, this));

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  marker_pose_sub_ = this->create_subscription<aruco_opencv_msgs::msg::ArucoDetection>(
    "aruco_detections", 10,
    std::bind(&HandEyeCalibNode::arucoDetectionsSubscriptionCallback, this, std::placeholders::_1));

  capture_measurements_service_ = this->create_service<lab7::srv::CaptureMeasurements>(
    "capture_measurements",
    std::bind(
      &HandEyeCalibNode::captureMeasurementsServiceCallback, this,
      std::placeholders::_1, std::placeholders::_2));

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

  param_description.description = "Threshold in meters for linear displacement in any of the axes";
  this->declare_parameter("linear_displacement_threshold", 0.1, param_description);

  param_description.description = "Threshold in degrees for angular displacement in any of the axes";
  this->declare_parameter("angular_displacement_threshold", 45, param_description);

  workspace_dir_ = this->get_parameter("workspace_dir").as_string();

  if (!std::filesystem::is_directory(workspace_dir_)) {
    RCLCPP_FATAL_STREAM(
      this->get_logger(), "Given workspace directory: " << workspace_dir_ << " does not exist!");

    rclcpp::shutdown();
  }

  if (!std::filesystem::is_directory(workspace_dir_ + "/output/lab7"))
    std::filesystem::create_directories(workspace_dir_ + "/output/lab7");

  robot_base_frame_name_ = this->get_parameter("robot_base_frame").as_string();
  robot_eef_frame_name_ = this->get_parameter("robot_eef_frame").as_string();
  target_marker_id_ = this->get_parameter("marker_id").as_int();
  measurements_required_ = this->get_parameter("measurements_required").as_int();

  double lin_disp_thld {this->get_parameter("linear_displacement_threshold").as_double()};
  double ang_disp_thld {this->get_parameter("angular_displacement_threshold").as_double()};

  if (lin_disp_thld <= 0.0 || ang_disp_thld <= 0.0) {
    RCLCPP_FATAL_STREAM(
      this->get_logger(), "Thresholds for capturing measurements can't be set to zero or negative.");

    rclcpp::shutdown();
  }

  calibration_measurements_.setCaptureThresholds(lin_disp_thld, ang_disp_thld);
  calibration_check_measurements_.setCaptureThresholds(lin_disp_thld, ang_disp_thld);

  RCLCPP_INFO(this->get_logger(), "Hand-Eye Calibration node initialized.");
}

void HandEyeCalibNode::generalTimerCallback()
{
  // getBase2EndEffectorFrame();
  // broadcastBase2CameraFrame();

  if (!is_calibration_complete_ && captured_calibration_measurements_) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Measurements for calibration captured successfully...");

    is_calibration_complete_ = calibrateHandEye(calibration_measurements_, camera_pose_robot_base_);

    if (is_calibration_complete_) {

      std::string output_file_name {
        workspace_dir_ + "/output/lab7/Calibration-" + createTimeStamp()};

      std::ofstream output_file_txt {output_file_name + ".txt"};
      saveCalibrationOutput(output_file_txt, camera_pose_robot_base_);

      /**
       * Automatically pauses after calibration to capture the next set of measurements for checking
       * the calibration result. Should manually send another service request to start capturing the
       * next set of measurements.
       */

      should_capture_measurements_ = false;
    }
  }

  if (is_calibration_complete_) {
    broadcastFrameCamera_RobotBase(camera_pose_robot_base_);
  }

  if (captured_calibration_measurements_ && !saved_calibration_check_measurements_) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Measurements for calibration check captured successfully");
    RCLCPP_INFO_STREAM(this->get_logger(), "Saving the measurements...");

    std::string output_file_name {
      workspace_dir_ + "/output/lab7/Calibration-Check-Measurements-" + createTimeStamp()};

    cv::FileStorage output_file_yaml {output_file_name + ".yaml", cv::FileStorage::WRITE};
    calibration_check_measurements_.writeMeasurements(output_file_yaml);

    saved_calibration_check_measurements_ = true;
    should_capture_measurements_ = false;

    RCLCPP_INFO_STREAM(this->get_logger(), "All measurements captured successfully.");
  }

}

void HandEyeCalibNode::measurementsCaptureTimerCallback()
{
  if (!is_calibration_complete_ && should_capture_measurements_) {
    captured_calibration_measurements_ = captureCalibrationMeasurements(calibration_measurements_);
  }

  else if (!is_calibration_complete_
          && !should_capture_measurements_
          && calibration_measurements_.getNumberOfMeasurements() != 0) {
    calibration_measurements_.clearMeasurements();

    RCLCPP_INFO_STREAM(this->get_logger(), "Cancelled capturing calibration measurements.");
  }

  if (is_calibration_complete_ && should_capture_measurements_) {
    captured_calibration_check_measurements_ =
      captureCalibrationCheckMeasurements(calibration_check_measurements_);
  }

  else if (is_calibration_complete_
          && !should_capture_measurements_
          && calibration_check_measurements_.getNumberOfMeasurements() != 0) {
    calibration_check_measurements_.clearMeasurements();

    RCLCPP_INFO_STREAM(this->get_logger(), "Cancelled capturing calibration check measurements.");
  }
}

void HandEyeCalibNode::captureMeasurementsServiceCallback(
  const std::shared_ptr<lab7::srv::CaptureMeasurements::Request> request,
  std::shared_ptr<lab7::srv::CaptureMeasurements::Response> response)
{
  switch (request->action) {

  case (lab7::srv::CaptureMeasurements::Request::START):
    should_capture_measurements_ = true;
    break;

  case (lab7::srv::CaptureMeasurements::Request::CANCEL):
    should_capture_measurements_ = false;
    break;

  case (lab7::srv::CaptureMeasurements::Request::RESET):
    resetMeasurements();
    break;

  default:
    response->set__success(false);
    return;

  }

  response->set__success(true);
}

void HandEyeCalibNode::resetMeasurements()
{
  // base2eef_frame_tvecs_.clear();
  // base2eef_frame_rmatxs_.clear();
  // cam2eef_frame_tvecs_.clear();
  // cam2eef_frame_rmatxs_.clear();

  // estimated_eef_positions_.clear();
  // actual_eef_positions_.clear();
  // estimated_eef_orientations_.clear();
  // actual_eef_orientations_.clear();

  calibration_measurements_.clearMeasurements();
  calibration_check_measurements_.clearMeasurements();
  camera_pose_robot_base_.setIdentity();
  is_calibration_complete_ = false;
  should_capture_measurements_ = false;

  RCLCPP_INFO(this->get_logger(), "Measurements have been reset, you can start over now.");
}

bool HandEyeCalibNode::getPoseEndEffector_RobotBase(Eigen::Affine3d& pose_out)
{
  pose_out.setIdentity();
  geometry_msgs::msg::Transform base2eef_transform {};

  try {
    base2eef_transform =
      tf_buffer_->lookupTransform(
        robot_eef_frame_name_, robot_base_frame_name_, tf2::TimePointZero).transform;
  }
  catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_STREAM(
      this->get_logger(),
      "Tried to transform " << robot_base_frame_name_ << " to " << robot_eef_frame_name_  << " : "
                            << ex.what());

    return false;
  }

  Eigen::Vector3d translation {
    base2eef_transform.translation.x,
    base2eef_transform.translation.y,
    base2eef_transform.translation.z};

  Eigen::Quaterniond rotation {
    base2eef_transform.rotation.w,
    base2eef_transform.rotation.x,
    base2eef_transform.rotation.y,
    base2eef_transform.rotation.z};

  rotation.normalize();

  pose_out.translation() = translation;
  pose_out.matrix().topLeftCorner<3, 3>() = rotation.toRotationMatrix();

  return true;
}

void HandEyeCalibNode::arucoDetectionsSubscriptionCallback(
  const aruco_opencv_msgs::msg::ArucoDetection& msg)
{
  detected_markers_ = msg;
}

bool HandEyeCalibNode::getPoseMarker_Camera(Eigen::Affine3d& pose_out)
{
  pose_out.setIdentity();
  geometry_msgs::msg::Transform base2marker_transform {};
  std::string marker_frame {"marker_" + std::to_string(target_marker_id_)};

  try {
    base2marker_transform =
      tf_buffer_->lookupTransform(
        marker_frame, robot_base_frame_name_, tf2::TimePointZero).transform;
  }
  catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_STREAM(
      this->get_logger(),
      "Tried to transform " << marker_frame << " to " << robot_eef_frame_name_  << " : "
                            << ex.what());

    return false;
  }

  Eigen::Vector3d translation {
    base2marker_transform.translation.x,
    base2marker_transform.translation.y,
    base2marker_transform.translation.z};

  Eigen::Quaterniond rotation {
    base2marker_transform.rotation.w,
    base2marker_transform.rotation.x,
    base2marker_transform.rotation.y,
    base2marker_transform.rotation.z};

  rotation.normalize();

  pose_out.translation() = translation;
  pose_out.matrix().topLeftCorner<3, 3>() = rotation.toRotationMatrix();

  return true;
}

bool HandEyeCalibNode::getPoseMarker_RobotBase(Eigen::Affine3d& pose_out)
{
  pose_out.setIdentity();

  for (const auto& marker: detected_markers_.markers) {
    if (marker.marker_id != target_marker_id_)
      continue;

    Eigen::Vector3d marker_position {
      marker.pose.position.x,
      marker.pose.position.y,
      marker.pose.position.z};

    Eigen::Quaterniond marker_orientation {
      marker.pose.orientation.w,
      marker.pose.orientation.x,
      marker.pose.orientation.y,
      marker.pose.orientation.z};

    marker_orientation.normalize();

    pose_out.translation() = marker_position;
    pose_out.matrix().topLeftCorner<3, 3>() = marker_orientation.toRotationMatrix();

    return true;
  }

  return false;
}

bool HandEyeCalibNode::captureCalibrationMeasurements(Measurements::Calibration& measurements_out)
{
  if (measurements_out.getNumberOfMeasurements() >= measurements_required_)
    return true;

  Eigen::Affine3d eef_pose_robot_base {Eigen::Affine3d::Identity()};
  Eigen::Affine3d eef_pose_camera {Eigen::Affine3d::Identity()};

  bool got_eef_pose_robot_base {getPoseEndEffector_RobotBase(eef_pose_robot_base)};

  /**
   * Marker is mounted on the end-effector so both these poses are treated as same.
   */

  bool got_eef_pose_camera {getPoseMarker_Camera(eef_pose_camera)};

  if (!got_eef_pose_robot_base || !got_eef_pose_camera)
    return false;

  bool is_captured {measurements_out.addMeasurement(eef_pose_robot_base, eef_pose_camera)};

  if (!is_captured)
    return false;

  std::stringstream info_stream {};
  info_stream << "Calibration measurement captured, "
              << "measurements remaining: "
              << measurements_required_ - measurements_out.getNumberOfMeasurements() << '\n';

  RCLCPP_INFO_STREAM(this->get_logger(), info_stream.str());

  return false;
}

bool HandEyeCalibNode::captureCalibrationCheckMeasurements(
  Measurements::CalibrationCheck& measurements_out)
{
  if (!is_calibration_complete_) {
    RCLCPP_WARN(
      this->get_logger(),
      "Calibration needs to be done first before capturing measures for verification.");

    return false;
  }

  if (measurements_out.getNumberOfMeasurements() >= measurements_required_)
    return true;

  Eigen::Affine3d estimated_eef_pose {Eigen::Affine3d::Identity()};
  Eigen::Affine3d measured_eef_pose {Eigen::Affine3d::Identity()};

  /**
   * Marker is mounted on the end-effector so both these poses are treated as same.
   */

  bool got_estimated_eef_pose {getPoseMarker_RobotBase(estimated_eef_pose)};
  bool got_measured_eef_pose {getPoseEndEffector_RobotBase(measured_eef_pose)};

  if (!got_estimated_eef_pose || !got_measured_eef_pose)
    return false;

  bool is_captured {measurements_out.addMeasurement(estimated_eef_pose, measured_eef_pose)};

  if (!is_captured)
    return false;

  std::stringstream info_stream {};
  info_stream << "Calibration check measurement captured, "
              << "measurements remaining: "
              << measurements_required_ - measurements_out.getNumberOfMeasurements() << '\n';

  RCLCPP_INFO_STREAM(this->get_logger(), info_stream.str());

  return false;
}

bool HandEyeCalibNode::calibrateHandEye(
  const Measurements::Calibration& measurements_in,
  Eigen::Affine3d& pose_out)
{
  pose_out.setIdentity();

  if (measurements_in.getNumberOfMeasurements() < measurements_required_) {
    RCLCPP_WARN(this->get_logger(), "Calibration failed: Insufficient measurements.");

    std::stringstream info_stream {};
    info_stream << "Get "
                << measurements_required_ - measurements_in.getNumberOfMeasurements()
                << " more measurements and try again.";

    RCLCPP_INFO_STREAM(this->get_logger(), info_stream.str());

    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Calibrating... This might take some while.");

  cv::Mat camera_rotation_matrix {};
  cv::Vec3d camera_translation_vector {};

  try {
    cv::calibrateHandEye(
      measurements_in.getCVRotationMatricesEEF_RobotBase(),
      measurements_in.getCVTranslationVectorsEEF_RobotBase(),

      measurements_in.getCVRotationMatricesEEF_Camera(),
      measurements_in.getCVTranslationVectorsEEF_Camera(),

      camera_rotation_matrix, camera_translation_vector,
      cv::CALIB_HAND_EYE_PARK);
  }
  catch (const cv::Exception& exception) {
    std::cerr << exception.what();
    RCLCPP_WARN(this->get_logger(), "Calibration failed, try again.");

    return false;
  }

  cv::Affine3d camera_pose_robot_base {cv::Affine3d::Identity()};

  camera_pose_robot_base.rotation(camera_rotation_matrix);
  camera_pose_robot_base.translation(camera_translation_vector);

  cv::cv2eigen(camera_pose_robot_base.matrix, pose_out.matrix());

  RCLCPP_INFO(this->get_logger(), "Hand-eye calibration complete. Check your result for accuracy.");

  return true;
}

void HandEyeCalibNode::broadcastFrameCamera_RobotBase(const Eigen::Affine3d& pose_in)
{
  if (!is_calibration_complete_)
    return;

  geometry_msgs::msg::TransformStamped tf_static_transform {};

  tf_static_transform.header.stamp = this->get_clock()->now();
  tf_static_transform.header.frame_id = robot_base_frame_name_;
  tf_static_transform.child_frame_id = "camera";

  tf_static_transform.transform.translation.x = pose_in.translation()(0);
  tf_static_transform.transform.translation.y = pose_in.translation()(1);
  tf_static_transform.transform.translation.z = pose_in.translation()(2);

  Eigen::Quaterniond rotation {pose_in.rotation()};
  tf_static_transform.transform.rotation.w = rotation.w();
  tf_static_transform.transform.rotation.x = rotation.x();
  tf_static_transform.transform.rotation.y = rotation.y();
  tf_static_transform.transform.rotation.z = rotation.z();

  tf_broadcaster_->sendTransform(tf_static_transform);
}

std::string HandEyeCalibNode::createTimeStamp()
{
  std::stringstream timeStamp;

  std::time_t t {std::time(nullptr)};
  std::tm tm {*std::localtime(&t)};
  timeStamp << std::put_time(&tm, "%b-%d-%y_%I:%M:%S");

  return timeStamp.str();
}

void HandEyeCalibNode::saveCalibrationOutput(
  std::ofstream& txt_file_out,
  const Eigen::Affine3d& camera_pose_in)
{
  if (!is_calibration_complete_) {
    RCLCPP_WARN(this->get_logger(), "Calibration is incomplete, not saving the file.");

    return;
  }

  // std::string output_file_name {workspace_dir_ + "/output/lab7/Calibration-" + createTimeStamp()};

  // cv::FileStorage output_file {output_file_name + ".yaml", cv::FileStorage::WRITE};

  // if (!output_file.isOpened()) {
  //   RCLCPP_FATAL(this->get_logger(), "Failed to write output, unable to write a new output file.");

  //   return;
  // }

  // output_file.writeComment("\nTransformation from base to camera frame");
  // output_file << "estimated_transformation" << base2cam_frame_mat_.matrix;
  // output_file.release();

  // std::ofstream output_file_txt {output_file_name + ".txt"};

  if (!txt_file_out.is_open()) {
    RCLCPP_FATAL(this->get_logger(), "Failed to write output, unable to write a new output file.");
    return;
  }

  txt_file_out << "Transformation matrix of camera frame from robot's base frame: \n"
               << camera_pose_in.matrix() << '\n';

  Eigen::Vector<double, 7> pose_vector;
  Eigen::Quaterniond rotation_q {camera_pose_in.rotation()};
  pose_vector << camera_pose_in.translation(), rotation_q.coeffs();

  txt_file_out << "Pose vector of camera relative to robot's base frame: \n"
               << pose_vector << '\n';

  txt_file_out.close();
}

// void HandEyeCalibNode::saveVerificationOutput()
// {
//   if (!is_calibration_complete_ || !is_verification_complete_) {
//     RCLCPP_WARN(this->get_logger(), "Calibration/verification is incomplete, not saving the file.");

//     return;
//   }

//   std::string output_file_name {
//     workspace_dir_ + "/output/lab7/verification-" + createTimeStamp() + ".txt"};

//   std::ofstream output_txt {output_file_name};

//   if (!output_txt.is_open()) {
//     RCLCPP_FATAL(this->get_logger(), "Failed to write output, unable to write a new output file.");

//     return;
//   }

//   output_txt << "Mean error vector: \n"
//              << mean_error_vector_ << '\n' << '\n'
//              << "Covariance matrix: " << '\n'
//              << covariance_matrix_ << '\n' << '\n'
//              << "Sum of squares error vector: " << '\n'
//              << sum_of_squared_errors_vector_ << '\n' << '\n'
//              << "Root sum of squares error vector: " << '\n'
//              << root_sum_of_squared_errors_vector_ << '\n';

//   output_txt.close();
// }

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto hand_eye_calib_node {std::make_shared<HandEyeCalibNode>()};

  rclcpp::spin(hand_eye_calib_node);

  rclcpp::shutdown();

  return 0;
}
