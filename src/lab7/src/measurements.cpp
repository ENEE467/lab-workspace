#include "lab7/measurements.hpp"

#include <iostream>

#include <Eigen/Geometry>

#include <opencv2/core/eigen.hpp>
#include <opencv2/core/affine.hpp>

namespace Measurements {

bool hasLinearDisplacement(
  const Eigen::Vector3d& current_position_in,
  const Eigen::Vector3d& previous_position_in,
  double threshold_in)
{
  auto linear_displacement {current_position_in - previous_position_in};

  return (linear_displacement.array() > threshold_in).any();
}

bool hasAngularDisplacement(
  const Eigen::Quaterniond& current_orientation_in,
  const Eigen::Quaterniond& previous_orientation_in,
  double threshold_degrees_in)
{
  double threshold_radians {threshold_degrees_in * (M_PI / 180)};

  auto angular_displacement_quat {current_orientation_in.conjugate() * previous_orientation_in};
  angular_displacement_quat.normalize();

  auto angular_displacement_rpy {angular_displacement_quat.toRotationMatrix().eulerAngles(2, 1, 0)};

  return (angular_displacement_rpy.array() > threshold_radians).any();
}

void poseVectorFromEigenAffine(const Eigen::Affine3d& pose_in, std::vector<double>& pose_out)
{
  pose_out.clear();

  Eigen::Vector3d translation_vector {pose_in.translation()};
  for (const auto& position_component: translation_vector) {
    pose_out.push_back(position_component);
  }

  Eigen::Quaterniond orientation_quaternion {pose_in.rotation()};
  for (const auto& quaternion_component: orientation_quaternion.coeffs()) {
    pose_out.push_back(quaternion_component);
  }
}

bool Calibration::addMeasurement(
  const Eigen::Affine3d& eef_pose_robot_base_in,
  const Eigen::Affine3d& eef_pose_camera_in)
{
  if (!measurements_.empty()) {
    // We're more interested in the fiducial marker pose.

    Eigen::Vector3d current_position {eef_pose_camera_in.translation()};
    Eigen::Quaterniond current_orientation {eef_pose_camera_in.rotation()};

    Eigen::Vector3d previous_position {measurements_.back().eef_pose_camera.translation()};
    Eigen::Quaterniond previous_orientation {measurements_.back().eef_pose_camera.rotation()};

    bool has_linear_displacement {
      hasLinearDisplacement(current_position, previous_position, linear_displacement_threshold_)};

    bool has_angular_displacement {
      hasAngularDisplacement(
        current_orientation, previous_orientation, angular_displacement_threshold_)};

    if (!has_linear_displacement || !has_angular_displacement)
      return false;
  }

  measurements_.emplace_back(eef_pose_robot_base_in, eef_pose_camera_in);

  return true;
}

std::vector<cv::Mat> Calibration::getCVRotationMatricesEEF_RobotBase() const
{
  std::vector<cv::Mat> rotation_matrices {};
  cv::Affine3d cv_transformation_matrix {};

  for (const auto& measurement: measurements_) {
    cv::eigen2cv(measurement.eef_pose_robot_base.matrix(), cv_transformation_matrix.matrix);
    rotation_matrices.emplace_back(cv_transformation_matrix.rotation());
  }

  return rotation_matrices;
}

std::vector<cv::Mat> Calibration::getCVTranslationVectorsEEF_RobotBase() const
{
  std::vector<cv::Mat> translation_vectors {};
  cv::Affine3d cv_transformation_matrix {};

  for (const auto& measurement: measurements_) {
    cv::eigen2cv(measurement.eef_pose_robot_base.matrix(), cv_transformation_matrix.matrix);
    translation_vectors.emplace_back(cv_transformation_matrix.translation());
  }

  return translation_vectors;
}

std::vector<cv::Mat> Calibration::getCVRotationMatricesEEF_Camera() const
{
  std::vector<cv::Mat> rotation_matrices {};
  cv::Affine3d cv_transformation_matrix {};

  for (const auto& measurement: measurements_) {
    cv::eigen2cv(measurement.eef_pose_camera.matrix(), cv_transformation_matrix.matrix);
    rotation_matrices.emplace_back(cv_transformation_matrix.rotation());
  }

  return rotation_matrices;
}

std::vector<cv::Mat> Calibration::getCVTranslationVectorsEEF_Camera() const
{
  std::vector<cv::Mat> translation_vectors {};
  cv::Affine3d cv_transformation_matrix {};

  for (const auto& measurement: measurements_) {
    cv::eigen2cv(measurement.eef_pose_camera.matrix(), cv_transformation_matrix.matrix);
    translation_vectors.emplace_back(cv_transformation_matrix.translation());
  }

  return translation_vectors;
}

bool CalibrationCheck::addMeasurement(
  const Eigen::Affine3d& estimated_eef_pose_in,
  const Eigen::Affine3d& measured_eef_pose_in)
{
  if (!measurements_.empty()) {
    // We're more interested in the fiducial marker pose.

    auto current_position {estimated_eef_pose_in.translation()};
    Eigen::Quaterniond current_orientation {estimated_eef_pose_in.rotation()};

    auto previous_position {measurements_.back().estimated_eef_pose.translation()};
    Eigen::Quaterniond previous_orientation {measurements_.back().estimated_eef_pose.rotation()};

    bool has_linear_displacement {
      hasLinearDisplacement(current_position, previous_position, linear_displacement_threshold_)};

    bool has_angular_displacement {
      hasAngularDisplacement(
        current_orientation, previous_orientation, angular_displacement_threshold_)};

    if (!has_linear_displacement || !has_angular_displacement)
      return false;
  }

  measurements_.emplace_back(estimated_eef_pose_in, measured_eef_pose_in);

  return true;
}

bool CalibrationCheck::writeMeasurements(cv::FileStorage& measurements_file_out) const
{
  if (measurements_file_out.isOpened()) {
    std::cerr << "Measurements output file isn't open." << '\n';

    return false;
  }

  if (measurements_.empty()) {
    std::cerr << "There are no measurements to write." << '\n';

    return false;
  }

  int measurement_count {1};
  std::vector<double> estimated_eef_pose_vector {};
  std::vector<double> measured_eef_pose_vector {};

  measurements_file_out.startWriteStruct("eef_pose_measurements", cv::FileNode::MAP);

  for (const auto& measurement: measurements_) {
    measurements_file_out.startWriteStruct(
      "measurement_" + std::to_string(measurement_count), cv::FileNode::MAP);

    poseVectorFromEigenAffine(measurement.estimated_eef_pose, estimated_eef_pose_vector);
    measurements_file_out << "estimated" << estimated_eef_pose_vector;

    poseVectorFromEigenAffine(measurement.measured_eef_pose, measured_eef_pose_vector);
    measurements_file_out << "measured" << measured_eef_pose_vector;

    measurements_file_out.endWriteStruct();

    measurement_count++;
  }

  measurements_file_out.endWriteStruct();
  measurements_file_out.release();

  return true;
}

}
