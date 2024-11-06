#pragma once

#include <vector>

#include <Eigen/Geometry>

#include <opencv2/core/eigen.hpp>
#include <opencv2/core/persistence.hpp>

namespace Measurement {

struct Calibration {

  Calibration(
    const Eigen::Affine3d& eef_pose_robot_base_in,
    const Eigen::Affine3d& eef_pose_camera_in)
  : eef_pose_robot_base {eef_pose_robot_base_in},
    eef_pose_camera {eef_pose_camera_in}
  {}

  // Variable name convention: <target>_pose_<reference frame>
  Eigen::Affine3d eef_pose_robot_base {Eigen::Affine3d::Identity()};
  Eigen::Affine3d eef_pose_camera {Eigen::Affine3d::Identity()};

};

struct CalibrationCheck {

  CalibrationCheck(
    const Eigen::Affine3d& estimated_eef_pose_in,
    const Eigen::Affine3d& measured_eef_pose_in)
  : estimated_eef_pose {estimated_eef_pose_in},
    measured_eef_pose {measured_eef_pose_in}
  {}

  // Reference frame is assumed as the robot base frame.
  Eigen::Affine3d estimated_eef_pose {Eigen::Affine3d::Identity()};
  Eigen::Affine3d measured_eef_pose {Eigen::Affine3d::Identity()};

};

}

namespace Measurements {

class Calibration {

public:
  Calibration() {}

  void clearMeasurements() {measurements_.clear();}
  void setCaptureThresholds(double linear_in, double angular_in)
  {
    linear_displacement_threshold_ = linear_in;
    angular_displacement_threshold_ = angular_in;
  }

  bool addMeasurement(
    const Eigen::Affine3d& eef_pose_robot_base_in,
    const Eigen::Affine3d& eef_pose_camera_in);

  int getNumberOfMeasurements() const {return measurements_.size();}

  std::vector<cv::Mat> getCVRotationMatricesEEF_RobotBase() const;
  std::vector<cv::Mat> getCVTranslationVectorsEEF_RobotBase() const;

  std::vector<cv::Mat> getCVRotationMatricesEEF_Camera() const;
  std::vector<cv::Mat> getCVTranslationVectorsEEF_Camera() const;

private:
  double linear_displacement_threshold_ {0};
  double angular_displacement_threshold_ {0};

  std::vector<Measurement::Calibration> measurements_ {};

};

class CalibrationCheck {

public:
  CalibrationCheck() {}

  void clearMeasurements() {measurements_.clear();}
  void setCaptureThresholds(double linear_in, double angular_in)
  {
    linear_displacement_threshold_ = linear_in;
    angular_displacement_threshold_ = angular_in;
  }

  bool writeMeasurements(cv::FileStorage& measurements_file_out) const;
  bool addMeasurement(
    const Eigen::Affine3d& estimated_eef_pose_in,
    const Eigen::Affine3d& measured_eef_pose_in);

  int getNumberOfMeasurements() const {return measurements_.size();}

private:
  double linear_displacement_threshold_ {0};
  double angular_displacement_threshold_ {0};

  std::vector<Measurement::CalibrationCheck> measurements_ {};

};

}
