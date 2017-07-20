#ifndef REALSENSE_HPP
#define REALSENSE_HPP

#include "calibration.hpp"

#include <memory>

#include <opencv2/core.hpp>

#include <librealsense/rs.hpp>

/* Implementation example for Intel RealSense camera */

class RealSenseCalibration : public Calibration {
 public:
  RealSenseCalibration() = default;
  ~RealSenseCalibration() = default;
  RealSenseCalibration(const RealSenseCalibration &) = delete;
  RealSenseCalibration(RealSenseCalibration &&) = delete;
  RealSenseCalibration &operator=(const RealSenseCalibration &) = delete;

  cv::Point3d depth_to_xyz(const float, const float, const float) const;
  cv::Point xyz_to_depth(const cv::Point3d &) const;
  void calibrate(const rs::intrinsics &, const float);

 private:
  rs::intrinsics depth_intrinsics;
  float scale;
};

class RealSense {
 public:
  RealSense();
  ~RealSense() = default;
  RealSense(RealSense &&) = delete;
  RealSense(const RealSense &) = delete;
  RealSense &operator=(const RealSense &) = delete;

  RealSenseCalibration calibration;

  void get_images();

  cv::Mat depth;

 private:
  rs::context context;
  rs::device *dev;
};

#endif
