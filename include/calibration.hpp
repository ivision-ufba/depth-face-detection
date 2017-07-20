#ifndef CALIBRATION_HPP
#define CALIBRATION_HPP

#include <opencv2/core.hpp>

class Calibration {
 public:
  virtual cv::Point3d depth_to_xyz(const float, const float,
                                   const float) const = 0;
  virtual cv::Point xyz_to_depth(const cv::Point3d &) const = 0;
};

#endif
