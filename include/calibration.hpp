#ifndef CALIBRATION_HPP
#define CALIBRATION_HPP

#include <opencv2/core.hpp>

/* Calibration interface must implement both functions.
 * 3D coordinates are always in milimeters */

class Calibration {
 public:
  virtual cv::Point3d depth_to_xyz(const float x, const float y,
                                   const float z) const = 0;
  virtual cv::Point xyz_to_depth(const cv::Point3d&) const = 0;
};

#endif
