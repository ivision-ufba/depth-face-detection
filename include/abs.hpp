#ifndef ABS_HPP
#define ABS_HPP

#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>

#include "calibration.hpp"

class Abs {
 public:
  Abs(const std::string&);
  ~Abs() = default;
  Abs(Abs&&) = delete;
  Abs(const Abs&) = delete;
  Abs& operator=(const Abs&) = delete;

  void load(const std::string&);
  void crop(const cv::Rect&);
  void crop(const cv::Point3d&, double);
  std::vector<cv::Point3d> to_points() const;
  std::vector<cv::Point2d> to_pixels() const;
  cv::Mat to_mat() const;
  void print_registered_pcloud(const cv::Mat&) const;
  cv::Point3d at(const int, const int) const;
  cv::Point3d at(const cv::Point) const;
  cv::Point3d closest_point_to(const int, const int) const;

 private:
  int read_int(const std::string&);
  cv::Mat valid, depth;
  std::vector<cv::Point> neighborhood;
  cv::Rect frame;
};

class AbsCalibration : public Calibration {
 public:
  AbsCalibration(const Abs&);
  ~AbsCalibration() = default;
  AbsCalibration(AbsCalibration&&) = delete;
  AbsCalibration(const AbsCalibration&&) = delete;
  AbsCalibration& operator=(const AbsCalibration&&) = delete;

  cv::Point3d depth_to_xyz(const float, const float, const float) const;
  cv::Point xyz_to_depth(const cv::Point3d&) const;

 private:
  const Abs& abs;
  cv::Point3d mean;
  cv::Mat cmatrix;
};

#endif
