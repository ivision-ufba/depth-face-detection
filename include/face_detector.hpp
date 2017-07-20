#ifndef FACE_DETECTOR_HPP
#define FACE_DETECTOR_HPP

#include <opencv/cv.h>
#include <opencv2/opencv.hpp>

#include <vector>

#include "calibration.hpp"

class FaceDetector {
 public:
  FaceDetector();
  ~FaceDetector();
  FaceDetector(const FaceDetector &) = delete;
  FaceDetector(FaceDetector &&) = delete;
  FaceDetector &operator=(const FaceDetector &) = delete;

  std::vector<cv::Rect> detect_frontal(const cv::Mat &, const Calibration &);
  std::vector<cv::Rect> range_detect(const cv::Mat &, const Calibration &,
                                     const int, const int, const int, const int,
                                     const int, const int);

 private:
  const float resolution;
  const unsigned int projection_width, projection_height, CX, CY, face_size,
      step;
  const double threshold;
  IplImage *projection, *mark, *sum, *squared_sum, *tilted_sum, *mark_sum,
      *sum_int, *tilted_sum_int;
  CvHaarClassifierCascade *face_cascade;

  void compute_rotation_matrix(double rotation_matrix[3][3], const double,
                               const double, const double) const;
  void compute_projection(const std::vector<cv::Point3d> &,
                          double rotation_matrix[3][3]);
  cv::Rect project(const cv::Point3d &, const Calibration &) const;
  std::vector<cv::Rect> project_and_merge(const std::vector<cv::Point3d> &,
                                          const Calibration &) const;
  void fill_holes();
  std::vector<cv::Point3d> grid_sample(const cv::Mat &, const Calibration &,
                                       const double, const unsigned int) const;
  cv::Point3d retrieve_3d_point(const unsigned int, const unsigned int) const;
  std::vector<cv::Point3d> helper(const std::vector<cv::Point3d> &,
                                  const double, const double, const double);
};

#endif
