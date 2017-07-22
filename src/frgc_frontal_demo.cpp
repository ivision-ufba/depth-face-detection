#include <opencv2/highgui.hpp>

#include "abs.hpp"
#include "face_detector.hpp"

/* Demonstration of frontal depth face detection for a
 * Face Recognition Grand Challenge (FRGC) point cloud
 * input in .abs format */

int main(const int argc, char** const argv) {
  if (argc <= 1)
    throw std::runtime_error("Usage: " + std::string(argv[0]) +
                             " <path to .abs file> ");

  // load abs
  const std::string path = argv[1];
  Abs abs(path);

  // adapt abs to accepted format
  AbsCalibration calibration(abs);
  cv::Mat depth_img = abs.to_mat();

  // detect faces
  FaceDetector detector;
  auto dets = detector.detect_frontal(depth_img, calibration);

  // draw detection
  cv::normalize(depth_img, depth_img, 0, 1, cv::NORM_MINMAX);
  cvtColor(depth_img, depth_img, CV_GRAY2RGB);
  for (auto det : dets) cv::rectangle(depth_img, det, cv::Scalar(1, 0, 0), 1);

  // show detection
  cv::imshow("depth", depth_img);
  cv::waitKey(0);
}
