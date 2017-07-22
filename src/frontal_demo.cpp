#include <opencv2/highgui.hpp>

#include "abs.hpp"
#include "face_detector.hpp"

/* Demonstration of frontal depth face detection for a point cloud */

int main(const int argc, char** const argv) {
  std::vector<cv::Point3d> points;

  // read point cloud into points
  // coordinates must be in milimeters

  // detect faces
  FaceDetector detector;
  auto dets = detector.detect_frontal(points);
}
