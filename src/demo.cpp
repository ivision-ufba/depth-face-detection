#include <opencv2/highgui.hpp>

#include "face_detector.hpp"

/* Demonstration of depth face detection for a point cloud input. It detects
 * faces in rotation intervals [0, 30] x [-20, 20] x [0, 0] degrees. */

int main(const int argc, char** const argv) {
  std::vector<cv::Point3d> points;

  // read point cloud into points
  // coordinates must be in milimeters

  // detect faces
  FaceDetector detector;
  auto dets = detector.range_detect(points, 0, -20, 0, 30, 20, 0);
}
