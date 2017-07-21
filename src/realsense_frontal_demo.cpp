#include <opencv2/highgui.hpp>

#include "face_detector.hpp"
#include "realsense.hpp"

/* Straighforward RealSense demonstration */

int main() {
  RealSense rs;
  FaceDetector detector;

  while (true) {
    rs.get_images();
    auto dets = detector.detect_frontal(rs.depth, rs.calibration);
    cvtColor(rs.depth, rs.depth, CV_GRAY2RGB);
    for (auto det : dets)
      cv::rectangle(rs.depth, det, cv::Scalar((1 << 16) - 1, 0, 0), 1);

    cv::imshow("depth", rs.depth);
    cv::waitKey(10);
  }
}
