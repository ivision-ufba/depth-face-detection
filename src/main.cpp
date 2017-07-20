#include <opencv2/highgui.hpp>

#include "face_detector.hpp"
#include "realsense.hpp"

#include <iostream>
using namespace std;

int main() {
  RealSense rs;
  FaceDetector detector;

  while (true) {
    rs.get_images();
    auto dets = detector.detect_frontal(rs.depth, rs.calibration);
    cvtColor(rs.depth, rs.depth, CV_GRAY2RGB);
    for (auto det : dets) {
      auto p = rs.calibration.xyz_to_depth(det);
      cv::circle(rs.depth, p, 1, cv::Scalar((1 << 16) - 1, 0, 0), 3);
    }

    cv::imshow("depth", rs.depth);
    cv::waitKey(10);
  }
}
