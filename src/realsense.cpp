#include "realsense.hpp"

RealSense::RealSense() {
  if (context.get_device_count() == 0)
    throw std::runtime_error("Error: not enough RealSense devices connected");

  dev = context.get_device(0);

  dev->enable_stream(rs::stream::depth, rs::preset::best_quality);

  dev->start();

  calibration.calibrate(dev->get_stream_intrinsics(rs::stream::depth),
                        dev->get_depth_scale());

  depth.create(480, 640, CV_16U);
}

void RealSense::get_images() {
  dev->wait_for_frames();
  void* depth_frame = (void*)dev->get_frame_data(rs::stream::depth);
  depth = cv::Mat(480, 640, CV_16U, depth_frame);
}

void RealSenseCalibration::calibrate(const rs::intrinsics& depth_intrinsics,
                                     const float scale) {
  this->depth_intrinsics = depth_intrinsics;
  this->scale = scale;
}

cv::Point3d RealSenseCalibration::depth_to_xyz(const float x, const float y,
                                               const float z) const {
  rs::float3 rs_point = depth_intrinsics.deproject({x, y}, z * scale);
  return cv::Point3d(rs_point.x * 1000.0, -rs_point.y * 1000.0,
                     -rs_point.z * 1000.0);
}

cv::Point RealSenseCalibration::xyz_to_depth(const cv::Point3d& point) const {
  cv::Point p;
  p.x = -(point.x * depth_intrinsics.fx) / point.z + depth_intrinsics.ppx;
  p.y = (point.y * depth_intrinsics.fy) / point.z + depth_intrinsics.ppy;
  return p;
}
