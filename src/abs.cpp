#include <abs.hpp>

#include <cassert>
#include <exception>
#include <fstream>
#include <iostream>
#include <queue>
#include <sstream>

Abs::Abs(const std::string& s) { load(s); }

void Abs::crop(const cv::Rect& roi) {
  const int rows = valid.rows;
  const int cols = valid.cols;

  for (int r = 0; r < rows; ++r)
    for (int c = 0; c < cols; ++c) {
      cv::Point p(r, c);
      if (!roi.contains(p)) valid.at<uchar>(r, c) = false;
    }
}

void Abs::crop(const cv::Point3d& center, double dist) {
  const int rows = valid.rows;
  const int cols = valid.cols;
  dist *= dist;

  for (int i = 0; i < rows; i++)
    for (int j = 0; j < cols; j++)
      if (valid.at<uchar>(i, j) &&
          pow(center.x - depth.at<cv::Vec3d>(i, j)[0], 2.0) +
                  pow(center.y - depth.at<cv::Vec3d>(i, j)[1], 2.0) >
              dist)
        valid.at<uchar>(i, j) = false;
}

int Abs::read_int(const std::string& str) {
  std::stringstream ss;
  ss << str;
  int num;
  ss >> num;
  return num;
}

std::vector<cv::Point3d> Abs::to_points() const {
  std::vector<cv::Point3d> points;

  const int rows = valid.rows;
  const int cols = valid.cols;

  for (int i = 0; i < rows; i++)
    for (int j = 0; j < cols; j++)
      if (valid.at<uchar>(i, j) - '0')
        points.emplace_back(depth.at<cv::Vec3d>(i, j));

  return points;
}

cv::Mat Abs::to_mat() const {
  cv::Mat depth_img(valid.rows, valid.cols, CV_32F);

  for (int i = 0; i < valid.rows; i++)
    for (int j = 0; j < valid.cols; j++)
      if (valid.at<uchar>(i, j) - '0')
        depth_img.at<float>(i, j) = depth.at<cv::Vec3d>(i, j)[2];
      else
        depth_img.at<float>(i, j) = 0;

  return depth_img;
}

std::vector<cv::Point2d> Abs::to_pixels() const {
  std::vector<cv::Point2d> pixels;

  const int rows = valid.rows;
  const int cols = valid.cols;

  for (int i = 0; i < rows; i++)
    for (int j = 0; j < cols; j++)
      if (valid.at<uchar>(i, j) - '0') pixels.emplace_back(i, j);

  return pixels;
}

void Abs::print_registered_pcloud(const cv::Mat& img) const {
  const int rows = valid.rows;
  const int cols = valid.cols;

  for (int i = 0; i < rows; ++i)
    for (int j = 0; j < cols; ++j)
      if (valid.at<uchar>(i, j))
        printf("v %.4lf %.4lf %.4lf %u %u %u\n", depth.at<cv::Vec3d>(i, j)[0],
               depth.at<cv::Vec3d>(i, j)[1], depth.at<cv::Vec3d>(i, j)[2],
               (unsigned int)img.at<cv::Vec3b>(i, j)[0],
               (unsigned int)img.at<cv::Vec3b>(i, j)[1],
               (unsigned int)img.at<cv::Vec3b>(i, j)[2]);
}

void Abs::load(const std::string& filename) {
  std::ifstream f(filename);
  if (!f) throw std::runtime_error("Unable to open image file.");

  // read the headers
  std::string l1, l2, l3;
  std::getline(f, l1);
  std::getline(f, l2);
  std::getline(f, l3);

  const int rows = read_int(l1);
  const int cols = read_int(l2);

  // create matrices
  depth.create(rows, cols, CV_64FC3);
  valid.create(rows, cols, CV_8U);

  // read the contents
  for (int i = 0; i < rows; i++)
    for (int j = 0; j < cols; j++) f >> valid.at<uchar>(i, j);

  // read x coordinates
  for (int i = 0; i < rows; i++)
    for (int j = 0; j < cols; j++) f >> depth.at<cv::Vec3d>(i, j)[0];

  // read y coordinates
  for (int i = 0; i < rows; i++)
    for (int j = 0; j < cols; j++) f >> depth.at<cv::Vec3d>(i, j)[1];

  // read z coordinates
  for (int i = 0; i < rows; i++)
    for (int j = 0; j < cols; j++) f >> depth.at<cv::Vec3d>(i, j)[2];
}

cv::Point3d Abs::at(const int i, const int j) const {
  if (valid.at<uchar>(i, j) == '0') return cv::Point3d(0, 0, 0);

  return cv::Point3d(depth.at<cv::Vec3d>(i, j));
}

cv::Point3d Abs::at(const cv::Point p) const {
  if (valid.at<uchar>(p) == '0') return cv::Point3d(0, 0, 0);

  return cv::Point3d(depth.at<cv::Vec3d>(p));
}

cv::Point3d Abs::closest_point_to(const int i, const int j) const {
  // check input boundaries
  if (i < 0 || i >= valid.rows || j < 0 || j >= valid.cols)
    throw std::runtime_error("Out of boundaries");

  // prepare breadth first search
  cv::Mat mask(valid.size(), CV_8U);
  mask.setTo(0);
  std::queue<int> qi, qj;
  qi.push(i);
  qj.push(j);
  mask.at<uchar>(i, j) = 1;

  // breadth first search
  while (!qi.empty()) {
    const int i = qi.front();
    const int j = qj.front();
    qi.pop();
    qj.pop();

    if (valid.at<uchar>(i, j) == '1') return at(i, j);

    const std::vector<int> i_neighbor = {1, -1, 0, 0};
    const std::vector<int> j_neighbor = {0, 0, 1, -1};

    for (int k = 0; k < 4; k++) {
      const int next_i = i + i_neighbor[k];
      const int next_j = j + j_neighbor[k];
      if (next_i >= 0 && next_i < mask.rows && next_j >= 0 &&
          next_j < mask.cols && !mask.at<uchar>(next_i, next_j)) {
        qi.push(next_i);
        qj.push(next_j);
        mask.at<uchar>(next_i, next_j) = 1;
      }
    }
  }

  throw std::runtime_error("No valid point in entire abs");
}

AbsCalibration::AbsCalibration(const Abs& abs) : abs(abs) {
  const std::vector<cv::Point3d> points = abs.to_points();
  const std::vector<cv::Point2d> pixels = abs.to_pixels();

  // compute mean
  mean = cv::Point3d(0, 0, 0);
  for (cv::Point3d point : points) mean += point;
  mean /= static_cast<double>(points.size());

  assert(pixels.size() == points.size());
  const int max_points = points.size();
  cv::Mat X(max_points * 2, 11, CV_64F), Y(max_points * 2, 1, CV_64F);
  int n = 0;
  for (int i = 0, len = points.size(); i < len; i++, n += 2) {
    Y.at<double>(n, 0) = pixels[i].x;
    Y.at<double>(n + 1, 0) = pixels[i].y;

    X.at<double>(n, 0) = (points[i].x - mean.x);
    X.at<double>(n, 1) = (points[i].y - mean.y);
    X.at<double>(n, 2) = (points[i].z - mean.z);
    X.at<double>(n, 3) = 1.0;
    X.at<double>(n, 4) = 0.0;
    X.at<double>(n, 5) = 0.0;
    X.at<double>(n, 6) = 0.0;
    X.at<double>(n, 7) = 0.0;
    X.at<double>(n, 8) = -pixels[i].x * (points[i].x - mean.x);
    X.at<double>(n, 9) = -pixels[i].x * (points[i].y - mean.y);
    X.at<double>(n, 10) = -pixels[i].x * (points[i].z - mean.z);

    X.at<double>(n + 1, 0) = 0.0;
    X.at<double>(n + 1, 1) = 0.0;
    X.at<double>(n + 1, 2) = 0.0;
    X.at<double>(n + 1, 3) = 0.0;
    X.at<double>(n + 1, 4) = (points[i].x - mean.x);
    X.at<double>(n + 1, 5) = (points[i].y - mean.y);
    X.at<double>(n + 1, 6) = (points[i].z - mean.z);
    X.at<double>(n + 1, 7) = 1.0;
    X.at<double>(n + 1, 8) = -pixels[i].y * (points[i].x - mean.x);
    X.at<double>(n + 1, 9) = -pixels[i].y * (points[i].y - mean.y);
    X.at<double>(n + 1, 10) = -pixels[i].y * (points[i].z - mean.z);
  }

  cv::Mat tmp_c = (X.t() * X).inv(cv::DECOMP_LU) * X.t() * Y;

  cmatrix.create(3, 4, CV_64F);
  for (int i = 0; i < 11; i++)
    cmatrix.at<double>(i / 4, i % 4) = tmp_c.at<double>(i, 0);
  cmatrix.at<double>(2, 3) = 1.0;
}

cv::Point3d AbsCalibration::depth_to_xyz(const float x, const float y,
                                         const float z) const {
  return abs.at(y, x);
}

cv::Point AbsCalibration::xyz_to_depth(const cv::Point3d& p) const {
  cv::Point2d ret;
  const double q = (p.x - mean.x) * cmatrix.at<double>(2, 0) +
                   (p.y - mean.y) * cmatrix.at<double>(2, 1) +
                   (p.z - mean.z) * cmatrix.at<double>(2, 2) +
                   cmatrix.at<double>(2, 3);
  ret.y =
      ((p.x - mean.x) * cmatrix.at<double>(0, 0) +
       (p.y - mean.y) * cmatrix.at<double>(0, 1) +
       (p.z - mean.z) * cmatrix.at<double>(0, 2) + cmatrix.at<double>(0, 3)) /
      q;
  ret.x =
      ((p.x - mean.x) * cmatrix.at<double>(1, 0) +
       (p.y - mean.y) * cmatrix.at<double>(1, 1) +
       (p.z - mean.z) * cmatrix.at<double>(1, 2) + cmatrix.at<double>(1, 3)) /
      q;
  return ret;
}
