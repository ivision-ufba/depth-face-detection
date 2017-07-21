#include "face_detector.hpp"
#include <queue>

#define RESOLUTION 0.127272727
#define WIDTH 1800
#define HEIGHT 1600
#define FACE_SIZE 21
#define PATH_CASCADE_FACE "misc/ALL_Spring2003_3D.xml"
#define THRESHOLD 1200
#define STEP 6

FaceDetector::FaceDetector()
    : resolution(RESOLUTION),
      projection_width(resolution * WIDTH),
      projection_height(resolution * HEIGHT),
      CX(projection_width / 2),
      CY(projection_height / 2),
      face_size(FACE_SIZE),
      step(STEP),
      threshold(THRESHOLD) {
  // open haar cascade
  face_cascade = (CvHaarClassifierCascade*)cvLoad(PATH_CASCADE_FACE, 0, 0, 0);
  if (!face_cascade) throw std::runtime_error("Error: unable to find cascade");

  // allocate images
  projection = cvCreateImage(cvSize(projection_width, projection_height),
                             IPL_DEPTH_64F, 1);
  mark = cvCreateImage(cvSize(projection_width, projection_height),
                       IPL_DEPTH_8U, 1);
  sum = cvCreateImage(cvSize(projection_width + 1, projection_height + 1),
                      IPL_DEPTH_64F, 1);
  squared_sum = cvCreateImage(
      cvSize(projection_width + 1, projection_height + 1), IPL_DEPTH_64F, 1);
  tilted_sum = cvCreateImage(
      cvSize(projection_width + 1, projection_height + 1), IPL_DEPTH_64F, 1);
  sum_int = cvCreateImage(cvSize(projection_width + 1, projection_height + 1),
                          IPL_DEPTH_32S, 1);
  tilted_sum_int = cvCreateImage(
      cvSize(projection_width + 1, projection_height + 1), IPL_DEPTH_32S, 1);
  mark_sum = cvCreateImage(cvSize(projection_width + 1, projection_height + 1),
                           IPL_DEPTH_32S, 1);
}

FaceDetector::~FaceDetector() {
  // release images
  cvReleaseImage(&projection);
  cvReleaseImage(&mark);
  cvReleaseImage(&sum);
  cvReleaseImage(&squared_sum);
  cvReleaseImage(&tilted_sum);
  cvReleaseImage(&sum_int);
  cvReleaseImage(&tilted_sum_int);
  cvReleaseImage(&mark_sum);

  // release cascade
  cvReleaseHaarClassifierCascade(&face_cascade);
}

void FaceDetector::compute_projection(const std::vector<cv::Point3d>& points,
                                      double rotation_matrix[3][3]) {
  /* computes a projection in 'projection' of the 3D points in 'points' after
   * rotating them by 'rotation_matrix'.
   * - 'points': vector of 3D points;
   * - 'rotation_matrix': matrix to rotate 3D points by prior to projecting. */

  cvSet(mark, cvRealScalar(0));

  for (const cv::Point3d& point : points) {
    // compute rotated coordinates
    const unsigned int x = CX + cvRound(point.x * rotation_matrix[0][0] +
                                        point.y * rotation_matrix[0][1] +
                                        point.z * rotation_matrix[0][2]);
    const unsigned int y = CY - cvRound(point.x * rotation_matrix[1][0] +
                                        point.y * rotation_matrix[1][1] +
                                        point.z * rotation_matrix[1][2]);
    const double z = point.x * rotation_matrix[2][0] +
                     point.y * rotation_matrix[2][1] +
                     point.z * rotation_matrix[2][2];

    // point falls into specified projection area
    if (x >= 0 && x < projection_width && y >= 0 && y < projection_height &&
        (!CV_IMAGE_ELEM(mark, uchar, y, x) ||
         z > CV_IMAGE_ELEM(projection, double, y, x))) {
      CV_IMAGE_ELEM(projection, double, y, x) = z;
      CV_IMAGE_ELEM(mark, uchar, y, x) = 1;
    }
  }
}

void FaceDetector::fill_holes() {
  /* fills holes in the projection image through the propagation of valid known
   * pixels. */

  // find filled pixels
  std::queue<unsigned int> row_queue, col_queue, c_queue;
  for (unsigned int i = 1; i < projection_width - 1; i++)
    for (unsigned int j = 1; j < projection_height - 1; j++)
      if (!CV_IMAGE_ELEM(mark, uchar, i, j) &&
          (CV_IMAGE_ELEM(mark, uchar, i, j - 1) ||
           CV_IMAGE_ELEM(mark, uchar, i, j + 1) ||
           CV_IMAGE_ELEM(mark, uchar, i - 1, j) ||
           CV_IMAGE_ELEM(mark, uchar, i + 1, j))) {
        row_queue.push(i);
        col_queue.push(j);
        c_queue.push(1);
      }

  // propagate information from filled pixels
  while (!row_queue.empty()) {
    unsigned int row = row_queue.front();
    unsigned int col = col_queue.front();
    unsigned int c = c_queue.front();

    row_queue.pop();
    col_queue.pop();
    c_queue.pop();

    // attempt to fill pixel
    if (!CV_IMAGE_ELEM(mark, uchar, row, col) && row > 0 && col > 0 &&
        row < projection_height - 1 && col < projection_width - 1 &&
        c < face_size / 2) {
      // mark pixel as visited
      CV_IMAGE_ELEM(mark, uchar, row, col) = c + 1;

      // visit neighborhood of pixel
      unsigned int valid_neighbors = 0;
      double z_neighbor_sum = 0;
      std::vector<int> row_neighbors = {0, 0, -1, 1};
      std::vector<int> col_neighbors = {-1, 1, 0, 0};
      for (int i = 0; i < 4; i++) {
        const uchar current_pixel = CV_IMAGE_ELEM(
            mark, uchar, row + row_neighbors[i], col + col_neighbors[i]);

        if (current_pixel && current_pixel <= c) {
          valid_neighbors++;
          z_neighbor_sum +=
              CV_IMAGE_ELEM(projection, double, row + row_neighbors[i],
                            col + col_neighbors[i]);
        } else {
          row_queue.push(row + row_neighbors[i]);
          col_queue.push(col + col_neighbors[i]);
          c_queue.push(c + 1);
        }
      }

      // fill pixel with average of valid neighbors
      CV_IMAGE_ELEM(projection, double, row, col) =
          z_neighbor_sum / valid_neighbors;
    }
  }

  // final adjustments
  for (unsigned int i = 0; i < projection_height; i++)
    for (unsigned int j = 0; j < projection_width; j++) {
      if (CV_IMAGE_ELEM(mark, uchar, i, j))
        CV_IMAGE_ELEM(mark, uchar, i, j) = 1;
      else
        CV_IMAGE_ELEM(projection, double, i, j) = -threshold;
    }
}

std::vector<cv::Point3d> FaceDetector::grid_sample(
    const cv::Mat& depth_img, const Calibration& calibration,
    const double threshold, const unsigned int step) const {
  /* generates a point cloud from a grid sample of a depth image.
   * - 'depth_img': depth image to be sampled;
   * - 'calibration': calibration object;
   * - 'threshold': threshold for discarding points;
   * - 'step': distance between grid points;
   * - returns: 3D points sampled in grid fashion from the depth image. */

  std::vector<cv::Point3d> points;
  for (int i = 0; i < depth_img.rows; i += step)
    for (int j = 0; j < depth_img.cols; j += step) {
      const cv::Point3d point =
          calibration.depth_to_xyz(j, i, depth_img.at<float>(i, j));

      if (point.z < -1.0 && point.z > -threshold)
        points.push_back(resolution * point);
    }

  return points;
}

cv::Point3d FaceDetector::retrieve_3d_point(const unsigned int row,
                                            const unsigned int col) const {
  /* retrieves the 3d point corresponding to a given pixel through the inverse
   * transformation.
   * - 'row': pixel row;
   * - 'col': pixel column;
   * - returns: 3D point corresponding to pixel (row, col). */

  const double x = (col + face_size / 2 - CX) / resolution;
  const double y = (CY - row - face_size / 2) / resolution;
  const double z = (CV_IMAGE_ELEM(sum, double, row + face_size / 2 + 6,
                                  col + face_size / 2 + 6) -
                    CV_IMAGE_ELEM(sum, double, row + face_size / 2 - 5,
                                  col + face_size / 2 + 6) -
                    CV_IMAGE_ELEM(sum, double, row + face_size / 2 + 6,
                                  col + face_size / 2 - 5) +
                    CV_IMAGE_ELEM(sum, double, row + face_size / 2 - 5,
                                  col + face_size / 2 - 5)) /
                   (121.0 * resolution);
  return cv::Point3d(x, y, z);
}

void FaceDetector::compute_rotation_matrix(double rotation_matrix[3][3],
                                           const double x_angle,
                                           const double y_angle,
                                           const double z_angle) const {
  /* computes rotation matrix for given angles.
   * - 'rotation_matrix': rotation matrix for the given angles;
   * - 'x_angle': given angle for the x axis;
   * - 'y_angle': given angle for the y axis;
   * - 'z_angle': given angle for the z axis; */

  const double cos_x = cos(x_angle);
  const double cos_y = cos(y_angle);
  const double cos_z = cos(z_angle);
  const double sinX = sin(x_angle);
  const double sinY = sin(y_angle);
  const double sinZ = sin(z_angle);

  rotation_matrix[0][0] = cos_z * cos_y + sinZ * sinX * sinY;
  rotation_matrix[0][1] = sinZ * cos_y - cos_z * sinX * sinY;
  rotation_matrix[0][2] = cos_x * sinY;
  rotation_matrix[1][0] = -sinZ * cos_x;
  rotation_matrix[1][1] = cos_z * cos_x;
  rotation_matrix[1][2] = sinX;
  rotation_matrix[2][0] = sinZ * sinX * cos_y - cos_z * sinY;
  rotation_matrix[2][1] = -cos_z * sinX * cos_y - sinZ * sinY;
  rotation_matrix[2][2] = cos_x * cos_y;
}

std::vector<cv::Point3d> FaceDetector::helper(
    const std::vector<cv::Point3d>& points, const double x_angle,
    const double y_angle, const double z_angle) {
  /* detects faces in projection image after rotation.
   * - 'points': cloud point to be projected;
   * - 'x_angle': x rotation angle;
   * - 'y_angle': y rotation angle;
   * - 'z_angle': z rotation angle;
   * - returns: faces detected in this angle. */

  double rotation_matrix[3][3];
  compute_rotation_matrix(rotation_matrix, x_angle * CV_PI / 180.0,
                          y_angle * CV_PI / 180.0, z_angle * CV_PI / 180.0);

  compute_projection(points, rotation_matrix);
  fill_holes();

  // compute integral images for efficiency
  cvIntegral(projection, sum, squared_sum, tilted_sum);
  cvIntegral(mark, mark_sum, NULL, NULL);
  for (unsigned int i = 0; i <= projection_height; i++)
    for (unsigned int j = 0; j <= projection_width; j++) {
      CV_IMAGE_ELEM(sum_int, int, i, j) = CV_IMAGE_ELEM(sum, double, i, j);
      CV_IMAGE_ELEM(tilted_sum_int, int, i, j) =
          CV_IMAGE_ELEM(tilted_sum, double, i, j);
    }

  cvSetImagesForHaarClassifierCascade(face_cascade, sum_int, squared_sum,
                                      tilted_sum_int, 1.0);

  // run detector
  std::vector<cv::Point3d> detections;
  for (unsigned int i = 0; i <= projection_height - face_size; i++)
    for (unsigned int j = 0; j <= projection_width - face_size; j++) {
      // efficiently retrieve sum of face_size window
      const unsigned int window_mark_sum =
          CV_IMAGE_ELEM(mark_sum, int, i + face_size, j + face_size) -
          CV_IMAGE_ELEM(mark_sum, int, i, j + face_size) -
          CV_IMAGE_ELEM(mark_sum, int, i + face_size, j) +
          CV_IMAGE_ELEM(mark_sum, int, i, j);

      // detection if window sum has correct value and haar cascade is positive
      if (window_mark_sum == face_size * face_size &&
          cvRunHaarClassifierCascade(face_cascade, cvPoint(j, i), 0) > 0) {
        const cv::Point3d rotated_point = retrieve_3d_point(i, j);

        // restore original coordinate system
        const double x = rotated_point.x * rotation_matrix[0][0] +
                         rotated_point.y * rotation_matrix[1][0] +
                         rotated_point.z * rotation_matrix[2][0];
        const double y = rotated_point.x * rotation_matrix[0][1] +
                         rotated_point.y * rotation_matrix[1][1] +
                         rotated_point.z * rotation_matrix[2][1];
        const double z = rotated_point.x * rotation_matrix[0][2] +
                         rotated_point.y * rotation_matrix[1][2] +
                         rotated_point.z * rotation_matrix[2][2];

        detections.emplace_back(x, y, z);
      }
    }

  return detections;
}

cv::Rect FaceDetector::project(const cv::Point3d& detection_point,
                               const Calibration& calibration) const {
  /* projects a detection point into a rectangle.
   * - 'detection_point': 3D coordinates of point;
   * - returns: projected rectangle of detection. */

  const cv::Point detection_br =
      calibration.xyz_to_depth(detection_point + cv::Point3d(105, 105, 0));
  const cv::Point detection_tl =
      calibration.xyz_to_depth(detection_point - cv::Point3d(105, 105, 0));

  return cv::Rect(detection_tl, detection_br);
}

std::vector<cv::Rect> FaceDetector::project_and_merge(
    const std::vector<cv::Point3d>& detection_points,
    const Calibration& calibration) const {
  /* projects detections into depth image and merges multiple detections.
   * - 'detection_points': detections in 3D points;
   * - returns: merged and projected detections as rectangles. */

  std::vector<cv::Rect> merged_detections;
  for (const cv::Point3d& detection_point : detection_points) {
    const cv::Point detection_pixel = calibration.xyz_to_depth(detection_point);

    // if point projects into previously projected detection, merge
    bool to_merge = false;
    for (const cv::Rect& detection : merged_detections)
      if (detection.contains(detection_pixel)) {
        to_merge = true;
        break;
      }

    if (!to_merge)
      merged_detections.push_back(project(detection_point, calibration));
  }

  return merged_detections;
}

std::vector<cv::Rect> FaceDetector::range_detect(
    const cv::Mat& _depth_img, const Calibration& calibration,
    const int min_x_angle, const int min_y_angle, const int min_z_angle,
    const int max_x_angle, const int max_y_angle, const int max_z_angle) {
  /* detects faces in intensity depth image in angle ranges.
   * - 'depth_img': depth intensity image;
   * - 'calibration': calibration interface;
   * - 'min_x_angle': x angle interval lower bound;
   * - 'min_y_angle': y angle interval lower bound;
   * - 'min_z_angle': z angle interval lower bound;
   * - 'max_x_angle': x angle interval upper bound;
   * - 'max_y_angle': y angle interval upper bound;
   * - 'max_z_angle': z angle interval upper bound;
   * - returns: 3D point coordinates of face detections. */

  // convert depth image to float for compatibility
  cv::Mat depth_img;
  _depth_img.convertTo(depth_img, CV_32F);

  std::vector<cv::Point3d> points =
      grid_sample(depth_img, calibration, threshold, step);

  std::vector<cv::Point3d> detections;
  for (int x_angle = min_x_angle; x_angle <= max_x_angle; x_angle += 10)
    for (int y_angle = min_y_angle; y_angle <= max_y_angle; y_angle += 10)
      for (int z_angle = min_z_angle; z_angle <= max_z_angle; z_angle += 10)
        if (x_angle + y_angle + z_angle <= 30) {
          const std::vector<cv::Point3d> new_detections =
              helper(points, x_angle, y_angle, z_angle);
          detections.insert(detections.end(), new_detections.begin(),
                            new_detections.end());
        }

  return project_and_merge(detections, calibration);
}

std::vector<cv::Rect> FaceDetector::detect_frontal(
    const cv::Mat& _depth_img, const Calibration& calibration) {
  /* detects frontal faces in intesity depth image.
   * - 'depth_img': depth intensity image;
   * - 'calibration': calibration interface;
   * - returns: 3D point coordinates of face detections. */

  // convert depth image to float for compatibility
  cv::Mat depth_img;
  _depth_img.convertTo(depth_img, CV_32F);

  std::vector<cv::Point3d> points =
      grid_sample(depth_img, calibration, threshold, step);

  return project_and_merge(helper(points, 0, 0, 0), calibration);
}
