#include "hoa.hpp"
#include "points_file.hpp"

Eigen::Matrix<double, Eigen::Dynamic, 3> ear::hoa::load_points() {
  Eigen::Matrix<double, Eigen::Dynamic, 3> points(points_file::num_points, 3);

  for (size_t i = 0; i < points_file::num_points; i++) {
    double phi = points_file::points[i][0], theta = points_file::points[i][1];
    points(i, 0) = std::sin(theta) * std::cos(phi);
    points(i, 1) = std::sin(theta) * std::sin(phi);
    points(i, 2) = std::cos(theta);
  }

  return points;
}
