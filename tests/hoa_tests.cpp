#include <catch2/catch.hpp>
#include "eigen_utils.hpp"
#include "hoa/hoa.hpp"

using namespace ear::hoa;

TEST_CASE("load_points") {
  auto points = load_points();
  CHECK_THAT(points.row(0), IsApprox(Eigen::RowVector3d(-0.007238307489963788,
                                                        0.8575241661297168,
                                                        0.5143927598714958)));
  CHECK_THAT(
      points.row(points.rows() - 1),
      IsApprox(Eigen::RowVector3d(-0.5784605915309438, 0.5246941913018843,
                                  -0.6245633271812653)));
}
