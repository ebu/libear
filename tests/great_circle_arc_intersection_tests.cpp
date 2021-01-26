#include <Eigen/Core>
#include <boost/math/constants/constants.hpp>
#include <catch2/catch.hpp>

#include "object_based/great_circle_arc_intersection.hpp"

using namespace ear;

namespace {
  const double PI = boost::math::constants::pi<double>();
}

void check_basic(const GreatCircleArcIntersection o) {
  double x = std::sqrt(0.5);

  REQUIRE(o.intersects(Eigen::Vector3d{0, 1, 0}, Eigen::Vector3d{0, 0, 1}));
  REQUIRE(o.intersects(Eigen::Vector3d{1, 0, 0}, Eigen::Vector3d{0, 0, 1}));
  REQUIRE(o.intersects(Eigen::Vector3d{-1, 0, 0}, Eigen::Vector3d{0, 0, 1}));
  REQUIRE(!o.intersects(Eigen::Vector3d{0, -1, 0}, Eigen::Vector3d{0, 0, 1}));

  REQUIRE(
      o.intersects(Eigen::Vector3d{0, x, x - 0.01}, Eigen::Vector3d{0, 0, 1}));
  REQUIRE(
      !o.intersects(Eigen::Vector3d{0, x, x + 0.01}, Eigen::Vector3d{0, 0, 1}));
  REQUIRE(
      !o.intersects(Eigen::Vector3d{0, x, x - 0.01}, Eigen::Vector3d{0, 1, 0}));
  REQUIRE(
      o.intersects(Eigen::Vector3d{0, x, x + 0.01}, Eigen::Vector3d{0, 1, 0}));

  REQUIRE(o.intersects(Eigen::Vector3d{x, 0, x + 0.01},
                       Eigen::Vector3d{-x, 0, x - 0.01}));
  REQUIRE(o.intersects(Eigen::Vector3d{x, 0, x - 0.01},
                       Eigen::Vector3d{-x, 0, x + 0.01}));
  REQUIRE(!o.intersects(Eigen::Vector3d{x, 0, x + 0.01},
                        Eigen::Vector3d{-x, 0, x + 0.01}));

  REQUIRE(!o.intersects(Eigen::Vector3d{0.1, x, x + 0.001},
                        Eigen::Vector3d{0.05, x, x + 0.001}));
  REQUIRE(o.intersects(Eigen::Vector3d{-0.1, x, x + 0.001},
                       Eigen::Vector3d{0.1, x, x + 0.001}));

  // wrong direction completely;
  REQUIRE(!o.intersects(Eigen::Vector3d{x, 0, -x}, Eigen::Vector3d{-x, 0, -x}));
}

TEST_CASE("basic") {
  GreatCircleArcIntersection o(Eigen::Vector3d{0.0, 1.0, 0.0},
                               Eigen::Vector3d{1.0, 0.0, 0.0}, PI / 4, PI / 2);
  check_basic(o);
}

TEST_CASE("wide") {
  // exactly the same as above, but specified upside-down
  GreatCircleArcIntersection o(Eigen::Vector3d{0.0, 1.0, 0.0},
                               Eigen::Vector3d{-1.0, 0.0, 0.0}, PI - PI / 4,
                               PI / 2);
  check_basic(o);
}
