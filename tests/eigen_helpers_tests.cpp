#include <Eigen/Core>
#include <catch2/catch.hpp>
#include "ear/common/helpers/eigen_helpers.hpp"

using namespace ear;

TEST_CASE("interp") {
  // x lower than xp(0)
  REQUIRE(interp(-1.0, Eigen::Vector3d{0.0, 1.0, 5.0},
                 Eigen::Vector3d{0.0, 1.0, 3.0}) == 0.0);
  // x equal to xp(0)
  REQUIRE(interp(0.0, Eigen::Vector3d{0.0, 1.0, 5.0},
                 Eigen::Vector3d{0.0, 1.0, 3.0}) == 0.0);
  // x between xp(0) and xp(-1)
  REQUIRE(interp(0.5, Eigen::Vector3d{0.0, 1.0, 5.0},
                 Eigen::Vector3d{0.0, 1.0, 3.0}) == 0.5);
  // x equal to xp(-1)
  REQUIRE(interp(5.0, Eigen::Vector3d{0.0, 1.0, 5.0},
                 Eigen::Vector3d{0.0, 1.0, 3.0}) == 3.0);
  // x higher than xp(-1)
  REQUIRE(interp(10.0, Eigen::Vector3d{0.0, 1.0, 5.0},
                 Eigen::Vector3d{0.0, 1.0, 3.0}) == 3.0);
}

TEST_CASE("copy_vector") {
  std::vector<bool> b{false, true, false};
  auto b_eigen_exp = Eigen::Array<bool, 3, 1>{false, true, false};

  auto b_eigen = copy_vector<Eigen::Array<bool, Eigen::Dynamic, 1>>(b);
  auto b_eigen_sz = copy_vector<Eigen::Array<bool, 3, 1>>(b);

  REQUIRE((b_eigen == b_eigen_exp).all());
  REQUIRE((b_eigen_sz == b_eigen_exp).all());
}

TEST_CASE("mask_write") {
  Eigen::Vector3d out = Eigen::Vector3d::Zero();
  std::vector<bool> mask{false, true, true};
  Eigen::Vector2d values(2.0, 5.0);
  Eigen::Vector3d expected(0.0, 2.0, 5.0);

  mask_write(out, mask, values);

  REQUIRE(out == expected);
}
