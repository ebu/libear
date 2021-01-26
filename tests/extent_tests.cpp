#include <Eigen/Core>
#include <boost/make_unique.hpp>
#include <catch2/catch.hpp>
#include <memory>
#include <tuple>
#include <vector>
#include "common/geom.hpp"
#include "common/helpers/eigen_helpers.hpp"
#include "ear/bs2051.hpp"
#include "eigen_utils.hpp"
#include "object_based/extent.hpp"
#include "reference/extent.hpp"

using namespace ear;

TEST_CASE("test_AngleToWeight") {
  for (int start_angle = 0; start_angle <= 180; start_angle += 2) {
    int end_angle = start_angle + 10;
    AngleToWeight atw(radians(start_angle), radians(end_angle));

    Eigen::Vector2d interp_from{radians(start_angle), radians(end_angle)};
    Eigen::Vector2d interp_to{1, 0};

    for (int angle = 0; angle < 180; angle++) {
      double angle_rad = radians(angle);

      CHECK(atw.from_cos(std::cos(angle_rad)) ==
            Approx(interp(angle_rad, interp_from, interp_to)));
      CHECK(atw.from_sin(std::sin(angle_rad)) ==
            Approx(interp(angle_rad, interp_from, interp_to)));
    }
  }
}

TEST_CASE("test_basis") {
  double eps = 1e-6;

  Eigen::Matrix3d expected;
  // cardinal directions
  expected = Eigen::Matrix3d::Identity();
  REQUIRE(calcBasis(cart(0.0, 0.0, 1.0)).isApprox(expected, eps));
  expected << 0, 1, 0,  //
      -1, 0, 0,  //
      0, 0, 1;
  REQUIRE(calcBasis(cart(90.0, 0.0, 1.0)).isApprox(expected, eps));
  expected << 0, -1, 0,  //
      1, 0, 0,  //
      0, 0, 1;
  REQUIRE(calcBasis(cart(-90.0, 0.0, 1.0)).isApprox(expected, eps));
  expected << -1, 0, 0,  //
      0, -1, 0,  //
      0, 0, 1;
  REQUIRE(calcBasis(cart(180.0, 0.0, 1.0)).isApprox(expected, eps));
  expected << 1, 0, 0,  //
      0, 0, 1,  //
      0, -1, 0;
  REQUIRE(calcBasis(cart(0.0, 90.0, 1.0)).isApprox(expected, eps));
  expected << 1, 0, 0,  //
      0, 0, -1,  //
      0, 1, 0;
  REQUIRE(calcBasis(cart(0.0, -90.0, 1.0)).isApprox(expected, eps));

  // slight offset from pole should behave as if pointing forwards
  expected << 1, 0, 0,  //
      0, 0, 1,  //
      0, -1, 0;
  REQUIRE(calcBasis(cart(90.0, 90.0 - 1e-6, 1.0)).isApprox(expected, eps));
  expected << 1, 0, 0,  //
      0, 0, -1,  //
      0, 1, 0;
  REQUIRE(calcBasis(cart(90.0, -90.0 + 1e-6, 1.0)).isApprox(expected, eps));
}

TEST_CASE("test_weight_func") {
  double fade = 10.0;
  double height = 10.0;

  double width;
  double azimuth;
  double expected;
  double actual;
  Eigen::Vector3d point;
  for (auto entry : {std::make_pair(20.0, 0.0), std::make_pair(360.0, 0.0),
                     std::make_pair(360.0, 180.0)}) {
    std::tie(width, azimuth) = entry;
    Eigen::VectorXd elevations = Eigen::VectorXd::LinSpaced(50, -90.0, 90.0);
    for (double elevation : elevations) {
      expected = interp(elevation,
                        Eigen::Vector4d{-(height / 2 + fade), -height / 2,
                                        height / 2, height / 2 + fade},
                        Eigen::Vector4d{0, 1, 1, 0});

      point = cart(azimuth, elevation, 1.0);
      WeightingFunction weightingFunc(cart(0.0, 0.0, 1.0), width, height);
      actual = weightingFunc(point.transpose());
      REQUIRE(actual == Approx(expected));
      // Swapped
      point = cart(elevation, azimuth, 1.0);
      WeightingFunction weightingFuncSwap(cart(0.0, 0.0, 1.0), height, width);
      actual = weightingFuncSwap(point.transpose());
      REQUIRE(actual == Approx(expected));
    }
  }
  Eigen::VectorXd azimuths = Eigen::VectorXd::LinSpaced(50, -180, 180);
  for (double azimuth : azimuths) {
    double expected = interp(azimuth,
                             Eigen::Vector4d{-(width / 2 + fade), -width / 2,
                                             width / 2, width / 2 + fade},
                             Eigen::Vector4d{0, 1, 1, 0});

    point = cart(azimuth, 0.0, 1.0);
    WeightingFunction weightingFunc(cart(0.0, 0.0, 1.0), width, height);
    actual = weightingFunc(point.transpose());
    REQUIRE(actual == Approx(expected));
    // Swapped
    point = cart(0.0, azimuth, 1.0);
    WeightingFunction weightingFuncSwap(cart(0.0, 0.0, 1.0), height, width);
    actual = weightingFuncSwap(point.transpose());
    REQUIRE(actual == Approx(expected));
  }
}

TEST_CASE("test_pv") {
  Layout layout = getLayout("9+10+3").withoutLfe();
  std::shared_ptr<PointSourcePanner> psp = configurePolarPanner(layout);
  PolarExtentPanner extentPanner(psp);

  REQUIRE(extentPanner.calcPvSpread(cart(0.0, 0.0, 1.0), 0.0, 0.0) ==
          psp->handle(cart(0.0, 0.0, 1.0)).get());
  REQUIRE(extentPanner.calcPvSpread(cart(10.0, 20.0, 1.0), 0.0, 0.0) ==
          psp->handle(cart(10.0, 20.0, 1.0)).get());

  std::vector<std::pair<Eigen::Vector3d, double>> positions{
      std::make_pair(cart(0.0, 0.0, 1.0), 1e-10),
      std::make_pair(cart(30.0, 10.0, 1.0), 1e-2)};
  for (auto position : positions) {
    Eigen::Vector3d pos = position.first;
    double tol = position.second;
    Eigen::VectorXd spread_pv = extentPanner.calcPvSpread(pos, 20.0, 10.0);
    REQUIRE(spread_pv.norm() == Approx(1.0));
    Eigen::VectorXd vv =
        spread_pv.transpose() * toPositionsMatrix(layout.positions());
    vv /= vv.norm();
    REQUIRE(vv.isApprox(pos, tol));
  }
}

TEST_CASE("reference_weight") {
  for (size_t i = 0; i < 1000; i++) {
    Eigen::Vector3d pos = Eigen::Vector3d::Random();
    pos.normalize();
    Eigen::Vector2d size = (Eigen::Vector2d::Random().array() + 1) * 180;
    double width = size(0);
    double height = size(1);

    INFO("size " << width << " " << height);
    INFO("pos " << pos.transpose());

    WeightingFunction wf(pos, width, height);
    reference::WeightingFunction wf_reference(pos, width, height);

    for (size_t i = 0; i < 500; i++) {
      Eigen::Vector3d sample_pos = Eigen::Vector3d::Random();
      sample_pos.normalize();
      double weight = wf(sample_pos.transpose());
      double weight_reference = wf_reference(sample_pos.transpose());
      CHECK(weight == Approx(weight_reference));
    }
  }
}

TEST_CASE("reference") {
  Layout layout = getLayout("9+10+3").withoutLfe();
  std::shared_ptr<PointSourcePanner> psp = configurePolarPanner(layout);
  PolarExtentPanner extentPanner(psp,
                                 boost::make_unique<SpreadingPanner>(
                                     psp, PolarExtentPanner::nRowsDefault));
  reference::PolarExtentPanner extentPannerReference(psp);

  for (size_t i = 0; i < 1000; i++) {
    Eigen::Vector3d pos = Eigen::Vector3d::Random();
    pos.normalize();
    Eigen::Vector2d size = (Eigen::Vector2d::Random().array() + 1) * 180;
    double width = size(0);
    double height = size(1);
    Eigen::VectorXd spread_pv = extentPanner.calcPvSpread(pos, width, height);
    Eigen::VectorXd spread_pv_reference =
        extentPannerReference.calcPvSpread(pos, width, height);

    INFO("size " << width << " " << height);
    INFO("spread_pv " << spread_pv.transpose());
    INFO("spread_pv_reference " << spread_pv_reference.transpose());
    CHECK(spread_pv.isApprox(spread_pv_reference, 1e-6));
  }
}
