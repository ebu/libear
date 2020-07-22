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
#include "object_based/polar_extent.hpp"
#include "reference/extent.hpp"

using namespace ear;

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
  // this isn't exposed by the real implementation; test the reference version,
  // which is checked against the real implementation in same_as_reference
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
      reference::WeightingFunction weightingFunc(cart(0.0, 0.0, 1.0), width,
                                                 height);
      actual = weightingFunc(point.transpose());
      REQUIRE(actual == Approx(expected));
      // Swapped
      point = cart(elevation, azimuth, 1.0);
      reference::WeightingFunction weightingFuncSwap(cart(0.0, 0.0, 1.0),
                                                     height, width);
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
    reference::WeightingFunction weightingFunc(cart(0.0, 0.0, 1.0), width,
                                               height);
    actual = weightingFunc(point.transpose());
    REQUIRE(actual == Approx(expected));
    // Swapped
    point = cart(0.0, azimuth, 1.0);
    reference::WeightingFunction weightingFuncSwap(cart(0.0, 0.0, 1.0), height,
                                                   width);
    actual = weightingFuncSwap(point.transpose());
    REQUIRE(actual == Approx(expected));
  }
}

TEST_CASE("test_pv") {
  Layout layout = getLayout("9+10+3").withoutLfe();
  std::shared_ptr<PointSourcePanner> psp = configurePolarPanner(layout);
  PolarExtent extentPanner(psp);
  Eigen::VectorXd pvs{psp->numberOfOutputChannels()};

  extentPanner.handle(cart(0.0, 0.0, 1.0), 0.0, 0.0, 0.0, pvs);
  REQUIRE(pvs == psp->handle(cart(0.0, 0.0, 1.0)).get());
  extentPanner.handle(cart(10.0, 20.0, 1.0), 0.0, 0.0, 0.0, pvs);
  REQUIRE(pvs == psp->handle(cart(10.0, 20.0, 1.0)).get());

  std::vector<std::pair<Eigen::Vector3d, double>> positions{
      std::make_pair(cart(0.0, 0.0, 1.0), 1e-5),
      std::make_pair(cart(30.0, 10.0, 1.0), 1e-2)};
  for (auto position : positions) {
    Eigen::Vector3d pos = position.first;
    double tol = position.second;
    extentPanner.handle(pos, 20.0, 10.0, 0.0, pvs);
    REQUIRE(pvs.norm() == Approx(1.0));
    Eigen::VectorXd vv =
        pvs.transpose() * toPositionsMatrix(layout.positions());
    vv /= vv.norm();
    REQUIRE(vv.isApprox(pos, tol));
  }
}

TEST_CASE("same_as_reference") {
  Layout layout = getLayout("9+10+3").withoutLfe();
  std::shared_ptr<PointSourcePanner> psp = configurePolarPanner(layout);

  PolarExtent extentPanner(psp);
  PolarExtent extentPannerScalar(psp, get_polar_extent_core_scalar());
  Eigen::VectorXd spread_pv{psp->numberOfOutputChannels()};

  reference::PolarExtentPanner extentPannerReference(psp);

  for (size_t i = 0; i < 1000; i++) {
    Eigen::Vector3d pos = Eigen::Vector3d::Random();
    pos.normalize();
    Eigen::Vector2d size = (Eigen::Vector2d::Random().array() + 1) * 180;
    double width = size(0);
    double height = size(1);
    Eigen::VectorXd spread_pv_reference =
        extentPannerReference.handle(pos, width, height, 0.0);

    INFO("size " << width << " " << height);
    INFO("spread_pv_reference " << spread_pv_reference.transpose());

    extentPanner.handle(pos, width, height, 0.0, spread_pv);
    INFO("spread_pv " << spread_pv.transpose());
    CHECK(spread_pv.isApprox(spread_pv_reference, 1e-5));

    extentPannerScalar.handle(pos, width, height, 0.0, spread_pv);
    INFO("spread_pv scalar " << spread_pv.transpose());
    CHECK(spread_pv.isApprox(spread_pv_reference, 1e-5));
  }
}

#ifdef CATCH_CONFIG_ENABLE_BENCHMARKING
TEST_CASE("bench_panner", "[.benchmark]") {
  Layout layout = getLayout("9+10+3").withoutLfe();
  std::shared_ptr<PointSourcePanner> psp = configurePolarPanner(layout);
  reference::PolarExtentPanner extentPannerRef(psp);

  PolarExtent extentPanner(psp);
  PolarExtent extentPannerScalar(psp, get_polar_extent_core_scalar());
  Eigen::VectorXd pvs{psp->numberOfOutputChannels()};

  Eigen::Vector3d pos(1, 1, 1);
  pos.normalize();

  std::vector<std::pair<double, double>> sizes = {
      {5, 5}, {30, 30}, {30, 50}, {90, 90}, {135, 135}, {360, 360},
  };

  for (auto &width_height : sizes) {
    double width, height;
    std::tie(width, height) = width_height;
    std::string size_name =
        std::to_string((int)width) + " " + std::to_string((int)height);

    BENCHMARK("ref " + size_name) {
      return extentPannerRef.handle(pos, width, height, 0.0);
    };

    BENCHMARK("impl scalar " + size_name) {
      return extentPannerScalar.handle(pos, width, height, 0.0, pvs);
    };

    BENCHMARK("impl " + size_name) {
      return extentPanner.handle(pos, width, height, 0.0, pvs);
    };
  }
}
#endif
