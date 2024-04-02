#include <Eigen/Core>
#include <catch2/catch.hpp>
#include <random>
#include <vector>
#include "common/geom.hpp"
#include "ear/common_types.hpp"

using namespace ear;

TEST_CASE("PolarPosition") {
  auto position = GENERATE(std::make_pair(0.0, 0.0), std::make_pair(0.0, 30.0),
                           std::make_pair(30.0, 0.0));
  double az = position.first;
  double el = position.second;
  REQUIRE(toCartesianVector3d(PolarPosition(az, el, 1.0))
              .isApprox(cart(az, el, 1.0)));
  REQUIRE(toCartesianVector3d(PolarPosition(az, el, 2.0))
              .isApprox(cart(az, el, 2.0)));
  REQUIRE(toNormalisedVector3d(PolarPosition(az, el, 2.0))
              .isApprox(cart(az, el, 1.0)));
}

TEST_CASE("cart") {
  REQUIRE(cart(0.0, 0.0, 1.0).isApprox(Eigen::Vector3d(0.0, 1.0, 0.0)));
  REQUIRE(cart(0.0, 0.0, 2.0).isApprox(Eigen::Vector3d(0.0, 2.0, 0.0)));
  REQUIRE(cart(0.0, 45.0, sqrt(2.0)).isApprox(Eigen::Vector3d(0.0, 1.0, 1.0)));
  REQUIRE(cart(45.0, 0.0, sqrt(2.0)).isApprox(Eigen::Vector3d(-1.0, 1.0, 0.0)));
}

TEST_CASE("test_azimuth") {
  REQUIRE(azimuth(Eigen::Vector3d(1.0, 1.0, 0.0)) == Approx(-45.0));
}

TEST_CASE("test_elevation") {
  REQUIRE(elevation(Eigen::Vector3d(0.0, 1.0, 1.0)) == Approx(45.0));
  REQUIRE(elevation(Eigen::Vector3d(std::sqrt(2.0), std::sqrt(2.0), 2.0)) ==
          Approx(45.0));
}

TEST_CASE("test_relative_angle") {
  REQUIRE(relativeAngle(0.0, 10.0) == 10.0);
  REQUIRE(relativeAngle(10.0, 10.0) == 10.0);
  REQUIRE(relativeAngle(11.0, 10.0) == 370.0);
  REQUIRE(relativeAngle(370.0, 10.0) == 370.0);
  REQUIRE(relativeAngle(371.0, 10.0) == 360.0 + 370.0);
}

TEST_CASE("test_inside_angle_range") {
  REQUIRE(insideAngleRange(0, 0, 10));
  REQUIRE(insideAngleRange(5, 0, 10));
  REQUIRE(insideAngleRange(10, 0, 10));

  REQUIRE(insideAngleRange(0, 10, 00));
  REQUIRE(!insideAngleRange(5, 10, 0));
  REQUIRE(insideAngleRange(15, 10, 0));
  REQUIRE(insideAngleRange(10, 10, 0));

  REQUIRE(insideAngleRange(0, -10, 10));
  REQUIRE(!insideAngleRange(180, -10, 10));
  REQUIRE(!insideAngleRange(-180, -10, 10));

  REQUIRE(insideAngleRange(0, -180, 180));
  REQUIRE(!insideAngleRange(0, -181, 181));
  REQUIRE(insideAngleRange(0, -180, 180, 1));

  REQUIRE(insideAngleRange(180, 180, -180));
  REQUIRE(insideAngleRange(180, 180, -180, 1));
  REQUIRE(!insideAngleRange(90, 180, -180));

  REQUIRE(insideAngleRange(0, 0, 0));
  REQUIRE(insideAngleRange(0, 0, 0, 1));
  REQUIRE(!insideAngleRange(90, 0, 0));

  REQUIRE(insideAngleRange(0, 1, 2, 2));
  REQUIRE(insideAngleRange(-1, 1, 2, 2));
  REQUIRE(insideAngleRange(359, 1, 2, 2));
}

/** @brief Are a and b the same, module some reversal or shift? */
bool inSameOrder(Eigen::MatrixXd a, Eigen::MatrixXd b) {
  // just produce all shifted and reversed versions of a, and compare
  // against b. Inefficient but simple.
  Eigen::MatrixXd a_shifted(a.rows(), a.cols());
  Eigen::MatrixXd a_shifted_rev(a.rows(), a.cols());
  Eigen::VectorXi indices(a.rows());
  for (int offset = 0; offset < a.rows(); ++offset) {
    a_shifted << a.bottomRows(a.rows() - offset), a.topRows(offset);
    if (a_shifted.isApprox(b)) {
      return true;
    }
    a_shifted_rev << a_shifted.colwise().reverse();
    if (a_shifted_rev.isApprox(b)) {
      return true;
    }
  }
  return false;
}

TEST_CASE("test_order_vertices_random") {
  Eigen::MatrixXd ngon1(4, 3);
  Eigen::MatrixXd ngon2(4, 3);
  Eigen::MatrixXd ngon3(5, 3);
  Eigen::MatrixXd ngon4(5, 3);
  ngon1 << cartT(30.0, 0.0, 1.0), cartT(-30.0, 0.0, 1.0),
      cartT(-30.0, 30.0, 1.0), cartT(30.0, 30.0, 1.0);
  ngon2 << cartT(30.0, 0.0, 1.0), cartT(-30.0, 0.0, 1.0),
      cartT(-30.0, 30.0, 1.0), cartT(30.0, 30.0, 1.0);
  ngon3 << cartT(30.0, 30.0, 1.0), cartT(0.0, 30.0, 1.0),
      cartT(-30.0, 30.0, 1.0), cartT(-110.0, 30.0, 1.0),
      cartT(110.0, 30.0, 1.0);
  ngon4 << cartT(30.0, 0.0, 1.0), cartT(0.0, 0.0, 1.0), cartT(-30.0, 0.0, 1.0),
      cartT(-110.0, 0.0, 1.0), cartT(110.0, 0.0, 1.0);
  std::vector<Eigen::MatrixXd> ngons = {ngon1, ngon2, ngon3, ngon4};

  std::default_random_engine generator;
  std::normal_distribution<double> distribution(0.0, 1.0);
  auto normal = [&](double) { return distribution(generator); };

  for (const auto& orderedNgon : ngons) {
    int numberOfNgons = static_cast<int>(orderedNgon.rows());
    Eigen::MatrixXd orderedNgonRandomized;
    Eigen::MatrixXd unorderedNgon;
    Eigen::MatrixXd reorderedNgon;
    Eigen::VectorXi indices;

    for (int i = 0; i < 10; ++i) {
      if (i == 0) {
        orderedNgonRandomized = orderedNgon;
      } else {
        Eigen::Matrix3d T = Eigen::Matrix3d::NullaryExpr(3, 3, normal);
        Eigen::Vector3d offset = Eigen::Vector3d::NullaryExpr(3, normal);
        orderedNgonRandomized =
            (orderedNgon * T).rowwise() + offset.transpose();
      }

      Eigen::VectorXi orderVec =
          Eigen::VectorXi::LinSpaced(numberOfNgons, 0, numberOfNgons);
      do {
        unorderedNgon = orderedNgonRandomized(orderVec, Eigen::indexing::all);
        indices = ngonVertexOrder(unorderedNgon);
        reorderedNgon = unorderedNgon(indices, Eigen::indexing::all);
        REQUIRE(inSameOrder(reorderedNgon, orderedNgonRandomized));
      } while (std::next_permutation(orderVec.begin(), orderVec.end()));
    }
  }
}

TEST_CASE("test_order_vertices_no_random") {
  Eigen::MatrixXd orderedNgon(5, 3);
  Eigen::MatrixXd unorderedNgon;
  Eigen::MatrixXd reorderedNgon;
  Eigen::VectorXi indices;
  orderedNgon << cartT(30.0, 0.0, 1.0),  //
      cartT(0.0, 0.0, 1.0),  //
      cartT(-30.0, 0.0, 1.0),  //
      cartT(-30.0, 30.0, 1.0),  //
      cartT(30.0, 30.0, 1.0);
  Eigen::VectorXi orderVec = Eigen::VectorXi::LinSpaced(
      orderedNgon.rows(), 0, static_cast<int>(orderedNgon.rows()));
  do {
    unorderedNgon = orderedNgon(orderVec, Eigen::indexing::all);
    indices = ngonVertexOrder(unorderedNgon);
    reorderedNgon = unorderedNgon(indices, Eigen::indexing::all);
    REQUIRE(inSameOrder(reorderedNgon, orderedNgon));
  } while (std::next_permutation(orderVec.begin(), orderVec.end()));
}

TEST_CASE("test_local_coordinate_system") {
  Eigen::RowVector3d x{1.0, 0.0, 0.0};
  Eigen::RowVector3d y{0.0, 1.0, 0.0};
  Eigen::RowVector3d z{0.0, 0.0, 1.0};
  Eigen::Matrix3d expected;

  REQUIRE(localCoordinateSystem(0, 0).isApprox(Eigen::Matrix3d::Identity()));
  expected << -y, x, z;
  REQUIRE(localCoordinateSystem(-90, 0).isApprox(expected));
  expected << y, -x, z;
  REQUIRE(localCoordinateSystem(90, 0).isApprox(expected));
  expected << -x, -y, z;
  REQUIRE(localCoordinateSystem(180, 0).isApprox(expected));
  expected << x, z, -y;
  REQUIRE(localCoordinateSystem(0, 90).isApprox(expected));
  expected << x, -z, y;
  REQUIRE(localCoordinateSystem(0, -90).isApprox(expected));
  expected << -y, z, -x;
  REQUIRE(localCoordinateSystem(-90, 90).isApprox(expected));
}
