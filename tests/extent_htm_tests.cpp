#include <Eigen/Core>
#include <catch2/catch.hpp>

#include "ear/bs2051.hpp"
#include "eigen_utils.hpp"
#include "extent_htm_util.hpp"
#include "object_based/extent_htm.hpp"
#include "reference/extent.hpp"

using namespace ear;

/// generate a random spherical triangle with less than 90 degree angle between
/// corners
std::array<Eigen::Vector3d, 3> gen_spherical_tri() {
  Eigen::Vector3d p1 = Eigen::Vector3d::Random();
  p1.normalize();

  Eigen::Vector3d p2;
  while (true) {
    p2 = Eigen::Vector3d::Random();
    p2.normalize();
    if (p1.dot(p2) > 0) break;
  }

  Eigen::Vector3d p3;
  while (true) {
    p3 = Eigen::Vector3d::Random();
    p3.normalize();
    // wider tolerance here as we will not generally have to deal with narrow
    // triangles
    if (p1.dot(p3) > 0 && p2.dot(p3) > 0 && p2.cross(p1).dot(p3) >= 1e-5) break;
  }

  return {p1, p2, p3};
}

bool pointInsideSphericalTriangle(const Eigen::Ref<const Eigen::Vector3d> &p1,
                                  const Eigen::Ref<const Eigen::Vector3d> &p2,
                                  const Eigen::Ref<const Eigen::Vector3d> &p3,
                                  const Eigen::Ref<const Eigen::Vector3d> &q) {
  return p2.cross(p1).dot(q) >= -1e-6 && p3.cross(p2).dot(q) >= -1e-6 &&
         p1.cross(p3).dot(q) >= -1e-6;
}

TEST_CASE("point_in_triangle_basic") {
  REQUIRE(pointInsideSphericalTriangle(
      Eigen::Vector3d{1.0, 0.0, 0.0}, Eigen::Vector3d{0.0, 0.0, 1.0},
      Eigen::Vector3d{0.0, 1.0, 0.0}, Eigen::Vector3d{1.0, 1.0, 1.0}));
  REQUIRE(!pointInsideSphericalTriangle(
      Eigen::Vector3d{1.0, 0.0, 0.0}, Eigen::Vector3d{0.0, 0.0, 1.0},
      Eigen::Vector3d{0.0, 1.0, 0.0}, Eigen::Vector3d{-1.0, -1.0, -1.0}));
}

TEST_CASE("point_in_triangle_random") {
  for (std::size_t j = 0; j < 1000; j++) {
    std::array<Eigen::Vector3d, 3> tri = gen_spherical_tri();
    REQUIRE(pointInsideSphericalTriangle(
        tri[0], tri[1], tri[2], (tri[0] + tri[1] + tri[2]).normalized()));
    REQUIRE(!pointInsideSphericalTriangle(
        tri[0], tri[1], tri[2], -(tri[0] + tri[1] + tri[2]).normalized()));
    REQUIRE(pointInsideSphericalTriangle(tri[0], tri[1], tri[2], tri[0]));
    REQUIRE(pointInsideSphericalTriangle(tri[0], tri[1], tri[2], tri[1]));
    REQUIRE(pointInsideSphericalTriangle(tri[0], tri[1], tri[2], tri[2]));
  }
}

/// generate vectors which when multiplied with the corner vectors of a
/// spherical triangle, produces a spread of points inside, including the
/// corners and points on the edge
std::vector<Eigen::Vector3d> gen_tri_points() {
  std::vector<Eigen::Vector3d> points;
  const size_t n_side = 20;
  for (size_t i = 0; i < n_side; i++) {
    // 0 to 1
    double x = (double)i / (n_side - 1);

    // x=0 -> n_side; x=1 -> 10
    int n_y = round((n_side - 1) * (1 - x)) + 1;

    for (int j = 0; j < n_y; j++) {
      double y = j == 0 ? 0.0 : (double)j * (1.0 - x) / (n_y - 1);
      double z = 1.0 - (x + y);

      points.emplace_back(x, y, z);
    }
  }

  return points;
}

/// get some spherical triangles for testing; these are a combination of HTM
/// triangles, and random triangles from gen_spherical_tri.
std::vector<std::array<Eigen::Vector3d, 3>> gen_spherical_tris() {
  std::vector<std::array<Eigen::Vector3d, 3>> result;

  HTM htm(3);
  for (auto &tri : htm.tris)
    result.push_back({htm.points[tri.points[0]], htm.points[tri.points[1]],
                      htm.points[tri.points[2]]});

  for (std::size_t j = 0; j < 500; j++) result.push_back(gen_spherical_tri());
  return result;
}

/// for a given extent shape:
/// - make a weighting function with a random position
/// - generate many spherical triangles; for each:
///     - use WeightingFunctionEdges to test if the tri is inside or outside
///       the extent shape
///     - sample points in the triangle, and test if they are inside or outside
///       the extent shape.
///         - if the triangle is inside, all points must be outside
///         - if the triangle is outside, all points must be outside
///     - check if all points are inside or outside. This should match the
///       result for the whole triangle, but a mismatch is not an error, as
///       WeightingFunctionEdges may safely be conservative. This is useful for
///       development, as making these match more closely can improve
///       performance
void check_tri_inside_outside(double width, double height) {
  Eigen::Vector3d pos = Eigen::Vector3d::Random();
  pos.normalize();
  INFO("testing width=\n"
       << width << " height=" << height << " pos=" << pos.transpose());
  WeightingFunction weightingFunc(pos, width, height);
  WeightingFunctionEdges edges(weightingFunc);

  size_t total_inside = 0;
  size_t total_outside = 0;
  size_t bf_total_inside = 0;
  size_t bf_total_outside = 0;

  auto tri_points = gen_tri_points();

  for (auto &tri : gen_spherical_tris()) {
    INFO("testing tri\n"
         << tri[0].transpose() << "\n"
         << tri[1].transpose() << "\n"
         << tri[2].transpose());
    bool inside, outside;
    std::tie(inside, outside) =
        edges.spherical_tri_inside_outside(tri[0], tri[1], tri[2]);

    bool all_inside = true;
    bool all_outside = true;
    for (auto &tri_point : tri_points) {
      Eigen::Vector3d p =
          tri[0] * tri_point(0) + tri[1] * tri_point(1) + tri[2] * tri_point(2);
      p.normalize();

      bool p_inside, p_outside;
      std::tie(p_inside, p_outside) = edges.point_inside_outside(p);

      // if we know it's inside or outside, then all the points in the triangle
      // must agree
      if (inside) REQUIRE(p_inside);
      if (outside) REQUIRE(p_outside);

      if (!p_inside) all_inside = false;
      if (!p_outside) all_outside = false;

      if (!inside && !outside && !all_inside && !all_outside) break;
    }

    // weighting function says it's not outside, but all points were outside
    if (all_outside != outside) {
      INFO("all_outside != outside");
      INFO(tri[0].transpose());
      INFO(tri[1].transpose());
      INFO(tri[2].transpose() << "\n");
    }

    // weighting function says it's not inside, but all points were inside
    if (all_inside != inside) {
      INFO("all_inside != inside\n");
      INFO(tri[0].transpose());
      INFO(tri[1].transpose());
      INFO(tri[2].transpose() << "\n");
    }

    if (inside) total_inside++;
    if (outside) total_outside++;
    if (all_inside) bf_total_inside++;
    if (all_outside) bf_total_outside++;
  }
  INFO("exact: " << total_inside << " inside " << total_outside << " outside");
  INFO("brute force: " << bf_total_inside << " inside " << bf_total_outside
                       << " outside\n");
}

TEST_CASE("spherical_tri") {
  // a range of sizes, and some specific sizes that caused issues during
  // development
  check_tri_inside_outside(0, 0);
  check_tri_inside_outside(40, 40);
  check_tri_inside_outside(170, 30);
  check_tri_inside_outside(180, 0);
  check_tri_inside_outside(180, 90);
  check_tri_inside_outside(280, 90);
  check_tri_inside_outside(287.438, 164.097);
  check_tri_inside_outside(287.438, 328.193);
  check_tri_inside_outside(338.701, 148.661);
  check_tri_inside_outside(350, 350);
  check_tri_inside_outside(360, 0);
  check_tri_inside_outside(360, 145);
  check_tri_inside_outside(360, 360);
}

TEST_CASE("htm") {
  HTM htm(3);
  for (auto &tri : htm.tris) {
    Eigen::Vector3d p1 = htm.points[tri.points[0]];
    Eigen::Vector3d p2 = htm.points[tri.points[1]];
    Eigen::Vector3d p3 = htm.points[tri.points[2]];

    // check the winding
    REQUIRE(p2.cross(p1).dot(p3) >= 1e-4);

    // check the opening angles
    REQUIRE(p1.dot(p2) >= 0.0);
    REQUIRE(p2.dot(p3) >= 0.0);
    REQUIRE(p3.dot(p1) >= 0.0);

    for (size_t edge_i = 0; edge_i < 3; edge_i++) {
      auto &edge = htm.edges[tri.edges[edge_i]];
      if (edge.has_parent) {
        auto &parent = htm.edges[edge.parent];
        // find the matching end of parent and child, and check that the parent
        // and child go in the same direction
        bool found = false;
        for (size_t i = 0; i < 2; i++)
          for (size_t j = 0; j < 2; j++)
            if (parent.points[i] == edge.points[j]) {
              REQUIRE(!found);
              found = true;

              Eigen::Vector3d cross_parent = htm.points[parent.points[i]].cross(
                  htm.points[parent.points[(i + 1) % 2]]);
              Eigen::Vector3d cross_child = htm.points[edge.points[j]].cross(
                  htm.points[edge.points[(j + 1) % 2]]);
              REQUIRE(std::asin(cross_parent.norm()) /
                          std::asin(cross_child.norm()) ==
                      Approx(2));

              cross_parent.normalize();
              cross_child.normalize();
              REQUIRE_THAT(cross_parent, IsApprox(cross_child));
            }
        REQUIRE(found);
      }
    }
  }
}

/// check that PVs from HTM and plain panners match
TEST_CASE("htm_panner") {
  PolarExtentPanner extentPanner(makeExtentPanner("9+10+3"));
  PolarExtentPanner extentPannerHTM(makeExtentPanner("9+10+3", true, 2));

  for (size_t i = 0; i < 1000; i++) {
    Eigen::Vector3d pos = Eigen::Vector3d::Random();
    pos.normalize();
    Eigen::Vector2d size = (Eigen::Vector2d::Random().array() + 1) * 180;
    double width = size(0);
    double height = size(1);
    Eigen::VectorXd spread_pv = extentPanner.calcPvSpread(pos, width, height);
    Eigen::VectorXd spread_pv_htm =
        extentPannerHTM.calcPvSpread(pos, width, height);

    INFO("size " << width << " " << height);
    REQUIRE_THAT(spread_pv_htm, IsApprox(spread_pv));
  }
}

#ifdef CATCH_CONFIG_ENABLE_BENCHMARKING
TEST_CASE("bench_panner", "[.benchmark]") {
  Layout layout = getLayout("9+10+3").withoutLfe();
  std::shared_ptr<PointSourcePanner> psp = configurePolarPanner(layout);
  reference::PolarExtentPanner extentPannerRef(psp);

  PolarExtentPanner extentPanner(makeExtentPanner("9+10+3"));
  PolarExtentPanner extentPannerHTM0(makeExtentPanner("9+10+3", true, 0));
  PolarExtentPanner extentPannerHTM1(makeExtentPanner("9+10+3", true, 1));
  PolarExtentPanner extentPannerHTM2(makeExtentPanner("9+10+3", true, 2));

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

    BENCHMARK("non-htm ref " + size_name) {
      return extentPannerRef.calcPvSpread(pos, width, height);
    };

    BENCHMARK("non-htm " + size_name) {
      return extentPanner.calcPvSpread(pos, width, height);
    };

    BENCHMARK("htm0 " + size_name) {
      return extentPannerHTM0.calcPvSpread(pos, width, height);
    };

    BENCHMARK("htm1 " + size_name) {
      return extentPannerHTM1.calcPvSpread(pos, width, height);
    };

    BENCHMARK("htm2 " + size_name) {
      return extentPannerHTM2.calcPvSpread(pos, width, height);
    };
  }
}
#endif
