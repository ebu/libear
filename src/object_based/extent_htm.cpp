#include "extent_htm.hpp"
#include <Eigen/Core>
#include <memory>

#include "../common/geom.hpp"
#include "../common/helpers/eigen_helpers.hpp"

const double PI = boost::math::constants::pi<double>();

namespace {
  bool pointInsideSphericalTriangle(
      const Eigen::Ref<const Eigen::Vector3d> &p1,
      const Eigen::Ref<const Eigen::Vector3d> &p2,
      const Eigen::Ref<const Eigen::Vector3d> &p3,
      const Eigen::Ref<const Eigen::Vector3d> &q) {
    return p2.cross(p1).dot(q) >= -1e-6 && p3.cross(p2).dot(q) >= -1e-6 &&
           p1.cross(p3).dot(q) >= -1e-6;
  }
}  // namespace

namespace ear {
  WeightingFunctionEdges::WeightingFunctionEdges(const WeightingFunction &wf)
      : wf(wf) {
    if (wf.is_circular) {
      inside_edges.emplace_back(
          Eigen::Vector3d{sin(wf.circlePos() + PI / 2),
                          cos(wf.circlePos() + PI / 2), 0},
          Eigen::Vector3d{0, 0, 1}, wf.circleRadius(), PI);
      outside_edges.emplace_back(
          Eigen::Vector3d{sin(wf.circlePos() + PI / 2),
                          cos(wf.circlePos() + PI / 2), 0},
          Eigen::Vector3d{0, 0, 1}, wf.circleRadius() + radians(wf._fadeWidth),
          PI);
    } else {
      // top and bottom edges
      inside_edges.emplace_back(Eigen::Vector3d{0, 1, 0},
                                Eigen::Vector3d{1, 0, 0},
                                PI / 2 - wf.circleRadius(), wf.circlePos());
      inside_edges.emplace_back(Eigen::Vector3d{0, 1, 0},
                                Eigen::Vector3d{-1, 0, 0},
                                PI / 2 - wf.circleRadius(), wf.circlePos());
      outside_edges.emplace_back(
          Eigen::Vector3d{0, 1, 0}, Eigen::Vector3d{1, 0, 0},
          PI / 2 - (wf.circleRadius() + radians(wf._fadeWidth)),
          wf.circlePos());
      outside_edges.emplace_back(
          Eigen::Vector3d{0, 1, 0}, Eigen::Vector3d{-1, 0, 0},
          PI / 2 - (wf.circleRadius() + radians(wf._fadeWidth)),
          wf.circlePos());

      // right and left circles
      inside_edges.emplace_back(
          Eigen::Vector3d{sin(wf.circlePos() + PI / 2),
                          cos(wf.circlePos() + PI / 2), 0},
          Eigen::Vector3d{0, 0, 1}, wf.circleRadius(), PI / 2);
      inside_edges.emplace_back(
          Eigen::Vector3d{-sin(wf.circlePos() + PI / 2),
                          cos(wf.circlePos() + PI / 2), 0},
          Eigen::Vector3d{0, 0, -1}, wf.circleRadius(), PI / 2);
      outside_edges.emplace_back(
          Eigen::Vector3d{sin(wf.circlePos() + PI / 2),
                          cos(wf.circlePos() + PI / 2), 0},
          Eigen::Vector3d{0, 0, 1}, wf.circleRadius() + radians(wf._fadeWidth),
          PI / 2);
      outside_edges.emplace_back(
          Eigen::Vector3d{-sin(wf.circlePos() + PI / 2),
                          cos(wf.circlePos() + PI / 2), 0},
          Eigen::Vector3d{0, 0, -1}, wf.circleRadius() + radians(wf._fadeWidth),
          PI / 2);
    }

    if (wf._width <= PI / 2) points_inside.emplace_back(0, 1, 0);

    if (wf._width > PI) {
      // touching at back, so there's possibly two regions at the top and bottom
      // outside
      if (wf._height <= PI / 2) {
        points_outside.emplace_back(0, 0, 1);
        points_outside.emplace_back(0, 0, -1);
      }
    } else {
      // not touching at back so there's only one region outside, and we only
      // need to check this if the maximum angle within it is less than 90
      if (wf._height > PI * (3.0 / 4.0)) points_outside.emplace_back(0, -1, 0);
    }
  }

  std::tuple<bool, bool> WeightingFunctionEdges::point_inside_outside_t(
      const Eigen::Ref<const Eigen::Vector3d> &position_t) const {
    if (wf.is_circular) {
      // simplified dot and norm of cross product assuming that circle_centre
      // is {0, 1, 0}
      double dot = position_t.y();

      return {wf.angle_to_weight.inside_from_cos(dot),
              wf.angle_to_weight.outside_from_cos(dot)};
    }

    Eigen::RowVector3d position_t_right = position_t;
    position_t_right(0) = std::abs(position_t(0));

    // for the straight lines
    if (position_t_right.head<2>() * wf.circle_test >= 0) {
      double sin_abs_elevation = std::abs(position_t(2));
      return {wf.angle_to_weight.inside_from_sin(sin_abs_elevation),
              wf.angle_to_weight.outside_from_sin(sin_abs_elevation)};
    } else {
      double dot = wf.right_circle_centre.dot(position_t_right.head<2>());

      return {wf.angle_to_weight.inside_from_cos(dot),
              wf.angle_to_weight.outside_from_cos(dot)};
    }
  }

  std::tuple<bool, bool> WeightingFunctionEdges::point_inside_outside(
      const Eigen::Ref<const Eigen::Vector3d> &position) const {
    return point_inside_outside_t(wf._flippedBasis * position);
  }

  std::tuple<bool, bool> WeightingFunctionEdges::spherical_tri_inside_outside(
      const Eigen::Ref<Eigen::Vector3d> &p1,
      const Eigen::Ref<Eigen::Vector3d> &p2,
      const Eigen::Ref<Eigen::Vector3d> &p3) const {
    Eigen::Vector3d p1_t = wf._flippedBasis * p1;
    Eigen::Vector3d p2_t = wf._flippedBasis * p2;
    Eigen::Vector3d p3_t = wf._flippedBasis * p3;

    bool p1_in, p1_out, p2_in, p2_out, p3_in, p3_out;

    // find if the three corners are inside or outside; bail out early if they
    // are inconsistent
    std::tie(p1_in, p1_out) = point_inside_outside_t(p1_t);
    std::tie(p2_in, p2_out) = point_inside_outside_t(p2_t);
    if (p1_in != p2_in || p1_out != p2_out) return {false, false};
    std::tie(p3_in, p3_out) = point_inside_outside_t(p3_t);
    if (p2_in != p3_in || p2_out != p3_out) return {false, false};

    if (p1_in && p2_in && p3_in) {
      for (auto &p : points_outside)
        if (pointInsideSphericalTriangle(p1_t, p2_t, p3_t, p))
          return {false, false};

      for (auto &edge : inside_edges) {
        if (edge.intersects(p1_t, p2_t) || edge.intersects(p2_t, p3_t) ||
            edge.intersects(p3_t, p1_t))
          return {false, false};
      }

      return {true, false};
    } else if (p1_out && p2_out && p3_out) {
      for (auto &p : points_inside)
        if (pointInsideSphericalTriangle(p1_t, p2_t, p3_t, p))
          return {false, false};

      for (auto &edge : outside_edges) {
        if (edge.intersects(p1_t, p2_t) || edge.intersects(p2_t, p3_t) ||
            edge.intersects(p3_t, p1_t))
          return {false, false};
      }
      return {false, true};
    } else
      return {false, false};
  }

  std::tuple<bool, bool> WeightingFunctionEdges::spherical_tri_inside_outside(
      const HTM &htm, InOutCache &in_out_cache, BoolCache &inside_edge_cache,
      BoolCache &outside_edge_cache, size_t tri_id) const {
    std::array<size_t, 3> point_ids = htm.tris[tri_id].points;

    Eigen::Vector3d p1_t = wf._flippedBasis * htm.points[point_ids[0]];
    Eigen::Vector3d p2_t = wf._flippedBasis * htm.points[point_ids[1]];
    Eigen::Vector3d p3_t = wf._flippedBasis * htm.points[point_ids[2]];
    std::array<Eigen::Vector3d *, 3> points_t = {&p1_t, &p2_t, &p3_t};

    bool p1_in, p1_out, p2_in, p2_out, p3_in, p3_out;

    std::tie(p1_in, p1_out) = in_out_cache.get(
        point_ids[0], [&]() { return point_inside_outside_t(p1_t); });
    std::tie(p2_in, p2_out) = in_out_cache.get(
        point_ids[1], [&]() { return point_inside_outside_t(p2_t); });
    if (p1_in != p2_in || p1_out != p2_out) return {false, false};
    std::tie(p3_in, p3_out) = in_out_cache.get(
        point_ids[2], [&]() { return point_inside_outside_t(p3_t); });
    if (p2_in != p3_in || p2_out != p3_out) return {false, false};

    if (p1_in && p2_in && p3_in) {
      for (auto &p : points_outside)
        if (pointInsideSphericalTriangle(p1_t, p2_t, p3_t, p))
          return {false, false};

      // check if any tri edge intersects the inside edge of the shape. This is
      // a bit tortured (getting from the cache by the edge id, but referring
      // to our own points means that the order of edges here has to match the
      // HTM), but this is to avoid transforming the points again. Ideally
      // these should be cached too...
      for (size_t i = 0; i < 3; i++) {
        size_t edge_i = htm.tris[tri_id].edges[i];
        bool intersects_inside_edge = inside_edge_cache.get(edge_i, [&]() {
          // if the inside edges don't cross the parent of this tri edge
          // (which this was split in half from), we know it doesn't cross
          // this edge either.
          if (htm.edges[edge_i].has_parent &&
              inside_edge_cache.has(htm.edges[edge_i].parent) &&
              !inside_edge_cache.get(htm.edges[edge_i].parent))
            return false;
          for (auto &shape_edge : inside_edges)
            if (shape_edge.intersects(*points_t[i], *points_t[(i + 1) % 3]))
              return true;
          return false;
        });
        if (intersects_inside_edge) return {false, false};
      }

      return {true, false};
    } else if (p1_out && p2_out && p3_out) {
      for (auto &p : points_inside)
        if (pointInsideSphericalTriangle(p1_t, p2_t, p3_t, p))
          return {false, false};

      // same as above, but for outside edges
      for (size_t i = 0; i < 3; i++) {
        size_t edge_i = htm.tris[tri_id].edges[i];
        bool intersects_outside_edge = outside_edge_cache.get(edge_i, [&]() {
          if (htm.edges[edge_i].has_parent &&
              outside_edge_cache.has(htm.edges[edge_i].parent) &&
              !outside_edge_cache.get(htm.edges[edge_i].parent))
            return false;
          for (auto &shape_edge : outside_edges)
            if (shape_edge.intersects(*points_t[i], *points_t[(i + 1) % 3]))
              return true;
          return false;
        });
        if (intersects_outside_edge) return {false, false};
      }

      return {false, true};
    } else
      return {false, false};
  }

  SpreadingPannerHTM::SpreadingPannerHTM(std::shared_ptr<PointSourcePanner> psp,
                                         int nRows, int levels)
      : htm(levels),
        tri_points_ptrs(htm.tris.size()),
        point_cache(htm.points.size()),
        inside_edge_cache(htm.edges.size()),
        outside_edge_cache(htm.edges.size()) {
    Eigen::MatrixXd input_points =
        _generatePanningPositionsEven(nRows).transpose();

    Eigen::MatrixXd input_pvs =
        _generatePanningPositionsResults(psp, input_points.transpose());

    points.resize(3, input_points.cols());
    point_pvs.resize(input_pvs.rows(), input_points.cols());
    tri_pvs = Eigen::MatrixXd::Zero(input_pvs.rows(), htm.tris.size());

    // assign points to triangles, summing all the PVs into tri_pvs
    //
    // assigned is used to ensure that each point is assigned to only one leaf
    // triangle (even though it's possible for a point to be in multiple
    // triangles because of tolerance)
    //
    // assign_points_to_tris works recursively writing into points and
    // point_pvs at next_point, in depth-first order
    std::vector<bool> assigned(input_points.cols(), false);
    size_t next_point = 0;
    for (size_t base_tri : htm.initial_tri_ids)
      assign_points_to_tris(input_points, input_pvs, assigned, next_point,
                            base_tri);

    ear_assert(next_point == (size_t)points.cols(), "missed a point");
    for (bool tri_assigned : assigned)
      ear_assert(tri_assigned, "point not assigned to any tri");
  }

  void SpreadingPannerHTM::assign_points_to_tris(
      const Eigen::Matrix<double, 3, Eigen::Dynamic> &input_points,
      const Eigen::MatrixXd &input_pvs, std::vector<bool> &assigned,
      size_t &next_point, size_t tri_id) {
    size_t start = next_point;
    if (!htm.tris[tri_id].has_next) {
      // if this is a leaf triangle, assign any un-assigned points to it
      HTM::Point &p1 = htm.points[htm.tris[tri_id].points[0]];
      HTM::Point &p2 = htm.points[htm.tris[tri_id].points[1]];
      HTM::Point &p3 = htm.points[htm.tris[tri_id].points[2]];

      for (size_t i = 0; i < (size_t)input_points.cols(); i++) {
        if (!assigned[i] &&
            pointInsideSphericalTriangle(p1, p2, p3, input_points.col(i))) {
          assigned[i] = true;
          points.col(next_point) = input_points.col(i);
          point_pvs.col(next_point) = input_pvs.col(i);
          next_point++;
          tri_pvs.col(tri_id) += input_pvs.col(i);
        }
      }
    } else {
      // otherwise, assign points to the subdivisions of this triangle
      for (size_t next_tri : htm.tris[tri_id].next_tris) {
        assign_points_to_tris(input_points, input_pvs, assigned, next_point,
                              next_tri);
        tri_pvs.col(tri_id) += tri_pvs.col(next_tri);
      }
    }
    size_t end = next_point;

    tri_points_ptrs[tri_id] = {start, end};
  }

  Eigen::VectorXd SpreadingPannerHTM::panningValuesForWeight(
      const WeightingFunction &weightFunc) {
    point_cache.clear();
    inside_edge_cache.clear();
    outside_edge_cache.clear();

    WeightingFunctionEdges edges(weightFunc);

    Eigen::VectorXd totalPv = Eigen::VectorXd::Zero(point_pvs.rows());
    for (size_t base_tri : htm.initial_tri_ids)
      integrate(weightFunc, edges, base_tri, totalPv);

    totalPv /= totalPv.norm();
    return totalPv;
  }

  void SpreadingPannerHTM::integrate(const WeightingFunction &wf,
                                     const WeightingFunctionEdges &edges,
                                     size_t tri_id, Eigen::VectorXd &pvs) {
    HTM::CTri &tri = htm.tris[tri_id];
    bool inside, outside;
    std::tie(inside, outside) = edges.spherical_tri_inside_outside(
        htm, point_cache, inside_edge_cache, outside_edge_cache, tri_id);
    if (inside)
      pvs += tri_pvs.col(tri_id);
    else if (outside)
      return;
    else {
      if (tri.has_next) {
        for (size_t next_tri_id : tri.next_tris)
          integrate(wf, edges, next_tri_id, pvs);
      } else {
        size_t start, end;
        std::tie(start, end) = tri_points_ptrs[tri_id];
        for (size_t i = start; i < end; i++) {
          double weight = wf(points.col(i).transpose());
          if (weight != 0.0) pvs.noalias() += weight * point_pvs.col(i);
        }
      }
    }
  }
}  // namespace ear
