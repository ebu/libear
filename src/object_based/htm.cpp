#include "htm.hpp"

namespace ear {
  using Point = HTM::Point;
  using ConstPointRef = const Eigen::Ref<const Eigen::Vector3d> &;
  using Tri = Eigen::Matrix3d;

  namespace {
    /// normalised mid-point between two points
    Point midpoint(ConstPointRef p1, ConstPointRef p2) {
      Point p = p1 + p2;
      return p.normalized();
    }

    /// sub-divide a triangle into 4
    /// NOTE: add_tris depends on the exact order of triangles and corners
    /// returned
    std::vector<Tri> next_tris(ConstPointRef p1, ConstPointRef p2,
                               ConstPointRef p3) {
      std::vector<Tri> tris;

      Point m12 = midpoint(p1, p2);
      Point m23 = midpoint(p2, p3);
      Point m31 = midpoint(p3, p1);

      Tri t;

      t << p1, m12, m31;
      tris.push_back(t);

      t << p2, m23, m12;
      tris.push_back(t);

      t << p3, m31, m23;
      tris.push_back(t);

      t << m12, m23, m31;
      tris.push_back(t);

      return tris;
    }

    /// get the 8 initial triangles
    std::vector<Tri> initial_tris() {
      Point x{1, 0, 0};
      Point y{0, 1, 0};
      Point z{0, 0, 1};

      std::vector<Tri> tris;

      Tri t;

      t << x, z, y;
      tris.push_back(t);
      t << y, z, -x;
      tris.push_back(t);
      t << -x, z, -y;
      tris.push_back(t);
      t << -y, z, x;
      tris.push_back(t);

      t << y, -z, x;
      tris.push_back(t);
      t << -x, -z, y;
      tris.push_back(t);
      t << -y, -z, -x;
      tris.push_back(t);
      t << x, -z, -y;
      tris.push_back(t);

      return tris;
    }
  }  // namespace

  HTM::HTM(int num_levels) {
    for (auto &tri : initial_tris())
      initial_tri_ids.push_back(add_tris(tri, num_levels));
  }

  /// given a position, find its index in points, adding it if there's no
  /// matches
  size_t HTM::point_id(ConstPointRef p) {
    for (size_t i = 0; i < points.size(); i++)
      if ((points[i] - p).squaredNorm() < 1e-6) return i;
    points.emplace_back(p);
    return points.size() - 1;
  }

  /// given two point indexes, find or add an edge between them, returning the
  /// index into edges
  size_t HTM::edge_id(size_t a, size_t b) {
    if (b < a) std::swap(a, b);
    for (size_t i = 0; i < edges.size(); i++)
      if ((edges[i].points[0] == a && edges[i].points[1] == b)) return i;
    edges.emplace_back(a, b);
    return edges.size() - 1;
  }

  /// add tri, and levels subdivisions
  size_t HTM::add_tris(Tri tri, int levels) {
    // add the triangle, points and edges
    tris.push_back({});
    size_t id = tris.size() - 1;

    for (size_t i = 0; i < 3; i++) tris[id].points[i] = point_id(tri.col(i));

    for (size_t i = 0; i < 3; i++)
      tris[id].edges[i] =
          edge_id(tris[id].points[i], tris[id].points[(i + 1) % 3]);

    if (levels) {
      // subdivide into 4 sub-triangles
      auto next =
          next_tris(points[tris[id].points[0]], points[tris[id].points[1]],
                    points[tris[id].points[2]]);

      // add the subdivisions
      // NOTE: this depends on the order from add_tris; the first three are the
      // corner subdivisions (which share edges with tri), the fourth is the
      // centre (which doesn't)
      for (size_t i = 0; i < 4; i++) {
        size_t next_tri_id = add_tris(next[i], levels - 1);
        tris[id].next_tris[i] = next_tri_id;

        if (i < 3) {
          edges[tris[next_tri_id].edges[0]].has_parent = true;
          edges[tris[next_tri_id].edges[0]].parent = tris[id].edges[i];

          edges[tris[next_tri_id].edges[2]].has_parent = true;
          edges[tris[next_tri_id].edges[2]].parent =
              tris[id].edges[(i + 2) % 3];
        }
      }

      tris[id].has_next = true;
    } else
      tris[id].has_next = false;

    return id;
  }

}  // namespace ear
