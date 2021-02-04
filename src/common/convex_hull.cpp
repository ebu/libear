#include "convex_hull.hpp"
#include <Eigen/Geometry>
#include "ear/helpers/assert.hpp"

namespace ear {

  std::vector<Facet> convex_hull(const std::vector<Eigen::Vector3d> &positions,
                                 double tolerance) {
    using Vec = Eigen::Vector3d;
    using Tri = std::array<size_t, 3>;

    // the mean point, guaranteed to be inside the convex hull
    Vec inside_point = Vec::Zero();
    for (auto &pos : positions) inside_point += pos;
    inside_point /= positions.size();

    // find all triangles on the convex hull:
    // - iterate through all possible triangles
    // - find the triangle normal, checking for collinearity
    // - it's on the convex hull if:
    //    - the inside point is not on the plane of the triangle
    //    - all points are on the same side of the plane of the triangle as the
    //    inside point
    std::vector<Tri> hull_tris;
    std::vector<Vec> hull_tri_normals;

    for (size_t i = 0; i < positions.size(); i++)
      for (size_t j = i + 1; j < positions.size(); j++)
        for (size_t k = j + 1; k < positions.size(); k++) {
          Vec normal =
              (positions[j] - positions[i]).cross(positions[k] - positions[i]);
          ear_assert(normal.squaredNorm() > tolerance,
                     "collinear points in convex hull");
          normal.normalize();  // XXX: not really required, but helps make
                               // tolerances consistent

          double dot_inside = normal.dot(inside_point - positions[i]);

          if (std::abs(dot_inside) < tolerance)
            continue;  // tri coplanar with inside point

          bool points_on_same_side_as_inside = true;
          for (auto &pos : positions) {
            double dot_point = normal.dot(pos - positions[i]);

            // check if signs are equal, with tolerance
            if (!(dot_inside > 0 ? dot_point > -tolerance
                                 : dot_point < tolerance)) {
              points_on_same_side_as_inside = false;
              break;
            }
          }

          if (points_on_same_side_as_inside) {
            hull_tris.push_back({i, j, k});
            hull_tri_normals.push_back(normal);
          }
        }

    // merge coplanar triangles into facets
    std::vector<Facet> hull_facets;
    std::vector<Vec> hull_facet_normals;

    for (size_t tri_i = 0; tri_i < hull_tris.size(); tri_i++) {
      const Vec &tri_point = positions[hull_tris[tri_i][0]];
      const Vec &tri_norm = hull_tri_normals[tri_i];

      // check each facet to see if this triangle is on the same plane.
      // if it is, add the points to the facet. if no facet is found, make a
      // new facet.
      bool found_facet = false;
      for (size_t facet_i = 0; facet_i < hull_facets.size(); facet_i++) {
        const Vec &facet_norm = hull_facet_normals[facet_i];
        const Vec &facet_point = positions[*hull_facets[facet_i].begin()];

        // they are on the same plane if the line between the test points is
        // not along the normal, and if the normals point in the same or
        // opposite direction. alternatively, we could just test all tri
        // vertices.
        if (std::abs((tri_point - facet_point).dot(facet_norm)) < tolerance &&
            facet_norm.cross(tri_norm).squaredNorm() < tolerance) {
          for (size_t corner_idx : hull_tris[tri_i]) {
            hull_facets[facet_i].insert((Facet::value_type)corner_idx);
          }

          found_facet = true;
          break;
        }
      }

      if (!found_facet) {
        Facet f;
        for (size_t corner_idx : hull_tris[tri_i])
          f.insert((Facet::value_type)corner_idx);
        hull_facets.push_back(std::move(f));
        hull_facet_normals.push_back(tri_norm);
      }
    }

    return hull_facets;
  }
}  // namespace ear
