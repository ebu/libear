#pragma once
#include <Eigen/Core>
#include <array>
#include <vector>

namespace ear {
  /// Represents the subdivision of a sphere into a Hierarchical Mesh of
  /// Triangles; see http://www.skyserver.org/HTM/
  ///
  /// This doesn't really do anything except hold the structure, as we don't
  /// need the standard querying operations for our application (the polar
  /// extent panner).
  ///
  /// Each edge and point has an index, which is shared between triangles.
  class HTM {
   public:
    using Point = Eigen::Vector3d;

    /// num_levels: the number of subdivisions below the 8 basis triangles
    HTM(int num_levels);

    /// Compact Triangle representation
    struct CTri {
      /// indexes into points of the corners
      std::array<size_t, 3> points;
      /// indexes into tris of the sub-triangles, valid if has_next
      std::array<size_t, 4> next_tris;
      /// indexes into edges of the edges of this triangle
      std::array<size_t, 3> edges;
      /// is this triangle sub-divided?
      bool has_next;
    };

    /// Compact Edge representation
    struct CEdge {
      CEdge(size_t a, size_t b) : points{a, b} {}
      /// the two ends of this edge; indexes into points
      std::array<size_t, 2> points;
      /// the edge which this edge was split in half from, if has_parent
      size_t parent = 0;
      /// was this edge split from another?
      bool has_parent = false;
    };

    /// point storage, referenced from tris and edges
    std::vector<Point> points;
    /// triangle storage, referenced from initial_tri_ids and triangles
    std::vector<CTri> tris;
    /// edge storage, referenced from triangles and edges
    std::vector<CEdge> edges;
    /// indexes into tris, the 8 basis triangles
    std::vector<size_t> initial_tri_ids;

   private:
    using ConstPointRef = const Eigen::Ref<const Eigen::Vector3d> &;
    using Tri = Eigen::Matrix3d;
    // initialisation functions, see implementation
    size_t point_id(ConstPointRef p);
    size_t edge_id(size_t a, size_t b);
    size_t add_tris(Tri tri, int levels);
  };
}  // namespace ear
