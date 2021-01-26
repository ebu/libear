#pragma once
#include "extent.hpp"
#include "great_circle_arc_intersection.hpp"
#include "htm.hpp"

namespace ear {
  /// cache for associating data with an index, used to avoid
  /// recalculating whether objects are all inside or outside the extent shape
  /// or intersect with edges
  template <typename Storage>
  class Cache {
   public:
    Cache(size_t n) : known(n, false), storage(n) {}

    template <typename F>
    typename Storage::Type get(size_t i, F f) {
      if (known[i])
        return storage.get(i);
      else {
        typename Storage::Type x = f();
        storage.set(i, x);
        known[i] = true;
        return x;
      }
    }

    typename Storage::Type get(size_t i) { return storage.get(i); }

    bool has(size_t i) { return known[i]; }

    void clear() { known.assign(known.size(), false); }

   private:
    std::vector<bool> known;
    Storage storage;
  };

  class InOutStorage {
   public:
    using Type = std::tuple<bool, bool>;
    InOutStorage(size_t n) : inside(n), outside(n) {}

    std::tuple<bool, bool> get(size_t i) { return {inside[i], outside[i]}; }

    void set(size_t i, std::tuple<bool, bool> x) {
      bool this_inside, this_outside;
      std::tie(this_inside, this_outside) = x;
      inside[i] = this_inside;
      outside[i] = this_outside;
    }

   private:
    std::vector<bool> inside;
    std::vector<bool> outside;
  };
  using InOutCache = Cache<InOutStorage>;

  template <typename T>
  class VectorStorage {
   public:
    using Type = T;
    VectorStorage(size_t n) : data(n) {}

    T get(size_t i) { return data[i]; }

    void set(size_t i, T x) { data[i] = x; }

   private:
    std::vector<T> data;
  };
  using InOutCache = Cache<InOutStorage>;
  using BoolCache = Cache<VectorStorage<bool>>;

  /// functionality for testing whether objects (points and triangles are
  /// inside or outside an extent shape represented by a weighting function.
  /// This uses internals of WeightingFunction, not the public interface to
  /// avoid duplication.
  ///
  /// For points, this does the same thing as the weighting function, but skips
  /// the part that actually calculates the weight.
  ///
  /// For spherical triangles, this stores:
  /// - a representation of the inside and outside edges of the extent shape,
  ///   as a series of arcs on the sphere surface.
  /// - a point inside any continuous region inside or outside the extent shape
  ///   which could be contained by a spherical triangle
  ///
  /// a triangle is considered to be inside if:
  /// - all three points are inside
  /// - its edges do not cross the inside edges of the extent shape (since the
  ///   extent shape could extend into the triangle even if all points are
  ///   inside)
  /// - none of the outside points are inside the triangle (since the other two
  ///   conditions could be met by a triangle surrounding a small region which
  ///   is outside the extent)
  ///
  /// and similar for outside
  ///
  /// Generally functions return tuples {inside, outside}, where inside is true
  /// if the object is completely inside the extent, and outside is true if the
  /// object is completely outside.
  ///
  /// Note that this is not completely specific: it may say that a triangle is
  /// not completely inside or outside even though it is, though it should
  /// never say that a triangle is completely inside or outside if it isn't.
  /// This is intended for use in SpreadingPannerHTM, where this is acceptable.
  class WeightingFunctionEdges {
   public:
    WeightingFunctionEdges(const WeightingFunction &wf);

    /// is a point which has already been transformed by _flippedBasis inside
    /// or outside?
    std::tuple<bool, bool> point_inside_outside_t(
        const Eigen::Ref<const Eigen::Vector3d> &position) const;

    /// is an untransformed point inside or outside?
    std::tuple<bool, bool> point_inside_outside(
        const Eigen::Ref<const Eigen::Vector3d> &position) const;

    /// is a spherical triangle represented by its 3 corners inside or outside?
    std::tuple<bool, bool> spherical_tri_inside_outside(
        const Eigen::Ref<Eigen::Vector3d> &p1,
        const Eigen::Ref<Eigen::Vector3d> &p2,
        const Eigen::Ref<Eigen::Vector3d> &p3) const;

    /// is tri_id in htm inside or outside? This uses caches and the structure
    /// of the HTM to avoid some tests
    ///
    /// in_out_cache: is a point inside or outside? points are shared between
    /// triangles
    ///
    /// inside_edge_cache: does an edge cross the inside edge of the extent?
    /// edges are shared between triangles, and we know that if a parent edge
    /// didn't cross, neither will a child edge
    ///
    /// outside_edge_cache: same, but for outside edges
    std::tuple<bool, bool> spherical_tri_inside_outside(
        const HTM &htm, InOutCache &in_out_cache, BoolCache &inside_edge_cache,
        BoolCache &outside_edge_cache, size_t tri_id) const;

   private:
    const WeightingFunction &wf;
    std::vector<GreatCircleArcIntersection> inside_edges;
    std::vector<GreatCircleArcIntersection> outside_edges;

    // points inside the shape, used to check that the extent is not entirely
    // contained by a triangle
    std::vector<Eigen::Vector3d> points_inside;
    // points outside the shape, used to check that the area outside the extent
    // is not entirely contained by a triangle
    std::vector<Eigen::Vector3d> points_outside;
  };

  /// spreading panner using a HTM to improve performance
  ///
  /// This pre-computes the total gains for all points within each triangle, so
  /// it can just test if a triangle is inside or outside and add all points in
  /// one go. If a triangle is neither inside or outside, it recurses down the
  /// HTM hierarchy, splitting triangles up into sub-triangles which can be
  /// tested too. Once the maximum number of levels of splitting has been
  /// reached and a triangle isn't completely inside or outside, we fall back
  /// on the brute-force method, calculating a weight for each point within the
  /// triangle.
  ///
  /// The number of levels selected is a trade-off between the time required to
  /// test triangles, and the time required to calculate weights for points in
  /// triangles which aren't completely inside or outside. If the triangle
  /// tests are made more efficient or more specific then more levels may make
  /// sense.
  class SpreadingPannerHTM : public SpreadingPannerBase {
   public:
    SpreadingPannerHTM(std::shared_ptr<PointSourcePanner> psp, int nRows,
                       int levels = 3);
    Eigen::VectorXd panningValuesForWeight(
        const WeightingFunction &weightFunc) override;

   private:
    void assign_points_to_tris(
        const Eigen::Matrix<double, 3, Eigen::Dynamic> &input_points,
        const Eigen::MatrixXd &input_pvs, std::vector<bool> &assigned,
        size_t &next_point, size_t tri_id);

    void integrate(const WeightingFunction &wf,
                   const WeightingFunctionEdges &edges, size_t tri_id,
                   Eigen::VectorXd &pvs);

    HTM htm;

    // position of points and their associated panning values
    Eigen::Matrix<double, 3, Eigen::Dynamic> points;
    Eigen::MatrixXd point_pvs;

    // total panning values for each triangle
    Eigen::MatrixXd tri_pvs;

    // pairs of (begin, end) which point into `points` and `point_pvs`,
    // assigning points to triangles
    std::vector<std::pair<size_t, size_t>> tri_points_ptrs;

    InOutCache point_cache;
    BoolCache inside_edge_cache;
    BoolCache outside_edge_cache;
  };
}  // namespace ear
