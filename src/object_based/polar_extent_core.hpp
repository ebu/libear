#pragma once
#include <cstddef>
#include <memory>

namespace ear {

  /// the type used for internal extent calculations (calculating the weights
  /// and summing the gains)
  using extent_float_t = float;

  /// data shared between the core and PolarExtent; see PolarExtent
  struct PolarExtentCoreContext {
    // sizes rounded up to a multiple of the batch size
    size_t num_points;
    size_t num_speakers;

    size_t real_num_speakers;

    // position of each point
    extent_float_t *xs, *ys, *zs;
    // panning result for point and speaker is panning_results[speaker *
    // num_points + point]
    extent_float_t *panning_results;
    // total panning results in each batch is summed_panning_results[batch *
    // num_speakers + speaker]
    extent_float_t *summed_panning_results;

    // use the circular weighting function?
    bool is_circular;

    // for determining the distance from the centre
    extent_float_t flippedBasis[9];
    extent_float_t circle_test[2];
    extent_float_t right_circle_centre[2];

    // for mapping from the distance from the centre to a weight
    extent_float_t cos_start_angle, cos_end_angle;
    extent_float_t sin_start_angle, sin_end_angle;
    extent_float_t m, c;

    extent_float_t *results;
  };

  /// interface for polar extent cores; see PolarExtent
  class PolarExtentCore {
   public:
    virtual ~PolarExtentCore();

    /// the required alignment of arrays in bytes
    virtual size_t alignment() const = 0;
    /// the number of points processed at once; affects the size and shape of
    /// arrays
    virtual size_t batch_size() const = 0;
    /// calculate the weight for each point, and sum the gains into ctx.results
    virtual void run(PolarExtentCoreContext &ctx) const = 0;
  };

  /// get a PolarExtentCore for a given xsimd architecture
  template <typename Arch>
  std::unique_ptr<PolarExtentCore> get_polar_extent_core_arch();
  /// get a scalar implementation of PolarExtentCore
  std::unique_ptr<PolarExtentCore> get_polar_extent_core_scalar();

  /// get the default implementation of PolarExtentCore for this platform
  std::unique_ptr<PolarExtentCore> get_polar_extent_core();
}  // namespace ear
