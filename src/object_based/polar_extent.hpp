#pragma once
#include "../common/point_source_panner.hpp"
#include "polar_extent_core.hpp"

namespace ear {

  Eigen::Matrix3d calcBasis(Eigen::Vector3d position);

  /// polar extent implementation
  ///
  /// this has a different structure from the reference implementation for
  /// efficiency: rather than the SpreadingPanner, which sums pre-calculated
  /// gains corresponding to points on the sphere, with weights calculated by a
  /// WeightingFunction instance, we have instead:
  ///
  /// - a core, which calculates the weight of each point according to some
  ///   some pre-processed parameters, and sums the weighted gains for each
  ///   point (without normalisation)
  ///
  /// - the PolarExtent class, which sets up the conditions for the core
  ///   (generating positions and corresponding gains, and pre-processing the
  ///   parameters), runs the core, and post-processes the results
  ///
  /// this makes it possible to have core implementations that process multiple
  /// points at once
  ///
  /// the various moving parts of this are as follows:
  ///
  /// - PolarExtentCore in polar_extent_core.hpp is the interface that core
  ///   implementations implement
  ///
  /// - PolarExtentCoreContext in polar_extent_core.hpp is the structure passed
  ///   between PolarExtent and the core implementations
  ///
  /// - get_polar_extent_core in polar_extent_core.hpp constructs the best core
  ///   implementation for this platform
  ///
  /// - PolarExtentCoreScalar in polar_extent_scalar.cpp is a scalar core
  ///   implementation to fall back on
  ///
  /// - PolarExtentCoreSimd in polar_extent_simd.hpp is a SIMD core
  ///   implementation using xsimd, parametrised by the instruction set to use
  ///
  /// - polar_extent_simd_instance.cpp is compiled once for each supported
  ///   architecture (see src/CMakeLists.txt), only instantiating
  ///   PolarExtentCoreSimd and an instance of get_polar_extent_core_arch for
  ///   that architecture
  class PolarExtent {
   public:
    PolarExtent(const std::shared_ptr<PointSourcePanner> &psp);
    // for testing
    PolarExtent(const std::shared_ptr<PointSourcePanner> &psp,
                std::unique_ptr<PolarExtentCore> core_impl);
    ~PolarExtent();

    PolarExtent(PolarExtent const &) = delete;
    PolarExtent &operator=(PolarExtent const &) = delete;

    void handle(Eigen::Vector3d position, double width, double height,
                double depth, Eigen::Ref<Eigen::VectorXd> out) const;

   private:
    void setup_angle_to_weight(double start_angle, double end_angle) const;
    void setup_weighting_function(Eigen::Vector3d position, double width,
                                  double height) const;

    void calc_pv_spread(Eigen::Vector3d position, double width, double height,
                        Eigen::Ref<Eigen::VectorXd> out) const;

    mutable PolarExtentCoreContext ctx;
    std::unique_ptr<PolarExtentCore> core_impl;
    std::shared_ptr<PointSourcePanner> psp;

    mutable Eigen::VectorXd pvsMin;
    mutable Eigen::VectorXd pvsMax;
  };

}  // namespace ear
