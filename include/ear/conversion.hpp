#pragma once
#include "metadata.hpp"

namespace ear {
  namespace conversion {
    /// structure for holding extent parameters without using an entire
    /// ObjectsTypeMetadata object
    struct EAR_EXPORT ExtentParams {
      double width;
      double height;
      double depth;
    };

    /// convert a Cartesian position to polar
    ///
    /// \rst
    /// This corresponds to
    /// :py:func:`ear.core.objectbased.conversion.point_cart_to_polar`.
    /// \endrst
    EAR_EXPORT PolarPosition pointCartToPolar(const CartesianPosition &pos);
    /// convert a polar position to Cartesian
    ///
    /// \rst
    /// This corresponds to
    /// :py:func:`ear.core.objectbased.conversion.point_polar_to_cart`.
    /// \endrst
    EAR_EXPORT CartesianPosition pointPolarToCart(const PolarPosition &pos);

    /// convert a Cartesian position and extent parameters to polar
    ///
    /// \rst
    /// This corresponds to
    /// :py:func:`ear.core.objectbased.conversion.extent_cart_to_polar`.
    /// \endrst
    EAR_EXPORT std::pair<PolarPosition, ExtentParams> extentCartToPolar(
        const CartesianPosition &pos, const ExtentParams &extent);

    /// convert a polar position and extent parameters to Cartesian
    ///
    /// \rst
    /// This corresponds to
    /// :py:func:`ear.core.objectbased.conversion.extent_polar_to_cart`.
    /// \endrst
    EAR_EXPORT std::pair<CartesianPosition, ExtentParams> extentPolarToCart(
        const PolarPosition &pos, const ExtentParams &extent);

    /// in-place conversion of Objects metadata to polar
    ///
    /// The cartesian flag is ignored, and the type of the position is used to
    /// determine whether the metadata is Cartesian or polar. If the metadata
    /// is Cartesian, then the position and extent parameters are converted to
    /// polar, and the cartesian flag is cleared.
    ///
    /// \rst
    /// This corresponds to
    /// :py:func:`ear.core.objectbased.conversion.to_polar`.
    /// \endrst
    EAR_EXPORT void toPolar(ObjectsTypeMetadata &otm);

    /// in-place conversion of Objects metadata to Cartesian
    ///
    /// The cartesian flag is ignored, and the type of the position is used to
    /// determine whether the metadata is Cartesian or polar. If the metadata
    /// is polar, then the position and extent parameters are converted to
    /// Cartesian, and the cartesian flag is set.
    ///
    /// \rst
    /// This corresponds to
    /// :py:func:`ear.core.objectbased.conversion.to_cartesian`.
    /// \endrst
    EAR_EXPORT void toCartesian(ObjectsTypeMetadata &otm);
  }  // namespace conversion
}  // namespace ear
