#pragma once

#include <Eigen/Core>
#include <boost/math/special_functions/factorials.hpp>
#include <boost/math/special_functions/legendre.hpp>
#include <cmath>
#include <map>
#include <utility>
#include "ear/helpers/assert.hpp"

namespace ear {
  namespace hoa {

    /// Associated Legendre function P_n^m(x), omitting the (-1)^m
    /// Condon-Shortley phase term.
    inline double Alegendre(int n, int m, double x) {
      return std::pow(-1.0, m) * boost::math::legendre_p(n, m, x);
    }

    /// Ambisonics Channel Number for order n and degree m.
    inline int to_acn(int n, int m) { return n * n + n + m; }

    /// Get the order n and degree m from a given Ambisonics Channel Number.
    inline std::pair<int, int> from_acn(int acn) {
      int n = (int)std::sqrt(acn);
      int m = acn - n * n - n;
      return {n, m};
    }

    // HOA normalisation functions corresponding to BS.2076-1 section 10.2

    enum class NormType {
      N3D,
      SN3D,
      FuMa,
    };

    inline double norm_N3D(int n, int abs_m) {
      return std::sqrt((2.0 * n + 1.0) *
                       boost::math::factorial<double>(n - abs_m) /
                       boost::math::factorial<double>(n + abs_m));
    }

    inline double norm_SN3D(int n, int abs_m) {
      return std::sqrt(boost::math::factorial<double>(n - abs_m) /
                       boost::math::factorial<double>(n + abs_m));
    }

    inline double norm_FuMa(int n, int abs_m) {
      static const std::map<std::pair<int, int>, double> conversion_factors = {
          {{0, 0}, 1.0 / std::sqrt(2.0)},
          {{1, 0}, 1.0},
          {{1, 1}, 1.0},
          {{2, 0}, 1.0},
          {{2, 1}, 2.0 / std::sqrt(3.0)},
          {{2, 2}, 2.0 / std::sqrt(3.0)},
          {{3, 0}, 1.0},
          {{3, 1}, std::sqrt(45.0 / 32.0)},
          {{3, 2}, 3.0 / std::sqrt(5.0)},
          {{3, 3}, std::sqrt(8.0 / 5.0)}};

      return conversion_factors.at({n, abs_m}) * norm_SN3D(n, abs_m);
    }

    using norm_f_t = double(int, int);

    inline norm_f_t &get_norm(NormType norm_type) {
      switch (norm_type) {
        case NormType::N3D:
          return norm_N3D;
        case NormType::SN3D:
          return norm_SN3D;
        case NormType::FuMa:
          return norm_FuMa;
        default:
          throw ear::internal_error("invalid value for norm_type");
      }
    }

    /// Spherical harmonic values for a given normalisation according to
    /// BS.2076-1 section 10.1.
    ///
    /// Here, az and el are the ADM azimuths and
    /// elevations converted to radians, rather than the HOA coordinate system.
    /// The only difference is that the elevation goes up from the equator, so
    /// we use sin rather than cos in the elevation term.
    template <typename NormT>
    double sph_harm(int n, int m, double az, double el, NormT norm) {
      double scale;
      if (m > 0)
        scale = std::sqrt(2.0) * std::cos(m * az);
      else if (m < 0)
        scale = -std::sqrt(2.0) * std::sin(m * az);
      else
        scale = 1;

      return norm(n, std::abs(m)) * Alegendre(n, std::abs(m), std::sin(el)) *
             scale;
    }

    /// get a matrix of HOA coefficients at the given positions
    template <typename NormT>
    Eigen::MatrixXd calc_Y_virt(Eigen::Matrix<double, Eigen::Dynamic, 3> points,
                                Eigen::VectorXi n, Eigen::VectorXi m,
                                NormT norm) {
      ear_assert(n.size() == m.size(), "n and m must be the same size");

      Eigen::MatrixXd Y_virt(n.size(), points.rows());
      for (Eigen::Index point_i = 0; point_i < points.rows(); point_i++) {
        for (Eigen::Index coeff_i = 0; coeff_i < n.size(); coeff_i++) {
          double az = -std::atan2(points(point_i, 0), points(point_i, 1));
          double el =
              std::atan2(points(point_i, 2),
                         std::hypot(points(point_i, 0), points(point_i, 1)));
          Y_virt(coeff_i, point_i) =
              sph_harm(n(coeff_i), m(coeff_i), az, el, norm);
        }
      }

      return Y_virt;
    }

    /// get a matrix of point-source panning values at the given points, given
    /// a function from a 3-vector to an optional vector of coefficients
    template <typename PanningFuncT>
    Eigen::MatrixXd calc_G_virt(Eigen::Matrix<double, Eigen::Dynamic, 3> points,
                                PanningFuncT panning_function) {
      Eigen::Index n_speakers =
          panning_function(Eigen::Vector3d{0, 1, 0}).get().size();

      Eigen::MatrixXd G_virt(n_speakers, points.rows());
      for (Eigen::Index point_i = 0; point_i < points.rows(); point_i++)
        G_virt.col(point_i) = panning_function(points.row(point_i)).get();

      return G_virt;
    }

    /// normalize a decode matrix such that the mean output power for the given
    /// input vectors is 1
    inline void normalize_decode_matrix(Eigen::MatrixXd &D,
                                        Eigen::MatrixXd Y_virt) {
      D *= std::sqrt(Y_virt.cols()) / (D * Y_virt).norm();
    }

    /// Vector which when multiplied with a HOA signal converts it from
    /// norm_from to norm_to
    template <typename NormToT, typename NormFromT>
    inline Eigen::VectorXd normalisation_conversion(Eigen::VectorXi n,
                                                    Eigen::VectorXi m,
                                                    NormToT norm_to,
                                                    NormFromT norm_from) {
      ear_assert(n.size() == m.size(), "n and m must be the same size");

      Eigen::VectorXd conversion(n.size());
      for (Eigen::Index coeff_i = 0; coeff_i < n.size(); coeff_i++)
        conversion(coeff_i) = norm_to(n(coeff_i), std::abs(m(coeff_i))) /
                              norm_from(n(coeff_i), std::abs(m(coeff_i)));

      return conversion;
    }

    /// Decoder matrix design using the AllRAD[0] technique.
    /// [0] F. Zotter and M. Frank, "All-round ambisonic panning and decoding,"
    /// JAES, vol. 60, no. 10, pp. 807-820, 2012.
    /// http://www.aes.org/e-lib/browse.cfm?elib=16554
    template <typename PanningFuncT, typename NormT>
    Eigen::MatrixXd allrad_design(
        Eigen::Matrix<double, Eigen::Dynamic, 3> points,
        PanningFuncT panning_function, Eigen::VectorXi n, Eigen::VectorXi m,
        NormT norm) {
      Eigen::MatrixXd Y_virt = calc_Y_virt(points, n, m, norm_N3D);
      Eigen::MatrixXd D_virt = Y_virt.transpose() / points.rows();
      Eigen::MatrixXd G_virt = calc_G_virt(points, panning_function);

      Eigen::MatrixXd D = G_virt * D_virt;

      normalize_decode_matrix(D, Y_virt);

      D *= normalisation_conversion(n, m, norm_N3D, norm).asDiagonal();

      return D;
    }

    /// load spherical-t-design points used to be used for AllRAD design
    Eigen::Matrix<double, Eigen::Dynamic, 3> load_points();

  }  // namespace hoa
}  // namespace ear
