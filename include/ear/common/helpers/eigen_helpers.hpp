#pragma once
#include <Eigen/Core>
#include <type_traits>
#include "ear/helpers/assert.hpp"
#include "ear/helpers/output_gains.hpp"

namespace ear {

  /** @brief Check if two containers have intersecting elements
   */
  template <class InputIterator1, class InputIterator2>
  bool doIntersect(InputIterator1 first1, InputIterator1 last1,
                   InputIterator2 first2, InputIterator2 last2) {
    while (first1 != last1 && first2 != last2) {
      if (*first1 < *first2)
        ++first1;
      else if (*first2 < *first1)
        ++first2;
      else {
        return true;
      }
    }
    return false;
  }

  template <int N, int M>
  double interp(double x, const Eigen::Matrix<double, N, 1> &xp,
                const Eigen::Matrix<double, M, 1> &yp) {
    ear_assert(std::is_sorted(xp.begin(), xp.end()),
               "in interp: unsorted x values");
    ear_assert(xp.size() == yp.size(),
               "in interp: must have same number of x and y points");
    ear_assert(xp.size() >= 2, "in interp: must have at least 2 points");

    if (x <= xp(0)) {
      return yp(0);
    }
    for (int i = 0; i < xp.size() - 1; ++i) {
      if (xp(i + 1) > x) {
        double x0 = xp(i);
        double x1 = xp(i + 1);
        double y0 = yp(i);
        double y1 = yp(i + 1);
        return y0 + (y1 - y0) / (x1 - x0) * (x - x0);
      }
    }
    return yp(yp.size() - 1);
  }

  // write to a vector in a way which is compatible with OutputGains
  template <typename VecT, typename ValueT>
  void vec_write(VecT &vec, size_t i, ValueT value) {
    vec[i] = value;
  }
  template <typename ValueT>
  void vec_write(OutputGains &vec, size_t i, ValueT value) {
    vec.write(i, value);
  }

  /// `mask_write(out, mask, values)` is equivalent to numpy
  /// `out[mask] = values` for boolean masks
  template <typename OutT, typename MaskT, typename ValuesT>
  void mask_write(OutT &&out, const MaskT &mask, const ValuesT &values) {
    Eigen::Index out_size = out.size();
    Eigen::Index mask_size = mask.size();
    Eigen::Index values_size = values.size();

    ear_assert(
        out_size == mask_size,
        "in mask_write: out_size and mask_write must be the same length");

    Eigen::Index j = 0;
    for (Eigen::Index i = 0; i < mask_size; i++)
      if (mask[i]) vec_write(out, i, values[j++]);

    ear_assert(j == values_size,
               "in mask_size: length of values must equal the number of "
               "entries in mask");
  }

  /// make an eigen copy of a std::vector (or other type with size() and
  /// operator[]), for use when Eigen::Map can't be used, e.g. with
  /// std::vector<bool>
  template <typename EigenT, typename ParamT,
            typename std::enable_if<EigenT::RowsAtCompileTime == Eigen::Dynamic,
                                    int>::type = 0>
  inline EigenT copy_vector(const ParamT &x) {
    Eigen::Index size = x.size();
    EigenT rv(size);

    for (Eigen::Index i = 0; i < size; i++) rv[i] = x[i];

    return rv;
  }

  template <typename EigenT, typename ParamT,
            typename std::enable_if<EigenT::RowsAtCompileTime != Eigen::Dynamic,
                                    int>::type = 0>
  inline EigenT copy_vector(const ParamT &x) {
    Eigen::Index size = x.size();
    EigenT rv;
    ear_assert(rv.rows() == size,
               "in copy_vector: incorrect size vector for Eigen type used");

    for (Eigen::Index i = 0; i < size; i++) rv[i] = x[i];

    return rv;
  }
}  // namespace ear
