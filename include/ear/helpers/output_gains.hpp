#pragma once
#include <algorithm>
#include <cstddef>
#include <vector>
#include "ear/exceptions.hpp"

namespace ear {
  // polymorphic interface for writing a vector of gains
  class OutputGains {
   public:
    // throw an invalid_argument if the size is not n
    virtual void check_size(size_t n) = 0;
    // get the size
    virtual size_t size() = 0;
    // write x at position i
    virtual void write(size_t i, double x) = 0;
    // set all coefficients to zero
    virtual void zero() = 0;

    virtual ~OutputGains() {}

    // copy from an arbitrary vector supporting .size() and operator[]
    template <typename T>
    void write_vector(T vector) {
      check_size(vector.size());
      for (size_t i = 0; i < (size_t)vector.size(); i++) {
        write(i, vector[i]);
      }
    }
  };

  // implementation of OutputGains, writing to a reference to a std::vector
  template <typename T>
  class OutputGainsT : public OutputGains {
   private:
    std::vector<T> &vec;

   public:
    OutputGainsT(std::vector<T> &vec) : vec(vec) {}
    virtual void check_size(size_t n) override {
      if (vec.size() != n) {
        throw invalid_argument("incorrect size for output vector");
      }
    }
    virtual size_t size() override { return vec.size(); }
    virtual void write(size_t i, double x) override { vec[i] = (T)x; }
    virtual void zero() override { std::fill(vec.begin(), vec.end(), (T)0.0); }
  };

  // polymorphic interface for writing a matrix of gains
  class OutputGainMat {
   public:
    // throw an invalid_argument if the storage is not of size (rows, cols)
    virtual void check_size(size_t rows, size_t cols) = 0;
    // get the number of rows
    virtual size_t rows() = 0;
    // get the number of cols
    virtual size_t cols() = 0;
    // write x at (row, col)
    virtual void write(size_t row, size_t col, double x) = 0;
    // zero all coefficients
    virtual void zero() = 0;

    virtual ~OutputGainMat() {}

    // copy from an Eigen matrix
    template <typename T>
    void write_mat(T mat) {
      check_size(mat.rows(), mat.cols());
      for (size_t col = 0; col < (size_t)mat.cols(); col++)
        for (size_t row = 0; row < (size_t)mat.rows(); row++)
          write(row, col, mat(row, col));
    }
  };

  // OutputGainMat which operates writes to a vector of vectors in column major
  // format
  template <typename T>
  class OutputGainMatVecT : public OutputGainMat {
   private:
    std::vector<std::vector<T>> &mat;

   public:
    OutputGainMatVecT(std::vector<std::vector<T>> &mat) : mat(mat) {}
    virtual void check_size(size_t rows, size_t cols) override {
      if (mat.size() != cols)
        throw invalid_argument("incorrect number of cols in output matrix");
      for (auto &col : mat)
        if (col.size() != rows)
          throw invalid_argument(
              "incorrect number of rows in output matrix column");
    }
    virtual size_t rows() override { return mat[0].size(); }
    virtual size_t cols() override { return mat.size(); }
    virtual void write(size_t row, size_t col, double x) override {
      mat[col][row] = (T)x;
    }
    virtual void zero() override {
      for (size_t col = 0; col < cols(); col++)
        for (size_t row = 0; row < rows(); row++) write(row, col, 0.0);
    }
  };
}  // namespace ear
