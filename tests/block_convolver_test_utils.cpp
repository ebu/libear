#include "block_convolver_test_utils.hpp"
#include <Eigen/Core>
#include <cassert>
#include <cmath>
#include <fstream>
#include <random>

/// Generate random impulses.
/// @param len length of returned buffer
/// @param num_nonzero The number of values to randomise
/// @param seedval Seed for rng.
Eigen::VectorXf generate_random(size_t len, size_t num_nonzero,
                                long int seedval) {
  Eigen::VectorXf data = Eigen::VectorXf::Zero(len);

  std::mt19937 gen(seedval);
  for (size_t i = 0; i < num_nonzero; i++) {
    data[std::uniform_int_distribution<int>(0, len - 1)(gen)] =
        std::uniform_real_distribution<float>()(gen);
  }

  return data;
}

/// Simple convolution implementation.
/// @param len output and input lengths
/// @param out[out] output data
/// @param out[in] input data
/// @param ir_len length of impulse response
/// @param ir impulse response data
void convolve(size_t len, float *out, float *in, size_t ir_len, float *ir) {
  for (size_t i = 0; i < len; i++) {
    float total = 0.0;
    for (size_t j = 0; j < ir_len; j++) {
      if (j > i) break;
      total += in[i - j] * ir[j];
    }
    out[i] = total;
  }
}

/// Fade up linearly over a block.
void fade_up(float *out, float *in, size_t n) {
  float i_scale = 1.0f / n;

  for (size_t i = 0; i < n; i++) {
    float a_v = (float)i * i_scale;
    out[i] = a_v * in[i];
  }
}

/// Fade down linearly over a block.
void fade_down(float *out, float *in, size_t n) {
  float i_scale = 1.0f / n;

  for (size_t i = 0; i < n; i++) {
    float a_v = (float)i * i_scale;
    out[i] = (1 - a_v) * in[i];
  }
}

/// Write a float array to a file, readable by numpy.loadtext
void write_array(const std::string &base, const std::string &name, size_t n,
                 float *x) {
  if (base != "") {
    std::ofstream out(base + name);
    assert(out.is_open());
    for (size_t i = 0; i < n; i++) out << x[i] << std::endl;
  }
}
