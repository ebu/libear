#pragma once
#include <Eigen/Core>
#include <cstddef>
#include <string>

/// Generate random impulses.
/// @param len length of returned buffer
/// @param num_nonzero The number of values to randomise
/// @param seedval Seed for rng.
Eigen::VectorXf generate_random(size_t len, size_t num_nonzero,
                                long int seedval);

/// Simple convolution implementation.
/// @param len output and input lengths
/// @param out[out] output data
/// @param out[in] input data
/// @param ir_len length of impulse response
/// @param ir impulse response data
void convolve(size_t len, float *out, float *in, size_t ir_len, float *ir);

/// Fade up linearly over a block.
void fade_up(float *out, float *in, size_t n);

/// Fade down linearly over a block.
void fade_down(float *out, float *in, size_t n);

/// Write a float array to a file, readable by numpy.loadtext
void write_array(const std::string &base, const std::string &name, size_t n,
                 float *x);
