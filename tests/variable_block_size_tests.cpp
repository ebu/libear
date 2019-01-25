#define EIGEN_RUNTIME_NO_MALLOC
#include <Eigen/Core>
#include <catch2/catch.hpp>
#include "ear/dsp/ptr_adapter.hpp"
#include "ear/dsp/variable_block_size.hpp"

using namespace ear;
using namespace ear::dsp;

TEST_CASE("test") {
  // dummy process function and parameters
  Eigen::Index inner_block_size = 512;
  Eigen::Index channels_in = 2;
  Eigen::Index channels_out = 4;

  auto do_process = [&](size_t block_size, const float *const *in,
                        float *const *out) {
    for (size_t i = 0; i < block_size; i++) {
      out[0][i] = in[0][i] * 2.0;
      out[1][i] = in[1][i] * 3.0;
      out[2][i] = in[0][i] * 4.0;
      out[3][i] = in[1][i] * 5.0;
    }
  };

  auto process = [&](const float *const *in, float *const *out) {
    do_process(inner_block_size, in, out);
  };

  // wrap this in an adapter
  VariableBlockSizeAdapter adapter(inner_block_size, channels_in, channels_out,
                                   process);
  PtrAdapter input_ptrs(channels_in);
  PtrAdapter output_ptrs(channels_out);

  // the test structure
  Eigen::VectorXi block_sizes(5);
  block_sizes << 0, 512, 1024, 300, 500;
  int test_len = block_sizes.sum();

  // generate an input and the expected output, which is shifted by
  // inner_block_size
  Eigen::MatrixXf input = Eigen::MatrixXf::Random(test_len, channels_in);

  Eigen::MatrixXf expected_output =
      Eigen::MatrixXf::Zero(test_len, channels_out);
  input_ptrs.set_eigen(input);
  output_ptrs.set_eigen(expected_output, inner_block_size);
  do_process(test_len - inner_block_size, input_ptrs.ptrs(),
             output_ptrs.ptrs());

  // run with the defined block sizes
  Eigen::MatrixXf output = Eigen::MatrixXf::Random(test_len, channels_out);
  Eigen::Index offset = 0;
  for (int block_size : block_sizes) {
    Eigen::internal::set_is_malloc_allowed(false);
    input_ptrs.set_eigen(input, offset);
    output_ptrs.set_eigen(output, offset);
    adapter.process(block_size, input_ptrs.ptrs(), output_ptrs.ptrs());
    Eigen::internal::set_is_malloc_allowed(true);
    offset += block_size;
  }

  // check the output
  REQUIRE(expected_output == output);
}
