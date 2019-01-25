#define EIGEN_RUNTIME_NO_MALLOC
#include <Eigen/Core>
#include <catch2/catch.hpp>
#include "ear/dsp/delay_buffer.hpp"
#include "ear/dsp/ptr_adapter.hpp"
#include "ear/helpers/assert.hpp"

using namespace ear;
using namespace ear::dsp;

TEST_CASE("delay_buffer") {
  int delay = 128;
  DelayBuffer db(5, delay);

  REQUIRE(db.get_delay() == delay);

  Eigen::VectorXi block_sizes(3);
  block_sizes << 64, 128, 256;

  int testlen = block_sizes.sum();
  Eigen::MatrixXf input = Eigen::MatrixXf::Random(testlen, 5);
  Eigen::MatrixXf output = Eigen::MatrixXf::Zero(testlen, 5);
  Eigen::MatrixXf expected_output = Eigen::MatrixXf::Zero(testlen, 5);

  expected_output(Eigen::seqN(delay, testlen - delay), Eigen::all) =
      input(Eigen::seqN(0, testlen - delay), Eigen::all);

  PtrAdapter in_ptrs(5), out_ptrs(5);

  Eigen::internal::set_is_malloc_allowed(false);
  int offset = 0;
  for (auto &block_size : block_sizes) {
    auto block = Eigen::seqN(offset, block_size);
    in_ptrs.set_eigen(input(block, Eigen::all));
    out_ptrs.set_eigen(output(block, Eigen::all));
    db.process(block_size, in_ptrs.ptrs(), out_ptrs.ptrs());
    offset += block_size;
  }
  Eigen::internal::set_is_malloc_allowed(true);

  REQUIRE(output == expected_output);
}

TEST_CASE("single_channel") {
  int delay = 128;
  DelayBuffer db(1, delay);

  int testlen = 512;
  Eigen::VectorXf input = Eigen::VectorXf::Random(testlen);
  Eigen::VectorXf output = Eigen::VectorXf::Zero(testlen);
  Eigen::VectorXf expected_output = Eigen::VectorXf::Zero(testlen);

  expected_output(Eigen::seqN(delay, testlen - delay)) =
      input(Eigen::seqN(0, testlen - delay));

  PtrAdapter in_ptrs(1), out_ptrs(1);

  Eigen::internal::set_is_malloc_allowed(false);
  in_ptrs.set_eigen(input);
  out_ptrs.set_eigen(output);
  db.process(testlen, in_ptrs.ptrs(), out_ptrs.ptrs());
  Eigen::internal::set_is_malloc_allowed(true);

  REQUIRE(output == expected_output);
}
