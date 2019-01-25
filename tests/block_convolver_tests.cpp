#define EIGEN_RUNTIME_NO_MALLOC
#include <catch2/catch.hpp>
#include <cstring>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include "block_convolver_test_utils.hpp"
#include "ear/dsp/block_convolver.hpp"
#include "ear/fft.hpp"

using namespace ear;
using namespace ear::dsp::block_convolver;

FFTImpl<float> &fft = get_fft_kiss<float>();

/// A test of the block convolver.
///
/// This class represents the inputs to a BlockConvolver across several blocks,
/// including the filters to switch between and the input data.
///
/// Once the inputs have been set up (see public attributes), calling run will
/// apply both a real convolver and a simulated convolver to these inputs, and
/// compare the results to check that they are equivalent.
///
/// The important bits are irs, the definitions of the impulse responses; and
/// ir_for_block, which specifies which impulse response to switch to before
/// each block.
class ConvolutionTest {
 public:
  /// block size for BlockConvolver
  size_t block_size;
  /// number of blocks to process
  size_t num_blocks;
  /// number of samples to process
  size_t len;
  /// impulse response to set before starting the test; don't set (creating a
  /// fade up) if negative)
  int initial_ir;
  /// the number of blocks to create the BlockConvolver with; will be set to
  /// the maximum filter length used if it's too low.
  size_t max_num_blocks;
  /// pass NULL for blocks containing all zero samples?
  bool null_for_zeros;
  /// Which constructor to use for BlockConvolver
  enum {
    NO_FILTER,  ///< standard, without a filter
    WITH_FILTER_NUM_BLOCKS,  ///< specify a filter and a number of blocks
    WITH_FILTER_NO_NUM_BLOCKS  ///< specify just a filter
  } constructor;

  /// Maximum acceptable error between the two versions.
  float max_error;

  /// available inpulse responses
  std::vector<Eigen::VectorXf> irs;
  /// Which ir index to use for each input block; should be num_blocks long.
  /// Negative numbers indicate no filter.
  std::vector<int> ir_for_block;

  /// Input data; len long.
  Eigen::VectorXf input;

  /// Output data from the test implementation; len long.
  Eigen::VectorXf test_output;
  /// Output data from the real implementation; len long.
  Eigen::VectorXf real_output;

  ConvolutionTest(size_t block_size, size_t num_blocks)
      : block_size(block_size),
        num_blocks(num_blocks),
        len(block_size * num_blocks),
        initial_ir(-1),
        max_num_blocks(0),
        null_for_zeros(false),
        constructor(NO_FILTER),
        max_error(1e-6f),
        input(len),
        test_output(Eigen::VectorXf::Zero(len)),
        real_output(Eigen::VectorXf::Zero(len)) {}

  /// Run a slow convolution on the specified inputs.
  void run_test_convolve(const std::string &out_dir = "") {
    // For each filter, create modified input data with fades applied, then
    // convolve and add to the output.
    for (size_t i = 0; i < irs.size(); i++) {
      Eigen::VectorXf input_for_ir = Eigen::VectorXf::Zero(len);

      // Apply fades for each block
      for (size_t block = 0; block < num_blocks; block++) {
        const size_t offset = block_size * block;
        // Was this filter active in the last block , or this block?
        bool last_block =
            (int)i == (block == 0 ? initial_ir : ir_for_block[block - 1]);
        bool this_block = (int)i == ir_for_block[block];

        // appropriate fades for this block
        if (last_block && this_block)
          input_for_ir(Eigen::seqN(offset, block_size)) =
              input(Eigen::seqN(offset, block_size));
        else if (!last_block && this_block)
          fade_up(&input_for_ir[offset], &input[offset], block_size);
        else if (last_block && !this_block)
          fade_down(&input_for_ir[offset], &input[offset], block_size);
      }

      write_array(out_dir, "input_" + std::to_string(i) + ".dat", len,
                  &input_for_ir[0]);

      // convolve and mix
      Eigen::VectorXf output_for_ir = Eigen::VectorXf::Zero(len);
      convolve(len, &output_for_ir[0], &input_for_ir[0], irs[i].size(),
               &irs[i][0]);
      test_output += output_for_ir;
    }
  }

  /// Run the real convolution.
  void run_real_convolve(const Context &ctx) {
    // Create filters from irs, while finding the required number of blocks.
    std::vector<Filter> filters;
    for (size_t i = 0; i < irs.size(); i++) {
      Filter filter(ctx, irs[i].rows(), irs[i].data());
      if (filter.num_blocks() > max_num_blocks)
        max_num_blocks = filter.num_blocks();
      filters.push_back(std::move(filter));
    }

    std::unique_ptr<BlockConvolver> convolver;

    // Create the filter
    switch (constructor) {
      case NO_FILTER:
        convolver.reset(new BlockConvolver(ctx, max_num_blocks));
        break;
      case WITH_FILTER_NUM_BLOCKS:
        convolver.reset(
            new BlockConvolver(ctx, filters[initial_ir], max_num_blocks));
        break;
      case WITH_FILTER_NO_NUM_BLOCKS:
        convolver.reset(new BlockConvolver(ctx, filters[initial_ir]));
        break;
    }

    Eigen::internal::set_is_malloc_allowed(false);

    // don't set the starting filter if initial_ir is invalid
    if (initial_ir >= 0) {
      convolver->set_filter(filters[initial_ir]);
    }

    // filter each block
    for (size_t block = 0; block < num_blocks; block++) {
      const size_t offset = block_size * block;
      auto block_sel = Eigen::seqN(offset, block_size);

      // crossfade if the filter has changed
      int this_filter = ir_for_block[block];
      int last_filter = block == 0 ? initial_ir : ir_for_block[block - 1];
      if (last_filter != this_filter) {
        if (this_filter >= 0)
          convolver->crossfade_filter(filters[this_filter]);
        else
          convolver->fade_down();
      }

      // filter the block to the output; possibly passing null if zeros are
      // detected
      if (null_for_zeros && (input(block_sel).array() == 0.0).all()) {
        convolver->process(nullptr, real_output(block_sel).data());
      } else {
        convolver->process(input(block_sel).data(),
                           real_output(block_sel).data());
      }
    }
    Eigen::internal::set_is_malloc_allowed(true);
  }

  /// Run the simulated and real convolutions, possibly writing the input and
  /// output datas to files.
  /// @param out_dir Output dir for data files; should end with a trailing
  /// slash.
  void run(const Context &ctx, const std::string &out_dir = "") {
    run_test_convolve(out_dir);
    run_real_convolve(ctx);

    write_array(out_dir, "input.dat", len, &input[0]);
    write_array(out_dir, "test_output.dat", len, &test_output[0]);
    write_array(out_dir, "real_output.dat", len, &real_output[0]);

    for (size_t i = 0; i < len; i++) {
      CHECK(std::fabs(test_output[i] - real_output[i]) < max_error);
    }
  }
};

TEST_CASE("filter_correct_num_blocks") {
  Context ctx(512, fft);
  // auto ctx = std::make_shared<BlockConvolver::Context>(512, fft);
  auto coeff = generate_random(2000, 100, 0);

  CHECK(Filter(ctx, 1, &coeff[0]).num_blocks() == 1);
  CHECK(Filter(ctx, 511, &coeff[0]).num_blocks() == 1);
  CHECK(Filter(ctx, 512, &coeff[0]).num_blocks() == 1);
  CHECK(Filter(ctx, 513, &coeff[0]).num_blocks() == 2);
}

TEST_CASE("single_block") {
  Context ctx(512, fft);
  ConvolutionTest t(512, 1);
  t.irs.emplace_back(generate_random(100, 10, 1));
  t.initial_ir = 0;
  t.ir_for_block = {0};
  t.input = generate_random(512, 200, 0);
  t.run(ctx);
}

TEST_CASE("two_blocks") {
  Context ctx(512, fft);
  ConvolutionTest t(512, 2);
  t.irs.emplace_back(generate_random(512 * 3, 20, 1));
  t.initial_ir = 0;
  t.ir_for_block = {0, 0};
  t.input = generate_random(512 * 2, 300, 0);
  t.run(ctx);
}

TEST_CASE("fade_once") {
  Context ctx(512, fft);
  ConvolutionTest t(512, 3);
  t.irs.emplace_back(generate_random(100, 10, 1));
  t.irs.emplace_back(generate_random(512, 10, 2));
  t.initial_ir = 0;
  t.ir_for_block = {0, 1, 1};
  t.input = generate_random(512 * 3, 300, 0);
  t.run(ctx);
}

TEST_CASE("fade_at_start_from_silence") {
  Context ctx(512, fft);
  ConvolutionTest t(512, 2);
  t.irs.emplace_back(generate_random(512, 10, 1));
  t.ir_for_block = {0, 0};
  t.input = generate_random(512 * 2, 300, 0);
  t.run(ctx);
}

TEST_CASE("fade_to_silence") {
  Context ctx(512, fft);
  ConvolutionTest t(512, 3);
  t.irs.emplace_back(generate_random(512, 10, 1));
  t.initial_ir = 0;
  t.ir_for_block = {0, -1, -1};
  t.input = generate_random(512 * 3, 300, 0);
  t.run(ctx);
}

TEST_CASE("fade_from_silence") {
  Context ctx(512, fft);
  ConvolutionTest t(512, 3);
  t.irs.emplace_back(generate_random(512, 10, 1));
  t.ir_for_block = {-1, 0, 0};
  t.input = generate_random(512 * 3, 300, 0);
  t.run(ctx);
}

TEST_CASE("fade_at_start_from_filter") {
  Context ctx(512, fft);
  ConvolutionTest t(512, 2);
  t.irs.emplace_back(generate_random(512, 10, 1));
  t.irs.emplace_back(generate_random(512, 10, 2));
  t.initial_ir = 0;
  t.ir_for_block = {1, 1};
  t.input = generate_random(512 * 2, 300, 0);
  t.run(ctx);
}

TEST_CASE("smaller_filter_than_convolver") {
  Context ctx(512, fft);
  ConvolutionTest t(512, 4);
  t.irs.emplace_back(generate_random(512 * 2, 20, 1));
  t.initial_ir = 0;
  t.ir_for_block = {0, 0, 0, 0};
  t.input = generate_random(512 * 4, 300, 0);
  t.max_num_blocks = 3;
  t.run(ctx);
}

TEST_CASE("different_num_blocks") {
  Context ctx(512, fft);
  ConvolutionTest t(512, 4);
  t.irs.emplace_back(generate_random(512 * 2, 20, 1));
  t.irs.emplace_back(generate_random(512 * 3, 20, 2));
  t.initial_ir = 0;
  t.ir_for_block = {0, 1, 1, 1};
  t.input = generate_random(512 * 4, 300, 0);
  t.run(ctx);
}

TEST_CASE("zero_input_blocks") {
  Context ctx(512, fft);
  ConvolutionTest t(512, 5);
  t.irs.emplace_back(generate_random(512 * 2, 20, 1));
  t.initial_ir = 0;
  t.ir_for_block = {0, 0, 0, 0, 0};
  t.input = generate_random(512 * 5, 300, 0);
  t.input(Eigen::seqN(512, 512 * 3)).setZero();
  t.run(ctx);
}

TEST_CASE("null_input_blocks") {
  Context ctx(512, fft);
  ConvolutionTest t(512, 5);
  t.irs.emplace_back(generate_random(512 * 2, 20, 1));
  t.initial_ir = 0;
  t.ir_for_block = {0, 0, 0, 0, 0};
  t.input = generate_random(512 * 5, 300, 0);
  t.input(Eigen::seqN(512, 512 * 3)).setZero();
  t.null_for_zeros = true;
  t.run(ctx);
}

TEST_CASE("lots_of_filters") {
  Context ctx(512, fft);
  ConvolutionTest t(512, 9);
  t.irs.emplace_back(generate_random(512 * 2, 20, 1));
  t.irs.emplace_back(generate_random(512 * 3, 20, 2));
  t.irs.emplace_back(generate_random(512 * 1, 20, 3));
  t.irs.emplace_back(generate_random(512 * 4, 20, 4));
  t.initial_ir = 0;
  t.ir_for_block = {0, 1, 2, 3, 3, 2, 2, 1, 0};
  t.input = generate_random(512 * 9, 500, 0);
  t.run(ctx);
}

TEST_CASE("construct_with_filter") {
  Context ctx(512, fft);
  ConvolutionTest t(512, 3);
  t.irs.emplace_back(generate_random(512 * 2, 20, 1));
  t.initial_ir = 0;
  t.ir_for_block = {0, 0, 0};
  t.constructor = ConvolutionTest::WITH_FILTER_NUM_BLOCKS;
  t.input = generate_random(512 * 3, 300, 0);
  t.run(ctx);
}

TEST_CASE("construct_with_filter_no_blocks") {
  Context ctx(512, fft);
  ConvolutionTest t(512, 3);
  t.irs.emplace_back(generate_random(512 * 2, 20, 1));
  t.initial_ir = 0;
  t.ir_for_block = {0, 0, 0};
  t.constructor = ConvolutionTest::WITH_FILTER_NO_NUM_BLOCKS;
  t.input = generate_random(512 * 3, 300, 0);
  t.run(ctx);
}
