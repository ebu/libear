#define EIGEN_RUNTIME_NO_MALLOC
#include <Eigen/Core>
#include <algorithm>
#include <catch2/catch.hpp>
#include <vector>
#include "ear/dsp/gain_interpolator.hpp"
#include "ear/dsp/ptr_adapter.hpp"
#include "eigen_utils.hpp"

using namespace ear;
using namespace ear::dsp;

// adapters to make apply_interp, apply_constant and process work with Eigen
// types
template <typename Interp = LinearInterpSingle, typename In, typename Out,
          typename Point>
void apply_interp(In &&in, Out &&out, SampleIndex block_start,
                  SampleIndex start, SampleIndex end, const Point &start_point,
                  const Point &end_point) {
  PtrAdapter in_p(in.cols());
  in_p.set_eigen(in);
  PtrAdapter out_p(out.cols());
  out_p.set_eigen(out);

  Interp::apply_interp(in_p.ptrs(), out_p.ptrs(), 0, in.rows(), block_start,
                       start, end, start_point, end_point);
}

template <typename Interp = LinearInterpSingle, typename In, typename Out,
          typename Point>
void apply_constant(In &&in, Out &&out, const Point &point) {
  PtrAdapter in_p(in.cols());
  in_p.set_eigen(in);
  PtrAdapter out_p(out.cols());
  out_p.set_eigen(out);

  Interp::apply_constant(in_p.ptrs(), out_p.ptrs(), 0, in.rows(), point);
}

template <typename Interp>
void process(Interp &interp, SampleIndex block_start,
             const Eigen::Ref<const Eigen::MatrixXf> &in,
             Eigen::Ref<Eigen::MatrixXf> out) {
  PtrAdapterConst in_p(in.cols());
  in_p.set_eigen(in);
  PtrAdapter out_p(out.cols());
  out_p.set_eigen(out);

  interp.process(block_start, in.rows(), in_p.ptrs(), out_p.ptrs());
}

// ensure that LinearInterpSingle is correct so that we can use it to generate
// expected results in tests of GainInterpolator; otherwise we would just end up
// reimplementing it here

TEST_CASE("LinearInterpSingle::apply_interp") {
  Eigen::VectorXf input = Eigen::VectorXf::Random(100);
  Eigen::VectorXf output = Eigen::VectorXf::Zero(100);

  apply_interp<LinearInterpSingle>(input, output, 100, 50, 250, 0.2f, 0.8f);

  Eigen::VectorXf p = Eigen::VectorXf::LinSpaced(200, 0, 199) / 200.0;
  Eigen::VectorXf gain_ramp = 0.8f * p.array() + (1.0f - p.array()) * 0.2f;
  Eigen::VectorXf expected =
      gain_ramp(Eigen::seqN(50, 100)).cwiseProduct(input);

  CHECK_THAT(output, IsApprox(expected));
}

TEST_CASE("LinearInterpSingle::apply_constant") {
  Eigen::VectorXf input = Eigen::VectorXf::Random(100);
  Eigen::VectorXf output = Eigen::VectorXf::Zero(100);

  apply_constant<LinearInterpSingle>(input, output, 0.3f);

  Eigen::VectorXf expected = 0.3f * input;

  CHECK_THAT(output, IsApprox(expected));
}

void run_test(GainInterpolator<LinearInterpSingle> &interp,
              Eigen::VectorXf &input, Eigen::VectorXf &expected_output,
              const std::vector<Eigen::Index> &block_sizes) {
  for (auto block_size : block_sizes) {
    Eigen::VectorXf output = Eigen::VectorXf::Zero(input.size());

    Eigen::internal::set_is_malloc_allowed(false);
    for (Eigen::Index offset = 0; offset < input.size(); offset += block_size) {
      auto block =
          Eigen::seq(offset, std::min(offset + block_size, input.size()) - 1);

      process(interp, offset, input(block), output(block));
    }
    Eigen::internal::set_is_malloc_allowed(true);

    CHECK_THAT(output, IsApprox(expected_output));
  }
}

// check that GainInterpolator makes the right calls to the templated InterpType

TEST_CASE("basic") {
  GainInterpolator<LinearInterpSingle> interp;

  interp.interp_points.emplace_back(100, 0.2f);
  interp.interp_points.emplace_back(200, 0.8f);
  interp.interp_points.emplace_back(300, 0.8f);
  interp.interp_points.emplace_back(400, 0.3f);

  Eigen::VectorXf input = Eigen::VectorXf::Random(500);
  Eigen::VectorXf expected_output = Eigen::VectorXf::Zero(500);

  auto block = Eigen::seqN(0, 100);
  apply_constant(input(block), expected_output(block), 0.2f);

  block = Eigen::seqN(100, 100);
  apply_interp(input(block), expected_output(block), 100, 100, 200, 0.2f, 0.8f);

  block = Eigen::seqN(200, 100);
  apply_constant(input(block), expected_output(block), 0.8f);

  block = Eigen::seqN(300, 100);
  apply_interp(input(block), expected_output(block), 300, 300, 400, 0.8f, 0.3f);

  block = Eigen::seqN(400, 100);
  apply_constant(input(block), expected_output(block), 0.3f);

  run_test(interp, input, expected_output, {50, 75, 100, 500});
}

TEST_CASE("step") {
  GainInterpolator<LinearInterpSingle> interp;

  interp.interp_points.emplace_back(100, 0.2f);
  interp.interp_points.emplace_back(200, 0.2f);
  interp.interp_points.emplace_back(200, 0.8f);
  interp.interp_points.emplace_back(300, 0.8f);

  Eigen::VectorXf input = Eigen::VectorXf::Random(400);
  Eigen::VectorXf expected_output = Eigen::VectorXf::Zero(400);

  auto block = Eigen::seqN(0, 200);
  apply_constant(input(block), expected_output(block), 0.2f);

  block = Eigen::seqN(200, 200);
  apply_constant(input(block), expected_output(block), 0.8f);

  run_test(interp, input, expected_output, {50, 75, 100, 400});
}

TEST_CASE("only_step") {
  GainInterpolator<LinearInterpSingle> interp;

  interp.interp_points.emplace_back(100, 0.2f);
  interp.interp_points.emplace_back(100, 0.8f);

  Eigen::VectorXf input = Eigen::VectorXf::Random(200);
  Eigen::VectorXf expected_output = Eigen::VectorXf::Zero(200);

  auto block = Eigen::seqN(0, 100);
  apply_constant(input(block), expected_output(block), 0.2f);

  block = Eigen::seqN(100, 100);
  apply_constant(input(block), expected_output(block), 0.8f);

  run_test(interp, input, expected_output, {50, 75, 100, 200});
}

TEST_CASE("one_point") {
  GainInterpolator<LinearInterpSingle> interp;

  interp.interp_points.emplace_back(100, 0.2);

  Eigen::VectorXf input = Eigen::VectorXf::Random(200);
  Eigen::VectorXf expected_output = Eigen::VectorXf::Zero(200);

  auto block = Eigen::seqN(0, 200);
  apply_constant(input(block), expected_output(block), 0.2f);

  run_test(interp, input, expected_output, {50, 75, 100, 200});
}

// tests for the other InterpTypes

TEST_CASE("vector") {
  GainInterpolator<LinearInterpVector> interp;

  std::vector<float> a{0.0f, 1.0f};
  std::vector<float> b{1.0f, 0.0f};

  interp.interp_points.emplace_back(100, a);
  interp.interp_points.emplace_back(200, b);

  Eigen::VectorXf input = Eigen::VectorXf::Random(300);
  Eigen::MatrixXf output = Eigen::MatrixXf::Zero(300, 2);

  Eigen::internal::set_is_malloc_allowed(false);
  process(interp, 0, input, output);
  Eigen::internal::set_is_malloc_allowed(true);

  Eigen::MatrixXf expected_output = Eigen::MatrixXf::Zero(300, 2);
  auto run_channel = [&](Eigen::Index out) {
    Eigen::MatrixXf tmp = Eigen::MatrixXf::Zero(300, 1);

    GainInterpolator<LinearInterpSingle> interp_test;
    interp_test.interp_points.emplace_back(100, a[out]);
    interp_test.interp_points.emplace_back(200, b[out]);

    process(interp_test, 0, input, tmp);

    expected_output(Eigen::all, out) += tmp;
  };
  run_channel(0);
  run_channel(1);

  CHECK_THAT(output, IsApprox(expected_output));
}

TEST_CASE("matrix") {
  GainInterpolator<LinearInterpMatrix> interp;

  std::vector<std::vector<float>> a{{0.0f, 0.3f}, {0.5f, 0.0f}};
  std::vector<std::vector<float>> b{{0.6f, 0.0f}, {0.0f, 0.7f}};

  interp.interp_points.emplace_back(100, a);
  interp.interp_points.emplace_back(200, b);

  Eigen::MatrixXf input = Eigen::MatrixXf::Random(300, 2);
  Eigen::MatrixXf output = Eigen::MatrixXf::Zero(300, 2);

  Eigen::internal::set_is_malloc_allowed(false);
  process(interp, 0, input, output);
  Eigen::internal::set_is_malloc_allowed(true);

  Eigen::MatrixXf expected_output = Eigen::MatrixXf::Zero(300, 2);
  auto run_channel = [&](Eigen::Index in, Eigen::Index out) {
    Eigen::MatrixXf tmp = Eigen::MatrixXf::Zero(300, 1);

    GainInterpolator<LinearInterpSingle> interp_test;
    interp_test.interp_points.emplace_back(100, a[in][out]);
    interp_test.interp_points.emplace_back(200, b[in][out]);

    process(interp_test, 0, input(Eigen::all, in), tmp);

    expected_output(Eigen::all, out) += tmp;
  };
  run_channel(0, 0);
  run_channel(0, 1);
  run_channel(1, 0);
  run_channel(1, 1);

  CHECK_THAT(output, IsApprox(expected_output));
}
