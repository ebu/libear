#include <boost/math/constants/constants.hpp>
#include <catch2/catch.hpp>
#include <complex>
#include "ear/bs2051.hpp"
#include "ear/decorrelate.hpp"
#include "ear/layout.hpp"
#include "helper/vector_approx.hpp"
#include "kissfft/kissfft.hh"

const double PI = boost::math::constants::pi<double>();

using namespace ear;

// declare private decorrelate functions
namespace ear {
  std::vector<double> designDecorrelatorBasic(int decorrelatorId, int size);
  std::vector<long> genRandMt19937(int seed, int n);
};  // namespace ear

TEST_CASE("test_gen_rand_mt19937") {
  int seed = 5489;
  int i = 10000;
  long expected = 4123659995;
  REQUIRE(genRandMt19937(seed, i)[i - 1] == expected);
};

TEST_CASE("test_design_decorrelator") {
  double rand = genRandMt19937(0, 1)[0] / static_cast<double>(UINT32_MAX - 1);
  std::vector<std::complex<double>> expected = {
      std::complex<double>(1.0, 1.0),
      std::exp(std::complex<double>(0.0, 2.0 * rand * PI))};
  std::vector<double> filt = designDecorrelatorBasic(0, 512);
  REQUIRE(filt.size() == 512);
  kissfft<double> fft(256, false);
  std::vector<std::complex<double>> actual(512);
  fft.transform_real(&filt[0], &actual[0]);
  REQUIRE(actual[0].real() == Approx(expected[0].real()));
  REQUIRE(actual[0].imag() == Approx(expected[0].imag()));
  REQUIRE(actual[1].real() == Approx(expected[1].real()));
  REQUIRE(actual[1].imag() == Approx(expected[1].imag()));
};

TEST_CASE("test_design_decorrelators") {
  Layout layout = getLayout("4+5+0").withoutLfe();
  std::vector<std::vector<double>> filters =
      designDecorrelators<double>(layout);

  // M+030 should get the second filter
  boost::optional<int> index = layout.indexForName("M+030");
  auto rightFilter = filters[boost::get<int>(index)];
  REQUIRE_VECTOR_APPROX(rightFilter, designDecorrelatorBasic(1, 512));
}
