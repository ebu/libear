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
};  // namespace ear

void check_decorrelator(const std::vector<double> &dec) {
  // decorrelator id 7, generated with ear
  REQUIRE(dec.size() == 512);
  REQUIRE(dec.at(0) == Approx(-0.1124280906086625));
  REQUIRE(dec.at(1) == Approx(-0.00944671630601479));
  REQUIRE(dec.at(255) == Approx(0.057714955000898516));
  REQUIRE(dec.at(256) == Approx(-0.018996037984052125));
  REQUIRE(dec.at(510) == Approx(0.08336121588594464));
  REQUIRE(dec.at(511) == Approx(-0.012216595581941523));
}

TEST_CASE("test_designDecorrelatorBasic") {
  auto dec = designDecorrelatorBasic(7, 512);
  check_decorrelator(dec);
}

TEST_CASE("test_design_decorrelators") {
  Layout layout = getLayout("4+5+0").withoutLfe();
  std::vector<std::vector<double>> filters =
      designDecorrelators<double>(layout);

  // M+030 should get the second filter
  boost::optional<int> index = layout.indexForName("M+030");
  auto rightFilter = filters[boost::get<int>(index)];
  REQUIRE_VECTOR_APPROX(rightFilter, designDecorrelatorBasic(1, 512));
}
