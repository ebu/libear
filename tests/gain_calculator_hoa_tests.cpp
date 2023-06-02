#include <catch2/catch.hpp>
#include <functional>
#include "ear/bs2051.hpp"
#include "ear/ear.hpp"
#include "eigen_utils.hpp"

using namespace ear;

TEST_CASE("warnings") {
  Layout layout = getLayout("0+5+0");
  GainCalculatorHOA gc(layout);

  HOATypeMetadata tm;
  tm.orders = {0, 1, 1, 1};
  tm.degrees = {0, -1, 0, 1};

  std::vector<std::vector<double>> gains(4);
  for (auto &col : gains) col.resize(6);

  SECTION("screenRef") {
    tm.screenRef = true;
    std::vector<Warning> warnings;
    gc.calculate(tm, gains, [&](const Warning &w) { warnings.push_back(w); });

    REQUIRE(warnings.size() == 1);
    REQUIRE(warnings[0].code == Warning::Code::HOA_SCREENREF_NOT_IMPLEMENTED);
  }

  SECTION("nfcRefDist") {
    tm.nfcRefDist = 1.0;
    std::vector<Warning> warnings;
    gc.calculate(tm, gains, [&](const Warning &w) { warnings.push_back(w); });

    REQUIRE(warnings.size() == 1);
    REQUIRE(warnings[0].code == Warning::Code::HOA_NFCREFDIST_NOT_IMPLEMENTED);
  }
}

#ifndef EAR_NO_EXCEPTIONS

TEST_CASE("exceptions") {
  Layout layout = getLayout("0+5+0");
  GainCalculatorHOA gc(layout);

  HOATypeMetadata tm;
  tm.orders = {0, 1, 1, 1};
  tm.degrees = {0, -1, 0, 1};

  std::vector<std::vector<double>> gains(4);
  for (auto &col : gains) col.resize(6);

  SECTION("orders same size") {
    tm.degrees.pop_back();

    REQUIRE_THROWS_AS(gc.calculate(tm, gains), invalid_argument);
  }

  SECTION("order negative") {
    tm.orders[0] = -1;

    REQUIRE_THROWS_AS(gc.calculate(tm, gains), invalid_argument);
  }

  SECTION("degree greater than order") {
    tm.degrees[3] = 2;

    REQUIRE_THROWS_AS(gc.calculate(tm, gains), invalid_argument);
  }

  SECTION("degree greater than order neg") {
    tm.degrees[3] = -2;

    REQUIRE_THROWS_AS(gc.calculate(tm, gains), invalid_argument);
  }
}

#endif
