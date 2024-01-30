#include <boost/make_unique.hpp>
#include <cassert>
#include <catch2/catch.hpp>
#include <cmath>
#include <map>
#include <string>
#include <vector>
#include "ear/bs2051.hpp"
#include "ear/ear.hpp"
#include "ear/metadata.hpp"

using namespace ear;

using Gains = std::vector<float>;

ObjectsTypeMetadata otmWithPos(Position p) {
  ObjectsTypeMetadata otm;
  otm.position = p;
  return otm;
}

using GainsMap = std::map<std::string, double>;
struct GainsMaps {
  GainsMap direct;
  GainsMap diffuse;
};

GainsMap mapGainsToChannelNames(std::vector<std::string> channelNames,
                                Gains gains) {
  assert(channelNames.size() == gains.size());
  GainsMap ret;
  std::transform(channelNames.begin(), channelNames.end(), gains.begin(),
                 std::inserter(ret, ret.end()),
                 std::make_pair<std::string const&, double const&>);
  return ret;
}

class GainCalculatorObjectsTester {
 public:
  GainCalculatorObjectsTester(Layout layout)
      : _layout(layout),
        _gainCalc(boost::make_unique<GainCalculatorObjects>(layout)){};

  GainsMaps run(const ObjectsTypeMetadata& metadata) {
    Gains directGains(_layout.channels().size(), 0.0);
    Gains diffuseGains(_layout.channels().size(), 0.0);
    _gainCalc->calculate(metadata, directGains, diffuseGains);
    auto channelNames = _layout.channelNames();
    auto directGainsMap = mapGainsToChannelNames(channelNames, directGains);
    auto diffuseGainsMap = mapGainsToChannelNames(channelNames, diffuseGains);
    double epsilon = 1e-6;
    for (auto it = directGainsMap.begin(); it != directGainsMap.end();) {
      if (std::abs(it->second) < epsilon) {
        it = directGainsMap.erase(it);
      } else {
        ++it;
      }
    }
    for (auto it = diffuseGainsMap.begin(); it != diffuseGainsMap.end();) {
      if (std::abs(it->second) < epsilon) {
        it = diffuseGainsMap.erase(it);
      } else {
        ++it;
      }
    }
    return GainsMaps{directGainsMap, diffuseGainsMap};
  };

 private:
  Layout _layout;
  std::unique_ptr<GainCalculatorObjects> _gainCalc;
};

TEST_CASE("gain_calculator") {
  auto layout = getLayout("4+7+0").withoutLfe();
  GainCalculatorObjectsTester gainCalc(layout);

  SECTION("basic_centre") {
    auto gainMap = gainCalc.run(otmWithPos(PolarPosition{0.0, 0.0, 1.0}));
    REQUIRE(gainMap.direct.size() == 1);
    REQUIRE(gainMap.direct.at("M+000") == Approx(1.0));
    REQUIRE(gainMap.diffuse.size() == 0);
  }
  SECTION("basic_left") {
    auto gainMap = gainCalc.run(otmWithPos(PolarPosition{30.0, 0.0, 1.0}));
    REQUIRE(gainMap.direct.size() == 1);
    REQUIRE(gainMap.direct.at("M+030") == Approx(1.0));
    REQUIRE(gainMap.diffuse.size() == 0);
  }
  SECTION("basic_left_up") {
    auto gainMap = gainCalc.run(otmWithPos(PolarPosition{45.0, 30.0, 1.0}));
    REQUIRE(gainMap.direct.size() == 1);
    REQUIRE(gainMap.direct.at("U+045") == Approx(1.0));
    REQUIRE(gainMap.diffuse.size() == 0);
  }
}

TEST_CASE("diffuse") {
  auto layout = getLayout("4+7+0").withoutLfe();
  GainCalculatorObjectsTester gainCalc(layout);

  SECTION("half") {
    auto otm = otmWithPos(PolarPosition{0.0, 0.0, 1.0});
    otm.diffuse = 0.5;
    auto gainMap = gainCalc.run(otm);
    REQUIRE(gainMap.direct.size() == 1);
    REQUIRE(gainMap.direct.at("M+000") == Approx(std::sqrt(0.5)));
    REQUIRE(gainMap.diffuse.size() == 1);
    REQUIRE(gainMap.diffuse.at("M+000") == Approx(std::sqrt(0.5)));
  }

  SECTION("full") {
    auto otm = otmWithPos(PolarPosition{0.0, 0.0, 1.0});
    otm.diffuse = 1.0;
    auto gainMap = gainCalc.run(otm);
    REQUIRE(gainMap.direct.size() == 0);
    REQUIRE(gainMap.diffuse.size() == 1);
    REQUIRE(gainMap.diffuse.at("M+000") == Approx(1.0));
  }
}

TEST_CASE("gain_value") {
  auto layout = getLayout("4+7+0").withoutLfe();
  GainCalculatorObjectsTester gainCalc(layout);

  auto otm = otmWithPos(PolarPosition{0.0, 0.0, 1.0});
  otm.gain = 0.5;
  auto gainMap = gainCalc.run(otm);
  REQUIRE(gainMap.direct.size() == 1);
  REQUIRE(gainMap.direct.at("M+000") == Approx(0.5));
  REQUIRE(gainMap.diffuse.size() == 0);
}

#ifndef EAR_NO_EXCEPTIONS

TEST_CASE("not implemented") {
  auto layout = getLayout("4+7+0").withoutLfe();
  GainCalculatorObjectsTester gainCalc(layout);

  auto otm = otmWithPos(PolarPosition{0.0, 0.0, 1.0});

  SECTION("cartesian") {
    otm.cartesian = true;
    REQUIRE_THROWS_AS(gainCalc.run(otm), not_implemented);
  }
  SECTION("Cartesian position") {
    otm.position = CartesianPosition(0.0, 0.0, 0.0);
    REQUIRE_THROWS_AS(gainCalc.run(otm), not_implemented);
  }
  SECTION("objectDivergence polar") {
    otm.objectDivergence = PolarObjectDivergence(0.5);
    REQUIRE_THROWS_AS(gainCalc.run(otm), not_implemented);
  }
  SECTION("objectDivergence Cartesian") {
    otm.objectDivergence = CartesianObjectDivergence(0.5);
    REQUIRE_THROWS_AS(gainCalc.run(otm), not_implemented);
  }
  SECTION("channelLock") {
    otm.channelLock.flag = true;
    REQUIRE_THROWS_AS(gainCalc.run(otm), not_implemented);
  }
  SECTION("zone") {
    otm.zoneExclusion.zones.push_back(
        PolarExclusionZone{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ""});
    REQUIRE_THROWS_AS(gainCalc.run(otm), not_implemented);
  }
  SECTION("screenRef") {
    otm.screenRef = true;
    REQUIRE_THROWS_AS(gainCalc.run(otm), not_implemented);
  }
}

#endif
