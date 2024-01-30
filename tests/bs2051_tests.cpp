#include <boost/algorithm/string.hpp>
#include <boost/optional/optional_io.hpp>
#include <catch2/catch.hpp>
#include <map>
#include <numeric>
#include <string>
#include <vector>
#include "common/geom.hpp"
#include "ear/bs2051.hpp"
#include "helper/compare.hpp"

using namespace ear;

TEST_CASE("layout") {
  auto layout = getLayout("4+5+0");
  REQUIRE(layout.name() == "4+5+0");
  REQUIRE(layout.channelNames()[0] == "M+030");
  REQUIRE(layout.channelNames()[1] == "M-030");
  REQUIRE(layout.channels().size() == 10);
  REQUIRE(layout.channels()[0].polarPosition() ==
          PolarPosition{30.0, 0.0, 1.0});
  REQUIRE(layout.channels()[1].polarPosition() ==
          PolarPosition{-30.0, 0.0, 1.0});
}

#ifndef EAR_NO_EXCEPTIONS
TEST_CASE("unkown_layout") { REQUIRE_THROWS(getLayout("wat")); }
#endif

TEST_CASE("all_positions_in_range") {
  std::vector<std::string> errors;
  for (const auto& layout : loadLayouts()) {
    SECTION(layout.name()) {
      errors.clear();
      layout.checkPositions(
          [&errors](const std::string& msg) { errors.push_back(msg); });
      for (const auto& error : errors) {
        WARN(error);
      }
      REQUIRE(errors.size() == 0);
    }
  }
}

/*
Test that most ranges are reasonably small; this detects ranges that
have been inverted. Screen speakers and LFE channels are ignored.
*/
TEST_CASE("azimuth_ranges") {
  for (const auto& layout : loadLayouts()) {
    for (const auto& channel : layout.channels()) {
      if (!channel.isLfe() && channel.name().find("SC") == std::string::npos) {
        auto azRange = channel.azimuthRange();
        auto range_size =
            relativeAngle(azRange.first, azRange.second) - azRange.first;
        REQUIRE(range_size <= 180.0);
      }
    }
  }
}

TEST_CASE("test_symmetry") {
  for (const auto& layout : loadLayouts()) {
    // find symmetric pairs of speakers
    std::map<std::string, std::vector<Channel>> symmetricPairs;
    for (auto& channel : layout.channels()) {
      std::vector<std::string> channelNameVec;
      std::string name = channel.name();
      boost::split(channelNameVec, name, boost::is_any_of("+-"));
      if (channelNameVec.size() > 1) {
        std::string key;
        key = std::accumulate(begin(channelNameVec), end(channelNameVec), key);
        symmetricPairs[key].push_back(channel);
      } else {
        REQUIRE(channel.name().find("LFE") != std::string::npos);
        REQUIRE(channel.isLfe());
      }
    }
    for (const auto& entry : symmetricPairs) {
      auto key = entry.first;
      auto pair = entry.second;
      if (pair.size() == 1) {
        // any non-paired speakers should be on the centre line
        std::vector<double> values = {0.0, -180.0, 180.0};
        REQUIRE(std::find(values.begin(), values.end(),
                          pair[0].polarPosition().azimuth) != values.end());
      } else if (pair.size() == 2) {
        // all pairs should be have symmetrical positions and ranges
        Channel a = pair[0];
        Channel b = pair[1];

        REQUIRE(a.polarPosition().elevation == b.polarPosition().elevation);
        REQUIRE(a.polarPosition().azimuth == -b.polarPosition().azimuth);
        REQUIRE(a.elevationRange() == b.elevationRange());
        REQUIRE(a.azimuthRange() == std::make_pair(-b.azimuthRange().second,
                                                   -b.azimuthRange().first));
      } else {
        REQUIRE(false);
      }
    }
  }
}
