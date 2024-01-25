#include <Eigen/Core>
#include <boost/make_unique.hpp>
#include <catch2/catch.hpp>
#include <map>
#include <string>
#include <vector>
#include "common/geom.hpp"
#include "ear/bs2051.hpp"
#include "ear/ear.hpp"
#include "ear/metadata.hpp"
#include "helper/vector_approx.hpp"

using namespace ear;

using Gains = std::vector<double>;

DirectSpeakersTypeMetadata tmWithLabels(std::vector<std::string> labels) {
  DirectSpeakersTypeMetadata tm;
  tm.speakerLabels = labels;
  return tm;
}

Gains directPv(const Layout& layout, const std::string& channel) {
  Eigen::VectorXd pv = Eigen::VectorXd::Zero(layout.channels().size());
  auto channels = layout.channelNames();
  auto it = std::find(channels.begin(), channels.end(), channel);
  pv(std::distance(channels.begin(), it)) = 1.0;
  return Gains(pv.data(), pv.data() + pv.rows() * pv.cols());
}

TEST_CASE("test_speaker_label") {
  Layout layout = getLayout("4+5+0");
  GainCalculatorDirectSpeakers p(layout);

  Gains actual(layout.channels().size());
  std::vector<std::string> prefixes = {
      "", "urn:itu:bs:2051:0:speaker:", "urn:itu:bs:2051:1:speaker:"};
  for (const std::string& prefix : prefixes) {
    // normal case
    p.calculate(tmWithLabels(std::vector<std::string>{prefix + "M+000"}),
                actual);
    REQUIRE_VECTOR_APPROX(actual, directPv(layout, "M+000"));
    p.calculate(tmWithLabels(std::vector<std::string>{prefix + "M+030"}),
                actual);
    REQUIRE_VECTOR_APPROX(actual, directPv(layout, "M+030"));

    // missing channels are ignored
    p.calculate(tmWithLabels(std::vector<std::string>{prefix + "M+030",
                                                      prefix + "B+000"}),
                actual);
    REQUIRE_VECTOR_APPROX(actual, directPv(layout, "M+030"));
    p.calculate(tmWithLabels(std::vector<std::string>{prefix + "B+000",
                                                      prefix + "M+030"}),
                actual);
    REQUIRE_VECTOR_APPROX(actual, directPv(layout, "M+030"));

    // matching more than one channel should pick the first
    p.calculate(tmWithLabels(std::vector<std::string>{prefix + "M+000",
                                                      prefix + "M+030"}),
                actual);
    REQUIRE_VECTOR_APPROX(actual, directPv(layout, "M+000"));
    p.calculate(tmWithLabels(std::vector<std::string>{prefix + "M+030",
                                                      prefix + "M+000"}),
                actual);
    REQUIRE_VECTOR_APPROX(actual, directPv(layout, "M+030"));
  }
}

TEST_CASE("test_speaker_label_additional_substitutions") {
  Layout layout = getLayout("4+5+0");
  GainCalculatorDirectSpeakers p(layout, {{"foo", "M+030"}});

  Gains actual(layout.channels().size());
  p.calculate(tmWithLabels(std::vector<std::string>{"foo"}), actual);
  REQUIRE_VECTOR_APPROX(actual, directPv(layout, "M+030"));
}

TEST_CASE("test_one_lfe_out") {
  // When there is only one LFE output, both LFE1 and LFE2 should be sent to it.
  Layout layout = getLayout("4+5+0");
  GainCalculatorDirectSpeakers p(layout);

  Gains actual(layout.channels().size());
  p.calculate(tmWithLabels(std::vector<std::string>{"LFE1"}), actual);
  REQUIRE_VECTOR_APPROX(actual, directPv(layout, "LFE1"));
  p.calculate(tmWithLabels(std::vector<std::string>{"LFE2"}), actual);
  REQUIRE_VECTOR_APPROX(actual, directPv(layout, "LFE1"));
}

TEST_CASE("test_no_lfe_out") {
  // When there is no LFE output, both LFE1 and LFE2 should be discarded.
  Layout layout = getLayout("0+2+0");
  GainCalculatorDirectSpeakers p(layout);

  Gains actual(layout.channels().size());
  p.calculate(tmWithLabels(std::vector<std::string>{"LFE1"}), actual);
  REQUIRE_VECTOR_APPROX(actual, Gains(layout.channels().size(), 0.0));
  p.calculate(tmWithLabels(std::vector<std::string>{"LFE2"}), actual);
  REQUIRE_VECTOR_APPROX(actual, Gains(layout.channels().size(), 0.0));
}

TEST_CASE("test_lfe_just_frequency") {
  // When there is only one LFE output, both LFE1 and LFE2 should be sent to it.
  Layout layout = getLayout("4+5+0");
  GainCalculatorDirectSpeakers p(layout);

  DirectSpeakersTypeMetadata tm;

  SECTION("just frequency") {
    tm.channelFrequency.lowPass = 100.0;
    Gains actual(layout.channels().size());
    p.calculate(tm, actual);
    REQUIRE_VECTOR_APPROX(actual, directPv(layout, "LFE1"));
  }

  SECTION("frequency and speakerLabel") {
    tm.channelFrequency.lowPass = 100.0;
    tm.speakerLabels = {"LFE1"};
    Gains actual(layout.channels().size());
    p.calculate(tm, actual);
    REQUIRE_VECTOR_APPROX(actual, directPv(layout, "LFE1"));
  }
}

TEST_CASE("test_dist_bounds_polar") {
  Layout layout = getLayout("9+10+3");
  GainCalculatorDirectSpeakers p(layout);

  DirectSpeakersTypeMetadata tm;
  PolarSpeakerPosition pos;

  Gains actual(layout.channels().size(), 0.0);
  Gains expected(layout.channels().size(), 0.0);

  // test horizontal bounds
  std::fill(expected.begin(), expected.end(), 0.0);
  expected[layout.indexForName("M+000").get()] = std::sqrt(0.5);
  expected[layout.indexForName("M+030").get()] = std::sqrt(0.5);

  // no bounds, and position not on speaker -> use psp
  pos = PolarSpeakerPosition(15.0, 0.0, 1.0);
  tm.position = pos;
  p.calculate(tm, actual);
  REQUIRE_VECTOR_APPROX(actual, expected);

  // lower bound expanded to speaker -> direct on that speaker
  pos = PolarSpeakerPosition(15.0, 0.0, 1.0);
  pos.azimuthMin = 0.0;
  tm.position = pos;
  p.calculate(tm, actual);
  REQUIRE_VECTOR_APPROX(actual, directPv(layout, "M+000"));

  // upper bound expanded to speaker -> direct on that speaker
  pos = PolarSpeakerPosition(15.0, 0.0, 1.0);
  pos.azimuthMax = 30.0;
  tm.position = pos;
  p.calculate(tm, actual);
  REQUIRE_VECTOR_APPROX(actual, directPv(layout, "M+030"));

  // upper and lower bounds expanded to speakers -> use psp again
  pos = PolarSpeakerPosition(15.0, 0.0, 1.0);
  pos.azimuthMin = 0.0;
  pos.azimuthMax = 30.0;
  tm.position = pos;
  p.calculate(tm, actual);
  REQUIRE_VECTOR_APPROX(actual, expected);

  // closer to one than the other -> direct to that speaker
  pos = PolarSpeakerPosition(14.0, 0.0, 1.0);
  pos.azimuthMin = 0.0;
  pos.azimuthMax = 30.0;
  tm.position = pos;
  p.calculate(tm, actual);
  REQUIRE_VECTOR_APPROX(actual, directPv(layout, "M+000"));

  // test vertical bounds
  std::fill(expected.begin(), expected.end(), 0.0);
  expected[layout.indexForName("M+000").get()] = std::sqrt(0.5);
  expected[layout.indexForName("U+000").get()] = std::sqrt(0.5);

  // // no bounds, and position not on speaker -> use psp
  pos = PolarSpeakerPosition(0.0, 15.0, 1.0);
  tm.position = pos;
  p.calculate(tm, actual);
  REQUIRE_VECTOR_APPROX(actual, expected);

  // lower bound expanded to speaker -> direct on that speaker
  pos = PolarSpeakerPosition(0.0, 15.0, 1.0);
  pos.elevationMin = 0.0;
  tm.position = pos;
  p.calculate(tm, actual);
  REQUIRE_VECTOR_APPROX(actual, directPv(layout, "M+000"));

  // upper bound expanded to speaker -> direct on that speaker
  pos = PolarSpeakerPosition(0.0, 15.0, 1.0);
  pos.elevationMax = 30.0;
  tm.position = pos;
  p.calculate(tm, actual);
  REQUIRE_VECTOR_APPROX(actual, directPv(layout, "U+000"));

  // upper and lower bounds expanded to speakers -> use psp again
  pos = PolarSpeakerPosition(0.0, 15.0, 1.0);
  pos.elevationMin = 0.0;
  pos.elevationMax = 30.0;
  tm.position = pos;
  p.calculate(tm, actual);
  REQUIRE_VECTOR_APPROX(actual, expected);

  // closer to one than the other -> direct to that speaker
  pos = PolarSpeakerPosition(0.0, 14.0, 1.0);
  pos.elevationMin = 0.0;
  pos.elevationMax = 30.0;
  tm.position = pos;
  p.calculate(tm, actual);
  REQUIRE_VECTOR_APPROX(actual, directPv(layout, "M+000"));

  // check that pole speakers are used even if 0 is excluded by azimuth range
  pos = PolarSpeakerPosition(15.0, 90.0, 1.0);
  pos.azimuthMin = 10.0;
  pos.azimuthMax = 20.0;
  tm.position = pos;
  p.calculate(tm, actual);
  REQUIRE_VECTOR_APPROX(actual, directPv(layout, "T+000"));
}

TEST_CASE("test_dist_bounds_cart", "[!shouldfail]") {
  Layout layout = getLayout("9+10+3");
  GainCalculatorDirectSpeakers p(layout);

  DirectSpeakersTypeMetadata tm;
  CartesianSpeakerPosition pos;

  Gains actual(layout.channels().size(), 0.0);
  Gains expected(layout.channels().size(), 0.0);

  // on speaker -> direct
  pos = CartesianSpeakerPosition(1.0, 0.0, 0.0);
  tm.position = pos;
  p.calculate(tm, actual);
  REQUIRE_VECTOR_APPROX(actual, directPv(layout, "M-090"));

  // lower bound on speaker -> direct
  pos = CartesianSpeakerPosition(1.0, 0.1, 0.0);
  pos.YMin = 0.0;
  tm.position = pos;
  p.calculate(tm, actual);
  REQUIRE_VECTOR_APPROX(actual, directPv(layout, "M-090"));

  // upper bound on speaker -> direct
  pos = CartesianSpeakerPosition(1.0, -0.1, 0.0);
  pos.YMax = 0.0;
  tm.position = pos;
  p.calculate(tm, actual);
  REQUIRE_VECTOR_APPROX(actual, directPv(layout, "M-090"));

  // pick closest within bound
  Eigen::Vector3d pos30 = cart(30.0, 0.0, 1.0);
  Eigen::Vector3d pos5 = cart(5.0, 0.0, 1.0);
  Eigen::Vector3d pos25 = cart(25.0, 0.0, 1.0);

  pos = CartesianSpeakerPosition(pos5(0), pos5(1), 0.0);
  pos.XMin = pos30(0);
  pos.XMax = -pos30(0);
  pos.YMin = pos30(1);
  pos.YMax = 1.0;
  tm.position = pos;
  p.calculate(tm, actual);
  REQUIRE_VECTOR_APPROX(actual, directPv(layout, "M+000"));

  pos = CartesianSpeakerPosition(pos25(0), pos25(1), 0.0);
  pos.XMin = pos30(0);
  pos.XMax = -pos30(0);
  pos.YMin = pos30(1);
  pos.YMax = 1.0;
  tm.position = pos;
  p.calculate(tm, actual);
  REQUIRE_VECTOR_APPROX(actual, directPv(layout, "M+030"));
}

TEST_CASE("mapping") {
  auto layout = getLayout("4+5+0").withoutLfe();
  GainCalculatorDirectSpeakers p(layout);
  Gains actual(layout.channels().size(), 0.0);

  DirectSpeakersTypeMetadata tm;
  tm.audioPackFormatID = "AP_0001000f";

  SECTION("AC_0001001c") {
    tm.position = PolarSpeakerPosition(135.0, 0.0, 1.0);
    tm.speakerLabels = {"urn:itu:bs:2051:0:speaker:M+135"};

    p.calculate(tm, actual);
    REQUIRE_VECTOR_APPROX(actual, directPv(layout, "M+110"));
  }

  SECTION("AC_00010011") {
    tm.position = PolarSpeakerPosition(180.0, 30.0, 1.0);
    tm.speakerLabels = {"urn:itu:bs:2051:0:speaker:U+180"};
    p.calculate(tm, actual);

    Gains expected(layout.channels().size(), 0.0);
    expected[layout.indexForName("U+110").get()] = std::sqrt(0.5);
    expected[layout.indexForName("U-110").get()] = std::sqrt(0.5);

    REQUIRE_VECTOR_APPROX(actual, expected);
  }
}

TEST_CASE("mapping_per_input") {
  auto layout = getLayout("4+5+0").withoutLfe();
  GainCalculatorDirectSpeakers p(layout);
  Gains actual(layout.channels().size(), 0.0);

  DirectSpeakersTypeMetadata tm;
  tm.speakerLabels = {"urn:itu:bs:2051:0:speaker:M+090"};
  tm.position = PolarSpeakerPosition(90.0, 0.0, 1.0);

  SECTION("AP_00010009") {
    tm.audioPackFormatID = "AP_00010009";
    p.calculate(tm, actual);

    Gains expected(layout.channels().size(), 0.0);
    expected[layout.indexForName("M+030").get()] = std::sqrt(1.0 / 3.0);
    expected[layout.indexForName("M+110").get()] = std::sqrt(2.0 / 3.0);

    REQUIRE_VECTOR_APPROX(actual, expected);
  }

  SECTION("AP_00010017") {
    tm.audioPackFormatID = "AP_00010017";
    p.calculate(tm, actual);

    Gains expected(layout.channels().size(), 0.0);
    expected[layout.indexForName("M+030").get()] = std::sqrt(0.5);
    expected[layout.indexForName("M+110").get()] = std::sqrt(0.5);

    REQUIRE_VECTOR_APPROX(actual, expected);
  }
}

TEST_CASE("not implemented") {
  auto layout = getLayout("4+7+0").withoutLfe();
  GainCalculatorDirectSpeakers p(layout);
  Gains gains(layout.channels().size(), 0.0);

  DirectSpeakersTypeMetadata tm;

  SECTION("screenEdgeLock horizontal") {
    PolarSpeakerPosition pos;
    pos.screenEdgeLock.horizontal = "left";
    tm.position = pos;
    REQUIRE_THROWS_AS(p.calculate(tm, gains), not_implemented);
  }

  SECTION("screenEdgeLock vertical") {
    PolarSpeakerPosition pos;
    pos.screenEdgeLock.vertical = "top";
    tm.position = pos;
    REQUIRE_THROWS_AS(p.calculate(tm, gains), not_implemented);
  }

  SECTION("cartesian positions") {
    tm.position = CartesianSpeakerPosition();
    REQUIRE_THROWS_AS(p.calculate(tm, gains), not_implemented);
  }
}

TEST_CASE("adm errors") {
  auto layout = getLayout("4+7+0").withoutLfe();
  GainCalculatorDirectSpeakers p(layout);
  Gains gains(layout.channels().size(), 0.0);

  DirectSpeakersTypeMetadata tm;

  SECTION("audioPackFormatID without speakerLabels") {
    tm.audioPackFormatID = "AP_00010002";
    REQUIRE_THROWS_AS(p.calculate(tm, gains), adm_error);
  }
}

TEST_CASE("warnings") {
  auto layout = getLayout("4+7+0").withoutLfe();
  GainCalculatorDirectSpeakers p(layout);
  Gains gains(layout.channels().size(), 0.0);

  DirectSpeakersTypeMetadata tm;

  SECTION("non-LFE frequency") {
    tm.channelFrequency.lowPass = 300.0;
    std::vector<Warning> warnings;
    p.calculate(tm, gains, [&](const Warning& w) { warnings.push_back(w); });
    REQUIRE(warnings.size() == 1);
    REQUIRE(warnings[0].code == Warning::Code::FREQ_NOT_LFE);
  }

  SECTION("LFE mismatch") {
    tm.channelFrequency.lowPass = 100.0;
    tm.speakerLabels = {"M+000"};
    std::vector<Warning> warnings;
    p.calculate(tm, gains, [&](const Warning& w) { warnings.push_back(w); });
    REQUIRE(warnings.size() == 1);
    REQUIRE(warnings[0].code == Warning::Code::FREQ_SPEAKERLABEL_LFE_MISMATCH);
  }
}

// TODO: implement screenEdgeLock tests
