#include <Eigen/Core>
#include <Eigen/StdVector>
#include <boost/make_unique.hpp>
#include <boost/optional/optional_io.hpp>
#include <catch2/catch.hpp>
#include <random>
#include <string>
#include "common/convex_hull.hpp"
#include "common/facets.hpp"
#include "common/geom.hpp"
#include "common/point_source_panner.hpp"
#include "ear/bs2051.hpp"
#include "ear/exceptions.hpp"
#include "helper/compare.hpp"

using namespace ear;

TEST_CASE("test_virtual_ngon") {
  Eigen::MatrixXd spkPositions(4, 3);
  spkPositions << cartT(30.0, 0.0, 1.0), cartT(-30.0, 0.0, 1.0),
      cartT(30.0, 30.0, 1.0), cartT(-30.0, 30.0, 1.0);
  Eigen::Vector4d virtualDownmix(0.2, 0.2, 0.3, 0.3);
  Eigen::Vector3d virtualPos = virtualDownmix.transpose() * spkPositions;
  auto ng =
      std::make_shared<VirtualNgon>(Eigen::Vector4i::LinSpaced(4, 0, 4),
                                    spkPositions, virtualPos, virtualDownmix);
  // panning to the virtual speaker, the output should be the normalised downmix
  boost::optional<Eigen::VectorXd> pv = ng->handle(virtualPos);
  REQUIRE(pv != boost::none);
  REQUIRE(pv.get().size() == 4);
  REQUIRE(pv->isApprox(virtualDownmix / virtualDownmix.norm()));

  std::default_random_engine generator;
  std::uniform_real_distribution<double> distribution(0.0, 1.0);
  auto random = [&](double) { return distribution(generator); };
  for (int i = 0; i < 100; ++i) {
    Eigen::Vector4d proportion = Eigen::Vector4d::NullaryExpr(4, random);
    // we can calculate a position within the ngon
    Eigen::Vector3d pos = spkPositions.transpose() * proportion;
    pos /= pos.norm();
    // which when rendered
    boost::optional<Eigen::VectorXd> pv = ng->handle(pos);
    REQUIRE(pv != boost::none);
    // can be multiplied by the speaker positions to produce another position
    Eigen::Vector3d posCalc = pv->transpose() * spkPositions;
    posCalc /= posCalc.norm();
    // that should be the same as the one we started with
    REQUIRE(pos.isApprox(posCalc));
  }
}

TEST_CASE("test_quad") {
  Eigen::MatrixXd spkPositions(4, 3);
  spkPositions << cartT(30.0, -15.0, 1.0), cartT(-30.0, -15.0, 1.0),
      cartT(30.0, 15.0, 1.0), cartT(-30.0, 15.0, 1.0);
  auto quad = std::make_shared<QuadRegion>(Eigen::Vector4i::LinSpaced(4, 0, 4),
                                           spkPositions);

  using PosGainsT = std::pair<Eigen::Vector3d, Eigen::Vector4d>;
  std::vector<PosGainsT, Eigen::aligned_allocator<PosGainsT>> posGains;
  posGains.push_back(
      std::make_pair(spkPositions.row(0), Eigen::Vector4d(1.0, 0.0, 0.0, 0.0)));
  posGains.push_back(
      std::make_pair(spkPositions.row(1), Eigen::Vector4d(0.0, 1.0, 0.0, 0.0)));
  posGains.push_back(
      std::make_pair(spkPositions.row(2), Eigen::Vector4d(0.0, 0.0, 1.0, 0.0)));
  posGains.push_back(
      std::make_pair(spkPositions.row(3), Eigen::Vector4d(0.0, 0.0, 0.0, 1.0)));
  posGains.push_back(
      std::make_pair(cart(0.0, 0.0, 1.0), Eigen::Vector4d(0.5, 0.5, 0.5, 0.5)));
  for (const auto& pair : posGains) {
    auto pv = quad->handle(pair.first);
    REQUIRE(pv != boost::none);
    REQUIRE(pv->isApprox(pair.second));
  }
};

TEST_CASE("test_stereo_downmix") {
  Eigen::MatrixXd spkPositions(2, 3);
  spkPositions << cartT(30.0, 0.0, 1.0), cartT(-30.0, 0.0, 1.0);

  auto p = std::make_shared<StereoPannerDownmix>(
      Eigen::Vector2i::LinSpaced(2, 0, 1), spkPositions);

  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector2d>> posGains;
  posGains.push_back(std::make_pair(cart(0.0, 0.0, 1.0),
                                    Eigen::Vector2d(sqrt(0.5), sqrt(0.5))));
  posGains.push_back(
      std::make_pair(cart(-30.0, 0.0, 1.0), Eigen::Vector2d(0.0, 1.0)));
  posGains.push_back(
      std::make_pair(cart(-110.0, 0.0, 1.0), Eigen::Vector2d(0.0, sqrt(0.5))));
  posGains.push_back(std::make_pair(cart(-180.0, 0.0, 1.0),
                                    Eigen::Vector2d(sqrt(0.25), sqrt(0.25))));

  for (auto pair : posGains) {
    auto pv = p->handle(pair.first);
    REQUIRE(pv->isApprox(pair.second));
  }
}

Eigen::MatrixXd createDownmixMatrixFromIndices(std::vector<int> indices,
                                               size_t size) {
  Eigen::MatrixXd downmix = Eigen::MatrixXd::Identity(size, size);
  for (int index : indices) {
    Eigen::VectorXd downmixRow = Eigen::VectorXd::Zero(size);
    downmixRow(index) = 1.0;
    downmix.conservativeResize(downmix.rows() + 1, Eigen::NoChange);
    downmix.row(downmix.rows() - 1) = downmixRow;
  }
  return downmix;
}

TEST_CASE("extra_pos_vertical_nominal") {
  std::vector<Channel> extraChannels;
  Eigen::MatrixXd downmix;
  std::vector<PolarPosition> expectedPositions;

  SECTION("0+5+0") {
    Layout layout = getLayout("0+5+0").withoutLfe();
    std::tie(extraChannels, downmix) = extraPosVerticalNominal(layout);
    std::vector<PolarPosition> expectedPositions = {
        PolarPosition(30.0, -30.0, 1.0),   PolarPosition(-30.0, -30.0, 1.0),
        PolarPosition(0.0, -30.0, 1.0),    PolarPosition(110.0, -30.0, 1.0),
        PolarPosition(-110.0, -30.0, 1.0), PolarPosition(30.0, 30.0, 1.0),
        PolarPosition(-30.0, 30.0, 1.0),   PolarPosition(0.0, 30.0, 1.0),
        PolarPosition(110.0, 30.0, 1.0),   PolarPosition(-110.0, 30.0, 1.0)};
    REQUIRE(extraChannels.size() == expectedPositions.size());
    for (size_t i = 0; i < extraChannels.size(); ++i) {
      REQUIRE(extraChannels[i].polarPosition() == expectedPositions[i]);
    }
    Eigen::MatrixXd expectedDownmix = createDownmixMatrixFromIndices(
        std::vector<int>{0, 1, 2, 3, 4, 0, 1, 2, 3, 4},
        layout.channels().size());
    REQUIRE(downmix == expectedDownmix);
  };

  SECTION("2+5+0") {
    Layout layout = getLayout("2+5+0").withoutLfe();
    std::tie(extraChannels, downmix) = extraPosVerticalNominal(layout);
    std::vector<PolarPosition> expectedPositions = {
        PolarPosition(30.0, -30.0, 1.0),   PolarPosition(-30.0, -30.0, 1.0),
        PolarPosition(0.0, -30.0, 1.0),    PolarPosition(110.0, -30.0, 1.0),
        PolarPosition(-110.0, -30.0, 1.0), PolarPosition(110.0, 30.0, 1.0),
        PolarPosition(-110.0, 30.0, 1.0)};
    REQUIRE(extraChannels.size() == expectedPositions.size());
    for (size_t i = 0; i < extraChannels.size(); ++i) {
      REQUIRE(extraChannels[i].polarPosition() == expectedPositions[i]);
    }
    Eigen::MatrixXd expectedDownmix = createDownmixMatrixFromIndices(
        std::vector<int>{0, 1, 2, 3, 4, 3, 4}, layout.channels().size());
    REQUIRE(downmix == expectedDownmix);
  };

  SECTION("4+5+0/4+5+1") {
    for (const std::string& layoutName : {"4+5+0", "4+5+1"}) {
      Layout layout = getLayout("4+5+0").withoutLfe();
      std::tie(extraChannels, downmix) = extraPosVerticalNominal(layout);
      std::vector<PolarPosition> expectedPositions = {
          PolarPosition(30.0, -30.0, 1.0), PolarPosition(-30.0, -30.0, 1.0),
          PolarPosition(0.0, -30.0, 1.0), PolarPosition(110.0, -30.0, 1.0),
          PolarPosition(-110.0, -30.0, 1.0)};
      REQUIRE(extraChannels.size() == expectedPositions.size());
      for (size_t i = 0; i < extraChannels.size(); ++i) {
        REQUIRE(extraChannels[i].polarPosition() == expectedPositions[i]);
      }
      Eigen::MatrixXd expectedDownmix = createDownmixMatrixFromIndices(
          std::vector<int>{0, 1, 2, 3, 4}, layout.channels().size());
      REQUIRE(downmix == expectedDownmix);
    }
  };

  SECTION("3+7+0") {
    Layout layout = getLayout("3+7+0").withoutLfe();
    std::tie(extraChannels, downmix) = extraPosVerticalNominal(layout);
    std::vector<PolarPosition> expectedPositions = {
        PolarPosition(0.0, -30.0, 1.0),   PolarPosition(30.0, -30.0, 1.0),
        PolarPosition(-30.0, -30.0, 1.0), PolarPosition(90.0, -30.0, 1.0),
        PolarPosition(-90.0, -30.0, 1.0), PolarPosition(135.0, -30.0, 1.0),
        PolarPosition(-135.0, -30.0, 1.0)};
    REQUIRE(extraChannels.size() == expectedPositions.size());
    for (size_t i = 0; i < extraChannels.size(); ++i) {
      REQUIRE(extraChannels[i].polarPosition() == expectedPositions[i]);
    }
    Eigen::MatrixXd expectedDownmix = createDownmixMatrixFromIndices(
        std::vector<int>{0, 1, 2, 5, 6, 7, 8}, layout.channels().size());
    REQUIRE(downmix == expectedDownmix);
  };

  SECTION("4+9+0") {
    Layout layout = getLayout("4+9+0").withoutLfe();
    std::tie(extraChannels, downmix) = extraPosVerticalNominal(layout);
    std::vector<PolarPosition> expectedPositions = {
        PolarPosition(30.0, -30.0, 1.0),   PolarPosition(-30.0, -30.0, 1.0),
        PolarPosition(0.0, -30.0, 1.0),    PolarPosition(90.0, -30.0, 1.0),
        PolarPosition(-90.0, -30.0, 1.0),  PolarPosition(135.0, -30.0, 1.0),
        PolarPosition(-135.0, -30.0, 1.0), PolarPosition(15.0, -30.0, 1.0),
        PolarPosition(-15.0, -30.0, 1.0)};
    REQUIRE(extraChannels.size() == expectedPositions.size());
    for (size_t i = 0; i < extraChannels.size(); ++i) {
      REQUIRE(extraChannels[i].polarPosition() == expectedPositions[i]);
    }
    Eigen::MatrixXd expectedDownmix = createDownmixMatrixFromIndices(
        std::vector<int>{0, 1, 2, 3, 4, 5, 6, 11, 12},
        layout.channels().size());
    REQUIRE(downmix == expectedDownmix);
  };

  SECTION("9+10+3") {
    Layout layout = getLayout("9+10+3").withoutLfe();
    std::tie(extraChannels, downmix) = extraPosVerticalNominal(layout);
    std::vector<PolarPosition> expectedPositions = {
        PolarPosition(135.0, -30.0, 1.0), PolarPosition(-135.0, -30.0, 1.0),
        PolarPosition(180.0, -30.0, 1.0), PolarPosition(90.0, -30.0, 1.0),
        PolarPosition(-90.0, -30.0, 1.0)};
    REQUIRE(extraChannels.size() == expectedPositions.size());
    for (size_t i = 0; i < extraChannels.size(); ++i) {
      REQUIRE(extraChannels[i].polarPosition() == expectedPositions[i]);
    }
    Eigen::MatrixXd expectedDownmix = createDownmixMatrixFromIndices(
        std::vector<int>{3, 4, 7, 8, 9}, layout.channels().size());
    REQUIRE(downmix == expectedDownmix);
  };

  SECTION("0+7+0") {
    Layout layout = getLayout("0+7+0").withoutLfe();
    std::tie(extraChannels, downmix) = extraPosVerticalNominal(layout);
    std::vector<PolarPosition> expectedPositions = {
        PolarPosition(30.0, -30.0, 1.0),   PolarPosition(-30.0, -30.0, 1.0),
        PolarPosition(0.0, -30.0, 1.0),    PolarPosition(90.0, -30.0, 1.0),
        PolarPosition(-90.0, -30.0, 1.0),  PolarPosition(135.0, -30.0, 1.0),
        PolarPosition(-135.0, -30.0, 1.0), PolarPosition(30.0, 30.0, 1.0),
        PolarPosition(-30.0, 30.0, 1.0),   PolarPosition(0.0, 30.0, 1.0),
        PolarPosition(90.0, 30.0, 1.0),    PolarPosition(-90.0, 30.0, 1.0),
        PolarPosition(135.0, 30.0, 1.0),   PolarPosition(-135.0, 30.0, 1.0)};
    REQUIRE(extraChannels.size() == expectedPositions.size());
    for (size_t i = 0; i < extraChannels.size(); ++i) {
      REQUIRE(extraChannels[i].polarPosition() == expectedPositions[i]);
    }
    Eigen::MatrixXd expectedDownmix = createDownmixMatrixFromIndices(
        std::vector<int>{0, 1, 2, 3, 4, 5, 6, 0, 1, 2, 3, 4, 5, 6},
        layout.channels().size());
    REQUIRE(downmix == expectedDownmix);
  };

  SECTION("4+7+0") {
    Layout layout = getLayout("4+7+0").withoutLfe();
    std::tie(extraChannels, downmix) = extraPosVerticalNominal(layout);
    std::vector<PolarPosition> expectedPositions = {
        PolarPosition(30.0, -30.0, 1.0),  PolarPosition(-30.0, -30.0, 1.0),
        PolarPosition(0.0, -30.0, 1.0),   PolarPosition(90.0, -30.0, 1.0),
        PolarPosition(-90.0, -30.0, 1.0), PolarPosition(135.0, -30.0, 1.0),
        PolarPosition(-135.0, -30.0, 1.0)};
    REQUIRE(extraChannels.size() == expectedPositions.size());
    for (size_t i = 0; i < extraChannels.size(); ++i) {
      REQUIRE(extraChannels[i].polarPosition() == expectedPositions[i]);
    }
    Eigen::MatrixXd expectedDownmix = createDownmixMatrixFromIndices(
        std::vector<int>{0, 1, 2, 3, 4, 5, 6}, layout.channels().size());
    REQUIRE(downmix == expectedDownmix);
  };
};

std::vector<std::pair<Eigen::Vector3d, Eigen::VectorXd>> generatePositionGains(
    const Layout& layout) {
  std::vector<std::pair<Eigen::Vector3d, Eigen::VectorXd>> ret;
  auto positions = layout.positions();
  for (size_t i = 0; i < positions.size(); ++i) {
    Eigen::VectorXd expectedGain =
        Eigen::VectorXd::Zero(layout.channels().size());
    expectedGain(i) = 1.0;
    ret.push_back(
        std::make_pair(toCartesianVector3d(positions[i]), expectedGain));
  }
  return ret;
}

TEST_CASE("test_polar_point_source_panner") {
  Eigen::MatrixXd positions(4, 3);
  positions << cartT(30.0, 0.0, 1.0),  //
      cartT(0.0, 0.0, 1.0),  //
      cartT(-30.0, 0.0, 1.0),  //
      cartT(0.0, 30.0, 1.0);
  std::vector<Eigen::Vector3i> outputChannelsVec = {{0, 1, 3}, {2, 1, 3}};
  std::vector<std::unique_ptr<RegionHandler>> regions_1;
  for (const auto& outputChannels : outputChannelsVec) {
    regions_1.push_back(boost::make_unique<Triplet>(
        outputChannels, positions(outputChannels, {0, 1, 2})));
  }

  // should fail if not given enough channels
  REQUIRE_THROWS(PolarPointSourcePanner(
      std::move(regions_1), static_cast<int>(positions.rows() - 1)));

  std::vector<std::unique_ptr<RegionHandler>> regions_2;
  for (const auto& outputChannels : outputChannelsVec) {
    regions_2.push_back(boost::make_unique<Triplet>(
        outputChannels, positions(outputChannels, {0, 1, 2})));
  }
  PolarPointSourcePanner psp(std::move(regions_2));
  REQUIRE(psp.numberOfOutputChannels() == positions.rows());
  for (int i = 0; i < psp.numberOfOutputChannels(); ++i) {
    Eigen::VectorXd pv_req =
        Eigen::VectorXd::Zero(psp.numberOfOutputChannels());
    pv_req(i) = 1.0;
    auto pv = psp.handle(positions(i, {0, 1, 2}));
    REQUIRE(pv != boost::none);
    REQUIRE(pv->isApprox(pv_req));
  }
  REQUIRE(psp.handle(Eigen::Vector3d{0.0, -1.0, 0.0}) == boost::none);
}

Eigen::VectorXi getChannelFlipVector(const Eigen::MatrixXd& spkPositions) {
  Eigen::VectorXi channelFlipX = Eigen::VectorXi::Zero(spkPositions.rows());
  Eigen::VectorXd spkNorm(spkPositions.rows());
  Eigen::RowVector3d spkPositionFlipX;
  Eigen::VectorXd::Index minIndex;
  for (int i = 0; i < spkPositions.rows(); ++i) {
    spkPositionFlipX << -spkPositions(i, 0), spkPositions(i, 1),
        spkPositions(i, 2);
    spkNorm = (spkPositions.rowwise() - spkPositionFlipX).rowwise().norm();
    spkNorm.minCoeff(&minIndex);
    channelFlipX(i) = static_cast<int>(minIndex);
  }
  return channelFlipX;
}

TEST_CASE("test_all_layouts") {
  for (const auto& l : loadLayouts()) {
    Layout layout = l.withoutLfe();
    Eigen::MatrixXd spkPositions(layout.positions().size(), 3);
    for (int i = 0; i < spkPositions.rows(); ++i) {
      spkPositions.row(i) = toCartesianVector3d(layout.positions().at(i));
    }

    SECTION(layout.name()) {
      std::shared_ptr<PointSourcePanner> psp = configurePolarPanner(layout);
      Eigen::VectorXi channelFlipX = getChannelFlipVector(spkPositions);

      // calculate gains for every position on a grid
      Eigen::VectorXd azimuths = Eigen::VectorXd::LinSpaced(21, -180.0, 180.0);
      Eigen::VectorXd elevations = Eigen::VectorXd::LinSpaced(11, -90.0, 90.0);

      std::vector<std::vector<Eigen::VectorXd>> pvs(61);
      for (int a = 0; a < azimuths.size(); ++a) {
        for (int e = 0; e < elevations.size(); ++e) {
          double az = azimuths(a);
          double el = elevations(e);
          Eigen::VectorXd position = cartT(az, el, 1.0);
          auto pv = psp->handle(position);
          REQUIRE(pv != boost::none);
          REQUIRE((pv->array() >= 0.0).any());

          // check that the gains are normalised

          // stereo is normalised only at the front, and at the back at -3dB
          if (layout.name() == std::string("0+2+0")) {
            if (abs(az) <= 30.0 && el == 0.0) {
              REQUIRE(pv->norm() == Approx(1.0));
            } else if (abs(az) >= 110.0 && el == 0.0) {
              REQUIRE(pv->norm() == Approx(sqrt(0.5)));
            }
          } else {
            REQUIRE(pv->norm() == Approx(1.0));
          }

          // check that the velocity vector matches the source position
          bool doPositionCheck = true;
          if (layout.name() == std::string("0+2+0")) {
            if (abs(az) >= 30.0 || el != 0.0) {
              doPositionCheck = false;
            }
          } else if (layout.name() == std::string("0+5+0") ||
                     layout.name() == std::string("2+5+0") ||
                     layout.name() == std::string("0+7+0")) {
            if (el != 0.0) {
              doPositionCheck = false;
            }
          }
          // only 9+10+3 has no remapping above the horizontal plane
          // all layouts have remapping below the horizontal plane
          if (layout.name() == std::string("9+10+3")) {
            if (el < 0.0) {
              doPositionCheck = false;
            }
          } else if (el != 0.0) {
            doPositionCheck = false;
          }

          if (doPositionCheck) {
            Eigen::VectorXd vv = pv.get().transpose() * spkPositions;
            vv /= vv.norm();
            INFO("az: " << az << " != " << azimuth(vv));
            INFO("el: " << el << " != " << elevation(vv));
            REQUIRE(vv.isApprox(cart(az, el, 1.0)));
          }

          Eigen::Vector3d positionFlipX{-position(0), position(1), position(2)};
          auto pvFlipX = psp->handle(positionFlipX);

          REQUIRE(pv.get().isApprox(pvFlipX.get()(channelFlipX)));
        }
      }
    }
  }
}

TEST_CASE("configure_full_polar_panner") {
  SECTION("0+5+0") {
    auto layout = getLayout("0+5+0").withoutLfe();
    auto psp = configurePolarPanner(layout);
    for (const auto& pair : generatePositionGains(layout)) {
      auto pv = psp->handle(pair.first);
      REQUIRE(pv != boost::none);
      REQUIRE(pv.get().isApprox(pair.second));
    }
    SECTION("azimuth=15.0") {
      Eigen::VectorXd expectedGain =
          Eigen::VectorXd::Zero(layout.channels().size());
      expectedGain({0, 2}) << 1.0 / std::sqrt(2.0), 1.0 / std::sqrt(2.0);
      auto pv = psp->handle(cart(15.0, 0.0, 1.0));
      REQUIRE(pv != boost::none);
      REQUIRE(pv.get().isApprox(expectedGain));
    };
    SECTION("azimuth=-15.0") {
      Eigen::VectorXd expectedGain =
          Eigen::VectorXd::Zero(layout.channels().size());
      expectedGain({1, 2}) << 1.0 / std::sqrt(2.0), 1.0 / std::sqrt(2.0);
      auto pv = psp->handle(cart(-15.0, 0.0, 1.0));
      REQUIRE(pv != boost::none);
      REQUIRE(pv.get().isApprox(expectedGain));
    };
  };
  SECTION("2+5+0") {
    auto layout = getLayout("2+5+0").withoutLfe();
    auto psp = configurePolarPanner(layout);
    for (const auto& pair : generatePositionGains(layout)) {
      auto pv = psp->handle(pair.first);
      REQUIRE(pv != boost::none);
      REQUIRE(pv.get().isApprox(pair.second));
    }
  };
  SECTION("4+5+0") {
    auto layout = getLayout("4+5+0").withoutLfe();
    auto psp = configurePolarPanner(layout);
    for (const auto& pair : generatePositionGains(layout)) {
      auto pv = psp->handle(pair.first);
      REQUIRE(pv != boost::none);
      REQUIRE(pv.get().isApprox(pair.second));
    }
  };
  SECTION("4+5+1") {
    auto layout = getLayout("4+5+1").withoutLfe();
    auto psp = configurePolarPanner(layout);
    for (const auto& pair : generatePositionGains(layout)) {
      auto pv = psp->handle(pair.first);
      REQUIRE(pv != boost::none);
      REQUIRE(pv.get().isApprox(pair.second));
    }
  };
  SECTION("3+7+0") {
    auto layout = getLayout("3+7+0").withoutLfe();
    auto psp = configurePolarPanner(layout);
    for (const auto& pair : generatePositionGains(layout)) {
      auto pv = psp->handle(pair.first);
      REQUIRE(pv != boost::none);
      REQUIRE(pv.get().isApprox(pair.second));
    }
  };
  SECTION("4+9+0") {
    auto layout = getLayout("4+9+0").withoutLfe();
    auto psp = configurePolarPanner(layout);
    for (const auto& pair : generatePositionGains(layout)) {
      auto pv = psp->handle(pair.first);
      REQUIRE(pv != boost::none);
      REQUIRE(pv.get().isApprox(pair.second));
    }
  };
  SECTION("9+10+3") {
    auto layout = getLayout("9+10+3").withoutLfe();
    auto psp = configurePolarPanner(layout);
    for (const auto& pair : generatePositionGains(layout)) {
      auto pv = psp->handle(pair.first);
      REQUIRE(pv != boost::none);
      REQUIRE(pv.get().isApprox(pair.second));
    }
  };
  SECTION("0+7+0") {
    auto layout = getLayout("0+7+0").withoutLfe();
    auto psp = configurePolarPanner(layout);
    for (const auto& pair : generatePositionGains(layout)) {
      auto pv = psp->handle(pair.first);
      REQUIRE(pv != boost::none);
      REQUIRE(pv.get().isApprox(pair.second));
    }
  };
  SECTION("4+7+0") {
    auto layout = getLayout("4+7+0").withoutLfe();
    auto psp = configurePolarPanner(layout);
    for (const auto& pair : generatePositionGains(layout)) {
      auto pv = psp->handle(pair.first);
      REQUIRE(pv != boost::none);
      REQUIRE(pv.get().isApprox(pair.second));
    }
  };
};

TEST_CASE("screen_loudspeaker_positions") {
  auto layout = getLayout("4+9+0").withoutLfe();

  SECTION("too wide") {
    SECTION("+") {
      for (auto& channel : layout.channels())
        if (channel.name() == "M+SC") channel.polarPosition({40.0, 0.0, 1.0});
      REQUIRE_THROWS_AS(configurePolarPanner(layout), not_implemented);
    }
    SECTION("-") {
      for (auto& channel : layout.channels())
        if (channel.name() == "M-SC") channel.polarPosition({-40.0, 0.0, 1.0});
      REQUIRE_THROWS_AS(configurePolarPanner(layout), not_implemented);
    }
  }

  SECTION("not in range") {
    SECTION("+") {
      for (auto& channel : layout.channels())
        if (channel.name() == "M+SC") channel.polarPosition({30.0, 0.0, 1.0});
      REQUIRE_THROWS_AS(configurePolarPanner(layout), invalid_argument);
    }

    SECTION("-") {
      for (auto& channel : layout.channels())
        if (channel.name() == "M-SC") channel.polarPosition({-30.0, 0.0, 1.0});
      REQUIRE_THROWS_AS(configurePolarPanner(layout), invalid_argument);
    }
  }
}

TEST_CASE("hull") {
  for (auto& layoutFull : loadLayouts()) {
    if (layoutFull.name() == "0+2+0") continue;
    auto layout = layoutFull.withoutLfe();

    SECTION(layout.name()) {
      std::vector<Eigen::Vector3d> positionsReal;
      std::vector<Eigen::Vector3d> positionsNominal;
      std::set<int> virtualVerts;
      Eigen::MatrixXd downmix;
      std::tie(positionsReal, positionsNominal, virtualVerts, downmix) =
          getAugmentedLayout(layout);

      std::vector<Facet> facets_precomputed = FACETS.at(layout.name());

      // check that we're not close to a tolerance that is too big or small
      for (double tolerance : {1e-6, 1e-5, 1e-4}) {
        SECTION("tol = " + std::to_string(tolerance)) {
          std::vector<Facet> facets_calculated =
              convex_hull(positionsNominal, tolerance);

          std::sort(facets_precomputed.begin(), facets_precomputed.end());
          std::sort(facets_calculated.begin(), facets_calculated.end());

          REQUIRE(facets_precomputed == facets_calculated);
        }
      }

#ifdef CATCH_CONFIG_ENABLE_BENCHMARKING
      if (layout.name() == "9+10+3") {
        BENCHMARK("hull") { return convex_hull(positionsNominal); };
      }
#endif
    }
  }
}
