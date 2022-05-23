#include <array>
#include <catch2/catch.hpp>
#include <vector>
#include "ear/conversion.hpp"

using namespace ear;
using namespace ear::conversion;

namespace ear {
  inline std::ostream &operator<<(std::ostream &s, const PolarPosition &pos) {
    return s << "PolarPosition{" << pos.azimuth << ", " << pos.elevation << ", "
             << pos.distance << "}";
  }

  inline std::ostream &operator<<(std::ostream &s,
                                  const CartesianPosition &pos) {
    return s << "CartesianPosition{" << pos.X << ", " << pos.Y << ", " << pos.Z
             << "}";
  }

  namespace conversion {
    inline std::ostream &operator<<(std::ostream &s, const ExtentParams &pos) {
      return s << "ExtentParams{" << pos.width << ", " << pos.height << ", "
               << pos.depth << "}";
    }
  }  // namespace conversion
}  // namespace ear

static void check_polar_equal(const PolarPosition &p1,
                              const PolarPosition &p2) {
  if (std::abs(p1.elevation) < 90.0 - 1e6)
    CHECK(p2.azimuth == Approx(p1.azimuth).margin(1e-6));
  CHECK(p2.elevation == Approx(p1.elevation).margin(1e-6));
  CHECK(p2.distance == Approx(p1.distance).margin(1e-6));
}

static void check_cart_equal(const CartesianPosition &p1,
                             const CartesianPosition &p2) {
  CHECK(p2.X == Approx(p1.X).margin(1e-6));
  CHECK(p2.Y == Approx(p1.Y).margin(1e-6));
  CHECK(p2.Z == Approx(p1.Z).margin(1e-6));
}

static void check_extent_equal(const ExtentParams &p1, const ExtentParams &p2) {
  CHECK(p2.width == Approx(p1.width).margin(1e-6));
  CHECK(p2.height == Approx(p1.height).margin(1e-6));
  CHECK(p2.depth == Approx(p1.depth).margin(1e-6));
}

TEST_CASE("conversion_cartesian_polar_loop") {
  double eps = 1e-6;
  for (double az : {0.0, -10.0, 10.0, 90.0, -90.0, 150.0, -150.0})
    for (double el : {0.0, -10.0, 10.0, -45.0, 45.0, -90.0, 90.0})
      for (double dist : {0.5, 1.0}) {
        PolarPosition polar_pos{az, el, dist};
        CartesianPosition cart_pos = pointPolarToCart(polar_pos);
        PolarPosition polar_pos_again = pointCartToPolar(cart_pos);

        INFO("polar" << polar_pos);
        INFO("cart" << cart_pos);
        INFO("polar again" << polar_pos_again);

        check_polar_equal(polar_pos, polar_pos_again);
      }
}

TEST_CASE("conversion_poles") {
  for (double sign : {-1.0, 1.0})
    for (double d : {0.5, 1.0, 2.0}) {
      check_cart_equal(pointPolarToCart({0.0, sign * 90.0, d}),
                       {0.0, 0.0, sign * d});
      check_polar_equal(pointCartToPolar({0.0, 0.0, sign * d}),
                        {0.0, sign * 90.0, d});
    }
}

TEST_CASE("conversion_centre") {
  for (double az : {-90.0, 0.0, 90.0})
    for (double el : {-90.0, 0.0, 90.0})
      check_cart_equal(pointPolarToCart({az, el, 0.0}), {0.0, 0.0, 0.0});

  CHECK(pointCartToPolar({0.0, 0.0, 0.0}).distance == Approx(0.0).margin(1e-6));
}

TEST_CASE("conversion_whd_mapping") {
  std::vector<std::tuple<double, double, std::string>> whd_mappings = {
      // azimuth, elevation, Cartesian equivalent of Width, Height, Depth
      {0.0, 0.0, "whd"},  //
      {90.0, 0.0, "dhw"},  // polar width -> Cartesian depth etc.
      {-90.0, 0.0, "dhw"},  //
      {180.0, 0.0, "whd"},  //
      {0.0, 90.0, "wdh"},  //
      {0.0, -90.0, "wdh"},  //
  };

  auto get_axis = [](const ExtentParams &ext) {
    std::array<double, 3> ext_arr{ext.width, ext.height, ext.depth};
    auto it = std::max_element(ext_arr.begin(), ext_arr.end());
    return "whd"[std::distance(ext_arr.begin(), it)];
  };

  for (auto &mapping : whd_mappings) {
    double az, el;
    std::string cart_whd;
    std::tie(az, el, cart_whd) = mapping;

    PolarPosition pos{az, el, 1.0};

    for (size_t axis_idx = 0; axis_idx < 3; axis_idx++) {
      char polar_axis = "whd"[axis_idx];
      char cart_axis = cart_whd[axis_idx];

      {
        CartesianPosition cart_pos;
        ExtentParams cart_extent;
        std::tie(cart_pos, cart_extent) =
            extentPolarToCart(pos, {polar_axis == 'w' ? 20.0 : 0.0,
                                    polar_axis == 'h' ? 20.0 : 0.0,
                                    polar_axis == 'd' ? 0.2 : 0.0});

        REQUIRE(get_axis(cart_extent) == cart_axis);
      }

      {
        PolarPosition polar_pos;
        ExtentParams polar_extent;
        std::tie(polar_pos, polar_extent) = extentCartToPolar(
            pointPolarToCart(pos),
            {cart_axis == 'w' ? 0.1 : 0.0, cart_axis == 'h' ? 0.1 : 0.0,
             cart_axis == 'd' ? 0.1 : 0.0});

        REQUIRE(get_axis(polar_extent) == polar_axis);
      }
    }
  }
}

TEST_CASE("conversion_reference") {
  {
    auto res = extentPolarToCart({10.0, 20.0, 0.3}, {40.0, 50.0, 0.6});
    CartesianPosition expected_pos{-0.08972503721988338, 0.3,
                                   0.1732050807568877};
    ExtentParams expected_extent{0.35166171614357594, 0.4470181645863707,
                                 0.5762749096794243};
    INFO(res.first << " " << res.second);
    check_cart_equal(res.first, expected_pos);
    check_extent_equal(res.second, expected_extent);
  }

  {
    auto res = extentCartToPolar({0.9, 0.8, 0.1}, {0.3, 0.5, 0.4});

    PolarPosition expected_pos{-34.85107611658391, 4.226794497273273,
                               0.9000000000000001};
    ExtentParams expected_extent{76.50724453298275, 104.9708107421662,
                                 0.1756348204517474};

    INFO(res.first << " " << res.second);
    check_polar_equal(res.first, expected_pos);
    check_extent_equal(res.second, expected_extent);
  }
}

TEST_CASE("conversion_wrappers") {
  auto get_extent = [](const ObjectsTypeMetadata &otm) {
    return ExtentParams{otm.width, otm.height, otm.depth};
  };

  // polar to cart
  {
    ObjectsTypeMetadata otm;
    otm.position = PolarPosition{10.0, 20.0, 0.3};
    otm.cartesian = false;
    otm.width = 40.0;
    otm.height = 50.0;
    otm.depth = 0.6;

    ObjectsTypeMetadata otm_cart = otm;
    toCartesian(otm_cart);

    auto res = extentPolarToCart(boost::get<PolarPosition>(otm.position),
                                 get_extent(otm));

    check_cart_equal(boost::get<CartesianPosition>(otm_cart.position),
                     res.first);
    check_extent_equal(get_extent(otm_cart), res.second);

    CHECK(otm_cart.cartesian);
  }

  // fix cart flag polar
  {
    ObjectsTypeMetadata otm;
    otm.position = PolarPosition{10.0, 20.0, 0.3};
    otm.cartesian = true;

    toCartesian(otm);
    CHECK(otm.cartesian);
  }

  // cart to polar
  {
    ObjectsTypeMetadata otm;
    otm.position = CartesianPosition{0.9, 0.8, 0.1};
    otm.cartesian = true;
    otm.width = 0.3;
    otm.height = 0.5;
    otm.depth = 0.4;

    ObjectsTypeMetadata otm_polar = otm;
    toPolar(otm_polar);

    auto res = extentCartToPolar(boost::get<CartesianPosition>(otm.position),
                                 get_extent(otm));

    check_polar_equal(boost::get<PolarPosition>(otm_polar.position), res.first);
    check_extent_equal(get_extent(otm_polar), res.second);

    CHECK(!otm_polar.cartesian);
  }

  // fix cart flag polar
  {
    ObjectsTypeMetadata otm;
    otm.position = CartesianPosition{0.9, 0.8, 0.1};
    otm.cartesian = false;

    toPolar(otm);
    CHECK(!otm.cartesian);
  }
}
