#include "ear/conversion.hpp"
#include <Eigen/Dense>
#include <Eigen/LU>
#include <boost/math/constants/constants.hpp>
#include <cmath>
#include <vector>
#include "common/geom.hpp"
#include "ear/helpers/assert.hpp"

const double PI = boost::math::constants::pi<double>();

namespace ear {
  namespace conversion {
    static double sign(double x) {
      if (x < 0.0)
        return -1.0;
      else if (x > 0.0)
        return 1.0;
      else
        return 0.0;
    }

    // static data

    struct Sector {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      double polar_start_az;  // or left
      double polar_end_az;  // or right
      double cart_start_az;
      double cart_end_az;

      Eigen::Vector2d cart_start_pos;
      Eigen::Vector2d cart_end_pos;

      Eigen::Matrix2d m;
    };

    using Sectors = std::vector<Sector, Eigen::aligned_allocator<Sector>>;

    static Sectors make_sectors() {
      Sectors out;

      using MappintPoint = std::pair<double, Eigen::Vector3d>;

      std::vector<MappintPoint> points = {
          {0.0, {0.0, 1.0, 0.0}},     {-30.0, {1.0, 1.0, 0.0}},
          {-110.0, {1.0, -1.0, 0.0}}, {110.0, {-1.0, -1.0, 0.0}},
          {30.0, {-1.0, 1.0, 0.0}},
      };

      for (size_t i = 0; i < points.size(); i++) {
        size_t j = (i + 1) % points.size();

        double polar_start_az = points.at(i).first;
        double polar_end_az = points.at(j).first;
        Eigen::Vector3d cart_start_pos = points.at(i).second;
        Eigen::Vector3d cart_end_pos = points.at(j).second;

        double cart_start_az = azimuth(cart_start_pos);
        double cart_end_az = azimuth(cart_end_pos);

        Eigen::Matrix2d m_inv;
        m_inv.transpose() << cart_start_pos.head<2>(), cart_end_pos.head<2>();
        Eigen::Matrix2d m = m_inv.inverse();

        out.push_back({polar_start_az, polar_end_az, cart_start_az, cart_end_az,
                       cart_start_pos.head<2>(), cart_end_pos.head<2>(), m});
      }

      return out;
    }

    static const Sectors sectors = make_sectors();
    static const double el_top = 30;
    static const double el_top_tilde = 45;

    // internal functions

    static const Sector &find_cart_sector(double az) {
      for (const Sector &sector : sectors)
        if (insideAngleRange(az, sector.cart_end_az, sector.cart_start_az))
          return sector;

      throw internal_error("could not find sector");
    }

    static const Sector &find_polar_sector(double az) {
      for (const Sector &sector : sectors)
        if (insideAngleRange(az, sector.polar_end_az, sector.polar_start_az))
          return sector;

      throw internal_error("could not find sector");
    }

    double map_az_to_linear(double left_az, double right_az, double azimuth) {
      double mid_az = (left_az + right_az) / 2.0;
      double az_range = right_az - mid_az;

      double rel_az = azimuth - mid_az;

      double gain_r =
          0.5 + 0.5 * std::tan(radians(rel_az)) / std::tan(radians(az_range));

      return std::atan2(gain_r, 1.0 - gain_r) * (2.0 / PI);
    }

    double map_linear_to_az(double left_az, double right_az, double x) {
      double mid_az = (left_az + right_az) / 2.0;
      double az_range = right_az - mid_az;

      double gain_l_ = std::cos(x * (PI / 2.0));
      double gain_r_ = std::sin(x * (PI / 2.0));

      double gain_r = gain_r_ / (gain_l_ + gain_r_);

      double rel_az = degrees(
          std::atan(2.0 * (gain_r - 0.5) * std::tan(radians(az_range))));

      return mid_az + rel_az;
    }

    // point conversions

    CartesianPosition pointPolarToCart(const PolarPosition &pos) {
      double r_xy, z;

      if (std::abs(pos.elevation) > el_top) {
        double el_tilde =
            el_top_tilde + (90.0 - el_top_tilde) *
                               (std::abs(pos.elevation) - el_top) /
                               (90.0 - el_top);
        z = pos.distance * sign(pos.elevation);
        r_xy = pos.distance * std::tan(radians(90.0 - el_tilde));
      } else {
        double el_tilde = el_top_tilde * pos.elevation / el_top;
        z = std::tan(radians(el_tilde)) * pos.distance;
        r_xy = pos.distance;
      }

      const Sector &sector = find_polar_sector(pos.azimuth);

      double rel_az = relativeAngle(sector.polar_end_az, pos.azimuth);
      double rel_left_az =
          relativeAngle(sector.polar_end_az, sector.polar_start_az);
      double p = map_az_to_linear(rel_left_az, sector.polar_end_az, rel_az);
      ear_assert(-1e-6 <= p && p <= 1.0 + 1e-6, "p should be 0 to 1");

      Eigen::Vector2d newpos =
          r_xy * (sector.cart_start_pos +
                  (sector.cart_end_pos - sector.cart_start_pos) * p);

      return {newpos.x(), newpos.y(), z};
    }

    PolarPosition pointCartToPolar(const CartesianPosition &pos) {
      double eps = 1e-10;
      if (std::abs(pos.X) < eps && std::abs(pos.Y) < eps) {
        if (std::abs(pos.Z) < eps)
          return {0.0, 0.0, 0.0};
        else
          return {0.0, sign(pos.Z) * 90.0, std::abs(pos.Z)};
      }

      const Sector &sector = find_cart_sector(azimuth({pos.X, pos.Y, 0.0}));

      Eigen::Vector2d g_lr = Eigen::RowVector2d{pos.X, pos.Y} * sector.m;
      double r_xy = g_lr.sum();

      double rel_left_az =
          relativeAngle(sector.polar_end_az, sector.polar_start_az);
      double az =
          map_linear_to_az(rel_left_az, sector.polar_end_az, g_lr(1) / r_xy);
      az = relativeAngle(-180.0, az);

      double el_tilde = degrees(std::atan(pos.Z / r_xy));

      double d, el;
      if (std::abs(el_tilde) > el_top_tilde) {
        double abs_el = el_top + (90.0 - el_top) *
                                     (std::abs(el_tilde) - el_top_tilde) /
                                     (90.0 - el_top_tilde);
        el = sign(el_tilde) * abs_el;
        d = std::abs(pos.Z);
      } else {
        el = el_top * el_tilde / el_top_tilde;
        d = r_xy;
      }

      return {az, el, d};
    }

    // extent conversion

    static Eigen::Vector3d whd2xyz(const ExtentParams &extent) {
      double x_size_width =
          extent.width < 180.0 ? std::sin(radians(extent.width / 2.0)) : 1.0;
      double y_size_width = (1.0 - std::cos(radians(extent.width / 2.0))) / 2.0;

      double z_size_height =
          extent.height < 180.0 ? std::sin(radians(extent.height / 2.0)) : 1.0;
      double y_size_height =
          (1.0 - std::cos(radians(extent.height / 2.0))) / 2.0;

      double y_size_depth = extent.depth;

      return {x_size_width,
              std::max({y_size_width, y_size_height, y_size_depth}),
              z_size_height};
    }

    static ExtentParams xyz2whd(double s_x, double s_y, double s_z) {
      double width_from_sx = 2.0 * degrees(std::asin(s_x));
      double width_from_sy = 2.0 * degrees(std::acos(1.0 - 2.0 * s_y));

      double width =
          width_from_sx + s_x * std::max(width_from_sy - width_from_sx, 0.0);

      double height_from_sz = 2.0 * degrees(std::asin(s_z));
      double height_from_sy = 2.0 * degrees(std::acos(1.0 - 2.0 * s_y));

      double height =
          height_from_sz + s_z * std::max(height_from_sy - height_from_sz, 0.0);

      // depth is the y size that is not accounted for by the calculated width
      // and height
      double equiv_y = whd2xyz({width, height, 0.0}).y();
      double depth = std::max(0.0, s_y - equiv_y);

      return {width, height, depth};
    }

    std::pair<PolarPosition, ExtentParams> extentCartToPolar(
        const CartesianPosition &pos, const ExtentParams &extent) {
      PolarPosition polar_pos = pointCartToPolar(pos);

      Eigen::Array3d extent_vec(extent.width, extent.depth, extent.height);
      Eigen::Array33d LCS =
          localCoordinateSystem(polar_pos.azimuth, polar_pos.elevation);
      Eigen::Array33d M = LCS.transpose().colwise() * extent_vec;

      Eigen::Vector3d extent_vec_rot = M.colwise().norm();
      ExtentParams polar_extent =
          xyz2whd(extent_vec_rot.x(), extent_vec_rot.y(), extent_vec_rot.z());

      return {polar_pos, polar_extent};
    }

    std::pair<CartesianPosition, ExtentParams> extentPolarToCart(
        const PolarPosition &pos, const ExtentParams &extent) {
      CartesianPosition pos_c = pointPolarToCart(pos);

      Eigen::Array3d front_size = whd2xyz(extent);

      Eigen::Array33d LCS = localCoordinateSystem(pos.azimuth, pos.elevation);
      Eigen::Array33d M = LCS.colwise() * front_size;
      Eigen::Vector3d size = M.colwise().norm();

      return {pos_c, {size.x(), size.z(), size.y()}};
    }

    // wrappers for OTM

    struct guess_cartesian_flag : public boost::static_visitor<bool> {
      bool operator()(const PolarPosition &) const { return false; }

      bool operator()(const CartesianPosition &) const { return true; }
    };

    void toPolar(ObjectsTypeMetadata &otm) {
      otm.cartesian =
          boost::apply_visitor(guess_cartesian_flag(), otm.position);

      if (otm.cartesian) {
        CartesianPosition cart_pos =
            boost::get<CartesianPosition>(otm.position);
        ExtentParams cart_extent{otm.width, otm.height, otm.depth};

        PolarPosition polar_pos;
        ExtentParams polar_extent;
        std::tie(polar_pos, polar_extent) =
            extentCartToPolar(cart_pos, cart_extent);

        otm.cartesian = false;
        otm.position = polar_pos;
        otm.width = polar_extent.width;
        otm.height = polar_extent.height;
        otm.depth = polar_extent.depth;
      }
    }

    void toCartesian(ObjectsTypeMetadata &otm) {
      otm.cartesian =
          boost::apply_visitor(guess_cartesian_flag(), otm.position);

      if (!otm.cartesian) {
        PolarPosition polar_pos = boost::get<PolarPosition>(otm.position);
        ExtentParams polar_extent{otm.width, otm.height, otm.depth};

        CartesianPosition cart_pos;
        ExtentParams cart_extent;
        std::tie(cart_pos, cart_extent) =
            extentPolarToCart(polar_pos, polar_extent);

        otm.cartesian = true;
        otm.position = cart_pos;
        otm.width = cart_extent.width;
        otm.height = cart_extent.height;
        otm.depth = cart_extent.depth;
      }
    }
  }  // namespace conversion
}  // namespace ear
