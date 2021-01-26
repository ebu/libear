#pragma once
#include <Eigen/Geometry>
#include <boost/math/constants/constants.hpp>
#include <limits>
#include <utility>

namespace ear {
  /// tests for intersections between a great circle and an arc on the surface
  /// of a unit sphere
  class GreatCircleArcIntersection {
    using Vec2 = Eigen::Vector2d;

   public:
    /// the plane of the arc is defined by front and right, which must be
    /// orthogonal
    ///
    /// opening_angle is the angle between the centre of the arc, the centre of
    /// the sphere, and the arc itself
    ///
    /// wrap angle is the angle between the mid-point of the arc, the centre of
    /// the arc, and one end of the arc
    ///
    /// the vector from the centre of the arc to the mid-point is the same as
    /// front
    ///
    /// if opening_angle is less than PI/2, the vector from the centre of the
    /// sphere to the centre of the arc is cross(right, front)
    GreatCircleArcIntersection(const Eigen::Ref<const Eigen::Vector3d> &front,
                               Eigen::Vector3d right, double opening_angle,
                               double wrap_angle) {
      const double PI = boost::math::constants::pi<double>();
      assert(std::abs(front.dot(right)) < 1e-6);

      // for wide opening angles, invert so that the arc is always above the
      // front-right plane
      if (opening_angle > PI / 2) {
        right *= -1;
        opening_angle = PI - opening_angle;
      }

      height = cos(opening_angle);

      Eigen::Vector3d normal = right.cross(front);
      normal.normalize();

      T.row(0) = -right;
      T.row(1) = front;
      T.row(2) = -normal;
      r = std::abs(sin(opening_angle));
      wrap_angle = std::min(std::abs(wrap_angle), PI);
      double test_angle = wrap_angle - PI / 2;
      test_vec = {sin(test_angle), cos(test_angle)};
    }

    /// does the great circle between p1_in and p2_in intersect the arc defined
    /// in the constructor? p1_in and p2_in must not be equal.
    bool intersects(const Eigen::Ref<const Eigen::Vector3d> &p1_in,
                    const Eigen::Ref<const Eigen::Vector3d> &p2_in) const {
      assert((p1_in - p2_in).squaredNorm() > 1e-6);

      Eigen::Vector3d p1 = T * p1_in;
      Eigen::Vector3d p2 = T * p2_in;

      // switch p1 and p2 so that if p2 intersects the plane, p1 definitely
      // does
      if (p1.z() > p2.z()) std::swap(p1, p2);

      if (p1.z() > -1e-6)
        // neither points on plane
        return false;

      // p1 projected onto plane
      Vec2 p = p1.head(2) * (-height / p1.z());
      Vec2 v;  // vector from p1 to p2 in plane (normalised)
      double d;  // distance along v to p2 (maybe inf)

      if (p2.z() < -1e-6) {
        // both points on plane
        Vec2 p2_plane = p2.head(2) * (-height / p2.z());
        v = p2_plane - p;
        d = v.norm();
        v /= d;
      } else {
        // only one point on the plane
        // find the direction of the line; both if these are the same, but
        // divide by zero (and are inaccurate) in different regions.
        //
        // direction is p1 + p2 * x for some x such that p1.z + x*p2.z == 0
        if (p2.z() > 0.707) {
          double x = -p1.z() / p2.z();
          v = p1.head(2) + p2.head(2) * x;
          assert(std::abs(p1.z() + p2.z() * x) < 1e-6);
        } else {
          double x = -p2.z() / p1.z();
          v = p1.head(2) * x + p2.head(2);
          assert(std::abs(p1.z() * x + p2.z()) < 1e-6);
        }

        v.normalize();
        d = std::numeric_limits<double>::infinity();
      }

      // check if a point on the circle is on the segment and arc
      auto test = [&](const Vec2 &cp) {
        assert(std::abs(cp.squaredNorm() - r * r) < 1e-6);
        double t = (cp - p).dot(v);
        Vec2 cp_right(std::abs(cp.x()), cp.y());
        return -1e-6 < t && t < d + 1e-6 && test_vec.dot(cp_right) >= -1e-6;
      };

      // find circle intersections, from here:
      // https://mathworld.wolfram.com/Circle-LineIntersection.html

      double dr2 = v.x() * v.x() + v.y() * v.y();
      Vec2 q = p + v;
      double D = p.x() * q.y() - q.x() * p.y();
      double disc = r * r * dr2 - D * D;

      if (disc < 0)
        return false;
      else if (disc < 1e-6) {
        double x = D * v.y() / dr2;
        double y = -D * v.x() / dr2;
        return test({x, y});
      } else {
        double sign_dy = v.y() >= 0 ? 1.0 : -1.0;
        double root_disc = sqrt(disc);

        {
          double x = (D * v.y() + sign_dy * v.x() * root_disc) / dr2;
          double y = (-D * v.x() + std::abs(v.y()) * root_disc) / dr2;
          if (test({x, y})) return true;
        }
        {
          double x = (D * v.y() - sign_dy * v.x() * root_disc) / dr2;
          double y = (-D * v.x() - std::abs(v.y()) * root_disc) / dr2;
          if (test({x, y})) return true;
        }
        return false;
      }
    }

   private:
    Eigen::Matrix3d T;
    double r;
    double height;
    Eigen::Vector2d test_vec;
  };
}  // namespace ear
