#include "ear/common/geom.hpp"

#include <cmath>

namespace ear {

  bool insideAngleRange(double x, double start, double end, double tol) {
    // end is clockwise from start; if end is start + 360, this rotation is
    // preserved; this makes sure that a range of (-180, 180) or (0, 360)
    // means any angle, while (-180, -180) or (0, 0) means a single angle,
    // even though -180/180 and 0/360 are nominally the same angle
    while (end - 360.0 > start) {
      end -= 360.0;
    }
    while (end < start) {
      end += 360.0;
    }
    double start_tol = start - tol;
    while (x - 360.0 >= start_tol) {
      x -= 360.0;
    }
    while (x < start_tol) {
      x += 360.0;
    }
    // x is greater than equal to start-tol, so we only need to compare
    // against the end.
    return x <= end + tol;
  }

  Eigen::VectorXi argSort(const Eigen::VectorXd& v) {
    Eigen::VectorXi indices = Eigen::VectorXi::LinSpaced(
        static_cast<int>(v.size()), 0, static_cast<int>(v.size()));
    std::sort(indices.begin(), indices.end(),
              [&v](size_t i1, size_t i2) { return v[i1] < v[i2]; });
    return indices;
  }

  Eigen::VectorXi ngonVertexOrder(Eigen::MatrixXd vertices) {
    Eigen::Vector3d centre = vertices.colwise().mean();

    // Pick two vertices to form a plane (with the third point being the
    // origin); the vertices are ordered by the angles of points projected onto
    // this plane. The first is picked arbitrarily, the second is picked to
    // minimise the colinearity with the first.
    Eigen::Vector3d a;
    Eigen::Vector3d b;
    a = vertices.row(0).transpose() - centre;
    double min = std::numeric_limits<double>::max();
    for (int i = 1; i < vertices.rows(); ++i) {
      Eigen::Vector3d vertex = vertices.row(i).transpose() - centre;
      double angle = abs(vertex.dot(a));
      if (angle < min) {
        min = angle;
        b = vertex;
      }
    }
    // These vectors are neither normalised or orthogonal, so the projection
    // onto them produces a linear transformation from the projection onto the
    // plane (relative to the origin); this is fine, as affine transformations
    // preserve straight lines.

    // find the angle of the projection of each vertex onto the plane
    auto verticesRelCentre = vertices.rowwise() - centre.transpose();
    auto vertexAngles =
        (verticesRelCentre * a)
            .binaryExpr(verticesRelCentre * b,
                        [](double a, double b) { return std::atan2(a, b); });
    return argSort(vertexAngles);
  }

  double azimuth(Eigen::Vector3d position) {
    return -degrees(atan2(position[0], position[1]));
  }

  double elevation(Eigen::Vector3d position) {
    auto radius = hypot(position[0], position[1]);
    return degrees(atan2(position[2], radius));
  }

  double distance(Eigen::Vector3d position) { return position.norm(); }

  Eigen::Vector3d cart(double azimuth, double elevation, double distance) {
    return Eigen::Vector3d(
        sin(radians(-azimuth)) * cos(radians(elevation)) * distance,
        cos(radians(-azimuth)) * cos(radians(elevation)) * distance,
        sin(radians(elevation)) * distance);
  };

  Eigen::Vector3d toCartesianVector3d(CartesianSpeakerPosition position) {
    return Eigen::Vector3d(position.X, position.Y, position.Z);
  }

  Eigen::Vector3d toCartesianVector3d(SpeakerPosition position) {
    if (position.type() == typeid(PolarSpeakerPosition)) {
      PolarSpeakerPosition pos = boost::get<PolarSpeakerPosition>(position);
      return toCartesianVector3d(pos);
    } else {
      CartesianSpeakerPosition pos =
          boost::get<CartesianSpeakerPosition>(position);
      return toCartesianVector3d(pos);
    }
  }

  Eigen::Vector3d toCartesianVector3d(PolarSpeakerPosition position) {
    return cart(position.azimuth, position.elevation, position.distance);
  }

  Eigen::Vector3d toCartesianVector3d(CartesianPosition position) {
    return Eigen::Vector3d(position.X, position.Y, position.Z);
  }

  PolarPosition toPolarPosition(CartesianPosition position) {
    auto cart_array = toCartesianVector3d(position);
    return PolarPosition(azimuth(cart_array), elevation(cart_array),
                         distance(cart_array));
  }

  Eigen::Vector3d toCartesianVector3d(PolarPosition position) {
    return cart(position.azimuth, position.elevation, position.distance);
  }

  CartesianPosition toCartesianPosition(PolarPosition position) {
    auto cartesian = toCartesianVector3d(position);
    return CartesianPosition(cartesian[0], cartesian[1], cartesian[2]);
  }

  Eigen::Vector3d toNormalisedVector3d(PolarPosition position) {
    return cart(position.azimuth, position.elevation, 1.0);
  }

  Eigen::MatrixXd toPositionsMatrix(
      const std::vector<PolarPosition>& positions) {
    Eigen::MatrixXd ret(positions.size(), 3);
    for (unsigned int i = 0; i < positions.size(); ++i) {
      ret.row(i) = toCartesianVector3d(positions[i]);
    }
    return ret;
  }

}  // namespace ear
