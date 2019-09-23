#pragma once
#include <boost/optional.hpp>
#include <boost/variant.hpp>
#include <string>
#include "common_types.hpp"
#include "ear/export.h"
#include "screen.hpp"

namespace ear {

  // typeDefinition==DirectSpeakers

  struct EAR_EXPORT ScreenEdgeLock {
    /// `screenEdgeLock` attribute on `position` element with
    /// `coordinate="azimuth"` or `coordinate="X"`
    boost::optional<std::string> horizontal;
    /// `screenEdgeLock` attribute on `position` element with
    /// `coordinate="elevation"` or `coordinate="Z"`
    boost::optional<std::string> vertical;
  };

  struct EAR_EXPORT PolarSpeakerPosition {
    PolarSpeakerPosition(double az = 0.0, double el = 0.0, double dist = 1.0)
        : azimuth(az), elevation(el), distance(dist){};
    double azimuth;
    boost::optional<double> azimuthMin;
    boost::optional<double> azimuthMax;
    double elevation;
    boost::optional<double> elevationMin;
    boost::optional<double> elevationMax;
    double distance;
    boost::optional<double> distanceMin;
    boost::optional<double> distanceMax;
    ScreenEdgeLock screenEdgeLock;
  };

  struct EAR_EXPORT CartesianSpeakerPosition {
    CartesianSpeakerPosition(double X = 0.0, double Y = 1.0, double Z = 0.0)
        : X(X), Y(Y), Z(Z){};
    double X;
    boost::optional<double> XMin;
    boost::optional<double> XMax;
    double Y;
    boost::optional<double> YMin;
    boost::optional<double> YMax;
    double Z;
    boost::optional<double> ZMin;
    boost::optional<double> ZMax;
    ScreenEdgeLock screenEdgeLock;
  };

  using SpeakerPosition =
      boost::variant<PolarSpeakerPosition, CartesianSpeakerPosition>;

  struct EAR_EXPORT ChannelFrequency {
    boost::optional<double> lowPass = boost::none;
    boost::optional<double> highPass = boost::none;
  };

  struct EAR_EXPORT DirectSpeakersTypeMetadata {
    /// contents of the `speakerLabel` tags in this `audioBlockFormat`, in the
    /// order given in the AXML
    std::vector<std::string> speakerLabels = {};
    /// contents of the `position` elements
    SpeakerPosition position = PolarSpeakerPosition();
    /// `frequency` information contained in the `audioChannelFormat`
    ChannelFrequency channelFrequency = {};
    /// `audioPackFormatID` of the `audioPackFormat` directly referencing this
    /// channel, for example `AP_00010002`
    boost::optional<std::string> audioPackFormatID = boost::none;
  };

  // typeDefinition==Objects

  struct EAR_EXPORT ChannelLock {
    ChannelLock(bool flag = false,
                boost::optional<double> maxDistance = boost::none)
        : flag(flag), maxDistance(maxDistance){};
    bool flag;
    boost::optional<double> maxDistance;
  };

  struct EAR_EXPORT PolarObjectDivergence {
    PolarObjectDivergence(double divergence = 0.0, double azimuthRange = 45.0)
        : divergence(divergence), azimuthRange(azimuthRange){};
    double divergence;
    double azimuthRange;
  };
  struct EAR_EXPORT CartesianObjectDivergence {
    CartesianObjectDivergence(double divergence = 0.0,
                              double positionRange = 0.0)
        : divergence(divergence), positionRange(positionRange){};
    double divergence;
    double positionRange;
  };

  using ObjectDivergence =
      boost::variant<PolarObjectDivergence, CartesianObjectDivergence>;

  struct EAR_EXPORT PolarExclusionZone {
    float minAzimuth;
    float maxAzimuth;
    float minElevation;
    float maxElevation;
    float minDistance;
    float maxDistance;
    std::string label;
  };

  struct EAR_EXPORT CartesianExclusionZone {
    float minX;
    float maxX;
    float minY;
    float maxY;
    float minZ;
    float maxZ;
    std::string label;
  };

  using ExclusionZone =
      boost::variant<PolarExclusionZone, CartesianExclusionZone>;

  struct EAR_EXPORT ZoneExclusion {
    std::vector<ExclusionZone> zones;
  };

  struct EAR_EXPORT ObjectsTypeMetadata {
    Position position = {};
    double width = 0.0;
    double height = 0.0;
    double depth = 0.0;
    /// value of the `cartesian` flag; should be the same type as used in
    /// position, objectDivergence and zoneExclusion, otherwise expect
    /// warnings.
    bool cartesian = false;
    double gain = 1.0;
    double diffuse = 0.0;
    ChannelLock channelLock = {};
    ObjectDivergence objectDivergence = {};
    ZoneExclusion zoneExclusion = {};
    bool screenRef = false;
    /// screen specification from the `audioProgrammeReferenceScreen` element
    /// of the `audioProgramme` being rendered
    Screen referenceScreen = getDefaultScreen();
  };

  // typeDefinition==HOA

  /// Representation of all audioChannelFormats in a HOA audioPackFormat.
  ///
  /// \ref orders and \ref degrees must be the same length and must be in the
  /// same order as the channels being rendered, such that the `i`th input
  /// channel has order `orders[i]` and degree `degrees[i]`.
  ///
  /// \rst
  /// ``normalization``, ``nfcRefDist`` and ``screenRef`` may be defined in an
  /// ``audioBlockFormat`` and/or ``audioPackFormat``; see the rules in
  /// :cite:`bs2127` section 5.2.7.3 for details.
  /// \endrst
  struct EAR_EXPORT HOATypeMetadata {
    /// value of the `order` element in the `audioBlockFormat` element of each
    /// channel
    std::vector<int> orders;
    /// value of the `degree` element in the `audioBlockFormat` element of each
    /// channel
    std::vector<int> degrees;
    std::string normalization = std::string("SN3D");
    double nfcRefDist = 0.0;
    bool screenRef = false;
    /// screen specification from the `audioProgrammeReferenceScreen` element
    /// of the `audioProgramme` being rendered
    Screen referenceScreen = getDefaultScreen();
  };

}  // namespace ear
