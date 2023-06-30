#pragma once
#include <boost/optional.hpp>
#include <string>
#include <vector>
#include "common_types.hpp"
#include "export.hpp"
#include "screen.hpp"

namespace ear {

  /** @brief Representation of a channel, with a name, real and nominal
   * positions, allowed azimuth and elevation ranges, and an lfe flag.
   */
  class EAR_EXPORT Channel {
   public:
    Channel() = default;
    /**
     * @param name Channel name.
     * @param polarPosition  real speaker location
     * @param polarPositionNominal nominal speaker location, defaults to
     *     polar_position
     * @param azimuthRange
     *     azimuth range in degrees; allowed range is interpreted as
     *     starting at azimuthRange[0], moving anticlockwise to azimuthRange[1];
     *     defaults to the azimuth of polar_nominal_position.
     * @param elevationRange
     *     elevation range in degrees; allowed range is interpreted as
     *     starting at elevationRange.first, moving up to elevationRange.second
     *     defaults to the elevation of polar_nominal_position.
     * @param isLfe flag to indicate an LFE channel
     */
    Channel(
        const std::string& name, PolarPosition polarPosition,
        boost::optional<PolarPosition> polarPositionNominal = boost::none,
        boost::optional<std::pair<double, double>> azimuthRange = boost::none,
        boost::optional<std::pair<double, double>> elevationRange = boost::none,
        bool isLfe = false);

    const std::string& name() const;
    PolarPosition polarPosition() const;
    PolarPosition polarPositionNominal() const;
    std::pair<double, double> azimuthRange() const;
    std::pair<double, double> elevationRange() const;
    bool isLfe() const;

    void name(const std::string& name);
    void polarPosition(PolarPosition polarPosition);
    void polarPositionNominal(
        const boost::optional<PolarPosition>& polarPositionNominal);
    void azimuthRange(
        const boost::optional<std::pair<double, double>>& azimuthRange);
    void elevationRange(
        const boost::optional<std::pair<double, double>>& elevationRange);
    void isLfe(bool isLfe);

    void checkPosition(std::function<void(const std::string&)> callback) const;

   private:
    std::string _name;
    PolarPosition _polarPosition;
    boost::optional<PolarPosition> _polarPositionNominal;
    boost::optional<std::pair<double, double>> _azimuthRange;
    boost::optional<std::pair<double, double>> _elevationRange;
    bool _isLfe;
  };

  /** @brief Representation of a loudspeaker layout, with a name and a list of
   * channels.
   */
  class EAR_EXPORT Layout {
   public:
    Layout(std::string name = "",
           std::vector<Channel> channels = std::vector<Channel>(),
           boost::optional<Screen> screen = getDefaultScreen());

    std::string name() const;
    std::vector<Channel>& channels();
    std::vector<Channel> channels() const;
    boost::optional<Screen> screen() const;

    void name(std::string name);
    void screen(boost::optional<Screen> screen);
    Layout withoutLfe() const;
    std::vector<bool> isLfe() const;
    std::vector<std::string> channelNames() const;
    void checkPositions(std::function<void(const std::string&)> callback) const;
    Channel channelWithName(const std::string& name) const;
    boost::optional<int> indexForName(const std::string& name) const;
    std::vector<PolarPosition> positions() const;
    std::vector<PolarPosition> nominalPositions() const;

   private:
    std::string _name;
    std::vector<Channel> _channels;
    boost::optional<Screen> _screen;
  };

}  // namespace ear
