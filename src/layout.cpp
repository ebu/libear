#include "ear/layout.hpp"

#include "common/geom.hpp"

namespace ear {

  Channel::Channel(const std::string& name, PolarPosition polarPosition,
                   boost::optional<PolarPosition> polarPositionNominal,
                   boost::optional<std::pair<double, double>> azimuthRange,
                   boost::optional<std::pair<double, double>> elevationRange,
                   bool isLfe)
      : _name(name),
        _polarPosition(polarPosition),
        _polarPositionNominal(polarPositionNominal),
        _azimuthRange(azimuthRange),
        _elevationRange(elevationRange),
        _isLfe(isLfe){};

  std::string Channel::name() const { return _name; }
  PolarPosition Channel::polarPosition() const { return _polarPosition; }
  PolarPosition Channel::polarPositionNominal() const {
    return get_optional_value_or(_polarPositionNominal, _polarPosition);
  }
  std::pair<double, double> Channel::azimuthRange() const {
    return get_optional_value_or(
        _azimuthRange,
        std::make_pair(polarPosition().azimuth, polarPosition().azimuth));
  }
  std::pair<double, double> Channel::elevationRange() const {
    return get_optional_value_or(
        _elevationRange,
        std::make_pair(polarPosition().elevation, polarPosition().elevation));
  }
  bool Channel::isLfe() const { return _isLfe; };

  void Channel::name(const std::string& name) { _name = name; }
  void Channel::polarPosition(PolarPosition polarPosition) {
    _polarPosition = polarPosition;
  }
  void Channel::polarPositionNominal(
      const boost::optional<PolarPosition>& polarPositionNominal) {
    _polarPositionNominal = polarPositionNominal;
  }
  void Channel::azimuthRange(
      const boost::optional<std::pair<double, double>>& azimuthRange) {
    _azimuthRange = azimuthRange;
  }
  void Channel::elevationRange(
      const boost::optional<std::pair<double, double>>& elevationRange) {
    _elevationRange = elevationRange;
  }
  void Channel::isLfe(bool isLfe) { _isLfe = isLfe; }

  void Channel::checkPosition(
      std::function<void(const std::string&)> callback) const {
    if (_azimuthRange &&
        !insideAngleRange(polarPosition().azimuth, azimuthRange().first,
                          azimuthRange().second)) {
      std::stringstream ss;
      ss << name() << ": azimuth " << polarPosition().azimuth
         << " out of range [" << azimuthRange().first << ", "
         << azimuthRange().second << "]";
      callback(ss.str());
    }
    if (_elevationRange &&
        !(elevationRange().first <= polarPosition().elevation &&
          polarPosition().elevation <= elevationRange().second)) {
      std::stringstream ss;
      ss << name() << ": elevation " << polarPosition().elevation
         << " out of range [" << elevationRange().first << ", "
         << elevationRange().second << "]";
      callback(ss.str());
    }
  }

  Layout::Layout(std::string name, std::vector<Channel> channels,
                 boost::optional<Screen> screen)
      : _name(std::move(name)),
        _channels(std::move(channels)),
        _screen(screen){};

  std::string Layout::name() const { return _name; }
  std::vector<Channel>& Layout::channels() { return _channels; }
  std::vector<Channel> Layout::channels() const { return _channels; }
  boost::optional<Screen> Layout::screen() const { return _screen; }

  void Layout::name(std::string name) { _name = name; }
  void Layout::screen(boost::optional<Screen> screen) { _screen = screen; }

  Layout Layout::withoutLfe() const {
    Layout layoutWithoutLfe;
    layoutWithoutLfe.name(_name);
    layoutWithoutLfe.screen(_screen);
    std::copy_if(_channels.begin(), _channels.end(),
                 back_inserter(layoutWithoutLfe.channels()),
                 [](const Channel& c) { return !c.isLfe(); });
    return layoutWithoutLfe;
  };

  std::vector<bool> Layout::isLfe() const {
    std::vector<bool> isLfeMapping;
    std::transform(_channels.begin(), _channels.end(),
                   std::back_inserter(isLfeMapping),
                   [](Channel c) -> bool { return c.isLfe(); });
    return isLfeMapping;
  };

  std::vector<std::string> Layout::channelNames() const {
    std::vector<std::string> names;
    std::transform(_channels.begin(), _channels.end(),
                   std::back_inserter(names),
                   [](Channel c) -> std::string { return c.name(); });
    return names;
  };

  void Layout::checkPositions(
      std::function<void(const std::string&)> callback) const {
    for (const auto& channel : _channels) {
      channel.checkPosition(callback);
    }
  }

  Channel Layout::channelWithName(const std::string& name) const {
    auto it = std::find_if(
        _channels.begin(), _channels.end(),
        [&name](const Channel c) -> bool { return c.name() == name; });
    return *it;
  }

  boost::optional<int> Layout::indexForName(const std::string& name) const {
    auto it = std::find_if(
        _channels.begin(), _channels.end(),
        [&name](const Channel c) -> bool { return c.name() == name; });
    if (it != _channels.end()) {
      return static_cast<int>(std::distance(_channels.begin(), it));
    } else {
      return boost::none;
    }
  }

  std::vector<PolarPosition> Layout::positions() const {
    std::vector<PolarPosition> pos;
    std::transform(
        _channels.begin(), _channels.end(), std::back_inserter(pos),
        [](Channel c) -> PolarPosition { return c.polarPosition(); });
    return pos;
  }

  std::vector<PolarPosition> Layout::nominalPositions() const {
    std::vector<PolarPosition> pos;
    std::transform(
        _channels.begin(), _channels.end(), std::back_inserter(pos),
        [](Channel c) -> PolarPosition { return c.polarPositionNominal(); });
    return pos;
  }

}  // namespace ear
