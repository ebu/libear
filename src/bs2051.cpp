#include "ear/bs2051.hpp"

#include "ear/helpers/assert.hpp"
#include "ear/metadata.hpp"
#include "resources.hpp"
#include "yaml-cpp/yaml.h"

namespace ear {

  Channel parseChannel(YAML::Node channel_node) {
    Channel channel;
    channel.name(channel_node["name"].as<std::string>());
    channel.polarPosition(
        PolarPosition{channel_node["position"]["az"].as<double>(),
                      channel_node["position"]["el"].as<double>()});
    if (channel_node["az_range"]) {
      channel.azimuthRange(
          channel_node["az_range"].as<std::pair<double, double>>());
    }
    if (channel_node["el_range"]) {
      channel.elevationRange(
          channel_node["el_range"].as<std::pair<double, double>>());
    }
    if (channel_node["is_lfe"]) {
      channel.isLfe(channel_node["is_lfe"].as<bool>());
    } else {
      channel.isLfe(false);
    }
    return channel;
  }

  Layout parseLayout(YAML::Node layout_node) {
    Layout layout;
    layout.name(layout_node["name"].as<std::string>());
    for (auto channel_node : layout_node["channels"]) {
      layout.channels().push_back(parseChannel(channel_node));
    }
    return layout;
  }

  std::vector<Layout> loadLayouts() {
    std::stringstream layoutsXml;
    ear_assert(getEmbeddedFile("2051_layouts.yaml", layoutsXml),
               "could not find embedded file 2051_layouts.yaml, this is "
               "probably a compilation error");
    YAML::Node root_node = YAML::Load(layoutsXml.str());

    std::vector<Layout> layouts;
    for (auto layout_node : root_node) {
      layouts.push_back(parseLayout(layout_node));
    }
    return layouts;
  }

  Layout getLayout(const std::string& name) {
    auto layouts = loadLayouts();
    auto it = std::find_if(
        layouts.begin(), layouts.end(),
        [&name](const Layout& l) -> bool { return l.name() == name; });
    if (it == layouts.end()) {
      throw unknown_layout(name);
    }
    return *it;
  }

}  // namespace ear
