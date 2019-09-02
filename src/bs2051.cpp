#include "ear/bs2051.hpp"

#include "ear/helpers/assert.hpp"
#include "ear/metadata.hpp"
#include "resources.hpp"

#include "bs2051_layouts.cpp"

namespace ear {

  std::vector<Layout> loadLayouts() { return BS2051_LAYOUTS; }

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
