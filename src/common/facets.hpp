#pragma once
#include <map>
#include <set>
#include <string>
#include <vector>

namespace ear {
  using Facet = std::set<int>;
  extern std::map<std::string, std::vector<Facet>> FACETS;
}  // namespace ear
