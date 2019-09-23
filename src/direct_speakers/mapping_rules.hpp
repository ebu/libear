#pragma once
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace ear {
  // Remap a particular channel if all output loudspeakers in gains exist and
  // the input layout is as given.
  struct MappingRule {
    // Label of speaker to match.
    std::string speakerLabel;
    // Gains to match and apply.
    std::vector<std::pair<std::string, double>> gains;
    // Optional ITU names of input layouts to match against. If this isn't given
    // then the rule applies for any input layout.
    std::vector<std::string> input_layouts;
    // Optional ITU names of output layouts to match against. If this isn't
    // given then the rule applies for any output layout.
    std::vector<std::string> output_layouts;
  };

  // mapping rules, after processing with _add_symmetric_rules
  extern const std::vector<MappingRule> rules;

  // mapping from common definitions audioPackFormatID to ITU pack
  extern const std::map<std::string, std::string> itu_packs;
};  // namespace ear
