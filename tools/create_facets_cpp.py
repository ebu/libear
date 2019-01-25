#!/usr/bin/python
from ear.core.point_source import _convex_hull_facets
from ear.core.point_source import extra_pos_vertical_nominal
from ear.core.bs2051 import layouts
from attr import evolve
import numpy as np

print('#include "ear/common/facets.hpp"')
print('')
print('namespace ear {')

for layout_name in layouts:
    if (layout_name == '0+2+0'):
        continue
    # add virtual speakers
    layout = layouts[layout_name].without_lfe
    extra_channels, downmix = extra_pos_vertical_nominal(layout)
    layout_extra = evolve(layout, channels=layout.channels + extra_channels)
    virtual_positions = [[0, 0, -1]]
    if not ('T+000' in layout.channel_names or 'UH+180' in layout.channel_names):
        virtual_positions.append([0, 0, 1])
    positions_nominal = np.concatenate(
        (layout_extra.nominal_positions, virtual_positions))
    facets = _convex_hull_facets(positions_nominal)
    print('const std::vector<Facet> FACETS_' +
          layout_name.replace('+', '_') + ' = {')
    for facet in facets:
        print(str(facet) + ',')
    print('};')

print('\nconst std::map<std::string, std::vector<Facet>> FACETS = {')
for layout_name in layouts:
    if (layout_name == '0+2+0'):
        continue
    print('   {"' + str(layout_name) + '", FACETS_' +
          layout_name.replace('+', '_') + '},')

print('};')
print('}  // namespace ear')
