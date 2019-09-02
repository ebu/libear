#!/usr/bin/python
from ear.core.bs2051 import layouts

print('#include <utility>\n')
print('namespace ear {\n')
print('const std::vector<Layout> BS2051_LAYOUTS = {')
for layout_name in layouts:
    layout = layouts[layout_name]
    print('Layout{{"{name}", std::vector<Channel>{{'.format(name = layout_name))
    for channel in layout.channels:
        print('Channel{{"{name}", PolarPosition{{{az}, {el}}}, '.format(
            name = channel.name,
            az = channel.polar_position.azimuth,
            el = channel.polar_position.elevation))
        if channel.polar_nominal_position:
            print('PolarPosition{{{az}, {el}}}, '.format(
                az = channel.polar_nominal_position.azimuth,
                el = channel.polar_nominal_position.elevation))
        else:
            print('boost::none, ')
        if channel.az_range:
            print('std::make_pair({az}, {el}), '.format(
                az = float(channel.az_range[0]),
                el = float(channel.az_range[1])))
        else:
            print('boost::none, ')
        if channel.el_range:
            print('std::make_pair({az}, {el}), '.format(
                az = float(channel.el_range[0]),
                el = float(channel.el_range[1])))
        else:
            print('boost::none, ')
        if channel.is_lfe:
            print('true},')
        else:
            print('false},')
    print('}},')
print('};')
print('}  // namespace ear')
