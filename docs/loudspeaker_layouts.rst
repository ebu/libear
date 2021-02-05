.. cpp:namespace:: ear

Loudspeaker Layouts
===================

An output loudspeaker layout is represented by :class:`Layout`, which contains
a name, a list of :class:`Channel` objects, and the reference :type:`Screen`.

Loudspeaker layouts specified in :cite:`bs2051` are supported, including
positions within the ranges specified. The function :func:`getLayout` is
therefore provided to obtain a :class:`Layout` object given the layout name,
e.g. ``4+5+0``.

When using layouts with non-nominal positions, the
:func:`Channel::polarPositionNominal` must match the position specified in Table
1 in :cite:`bs2051`, and the :func:`Channel::polarPosition` must meet the
specified constraints, including azimuth/elevation ranges and symmetry
requirements.

Non-standard loudspeaker layouts may be used, however their behaviour may
change in future versions of the library. Loudspeaker layouts must be similar
to those in :cite:`bs2051`, with 1, 2 or 3 layers and an optional T+000 or
UH+180 loudspeaker. Using this functionality requires some understanding of the
internals of the renderer, particularly section 6.1.3.1 of :cite:`bs2127`.
