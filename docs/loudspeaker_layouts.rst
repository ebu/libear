.. cpp:namespace:: ear

Loudspeaker Layouts
===================

An output loudspeaker layout is represented by :class:`Layout`, which contains
a name, a list of :class:`Channel` objects, and the reference :type:`Screen`.

Although :class:`Layout` objects could be constructed directly from scratch,
only layouts listed in :cite:`bs2051` are supported. The function
:func:`getLayout` is therefore provided to obtain a :class:`Layout` object
given the layout name, e.g. ``4+5+0``.
