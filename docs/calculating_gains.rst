.. cpp:namespace:: ear

Calculating Gains
#################

To calculate gains for an ADM item, the ADM metadata is transferred into a
``TypeMetadata`` struct for that type, which is passed to the ``::calculate``
method of a ``GainCalculator`` object instantiated with the desired loudspeaker
layout in order to calculate the gains. Both types are dependant on the ADM
``typeDefinition``, and are described below.

``TypeMetadata`` structures (and sub-structures) for each type are defined in
:ref:`file_ear_metadata.hpp`, while ``GainCalculator`` classes are defined in
:ref:`file_ear_ear.hpp`.

The mapping between ADM metadata and ``TypeMetadata`` structures must be
performed by the user of the library. The basic mapping is given in the
documentation for each ``TypeMetadata`` structure (linked below), but for more
details see :cite:`bs2127` section 5.2.

Errors may be returned by throwing exceptions during ``GainCalculator``
constructor or ``::calculate`` calls, while warnings may be returned from
``::calculate`` through the ``warning_cb`` parameter; see :doc:`error_handling`
for details.

For information on loudspeaker layouts, see :doc:`loudspeaker_layouts`.

DirectSpeakers
~~~~~~~~~~~~~~

*DirectSpeakers* metadata is represented by the
:class:`DirectSpeakersTypeMetadata`, and gains are calculated by
:class:`GainCalculatorDirectSpeakers`.

Objects
~~~~~~~

*Objects* metadata is represented by the
:class:`ObjectsTypeMetadata`, and gains are calculated by
:class:`GainCalculatorObjects`.

Two gain vectors are produced by :func:`GainCalculatorObjects::calculate`,
``directGains`` and ``diffuseGains``. To apply these, see :ref:`dsp_objects`.

HOA
~~~

*HOA* metadata is represented by the
:class:`HOATypeMetadata`, and a decode matrix is calculated by
:class:`GainCalculatorHOA`.
