.. cpp:namespace:: ear::conversion

Metadata Conversion
###################

Functions are provided for converting Objects metadata between polar and
Cartesian formats according to :cite:`bs2127` section 10.

See the :doc:`EAR reference documentation <ear:conversion>` for more general information.

To convert positions only, use :func:`pointCartToPolar` and :func:`pointPolarToCart`.

To convert positions and extent parameters, use :func:`extentCartToPolar` and :func:`extentPolarToCart`.

To convert whole block formats, use :func:`toPolar` and :func:`toCartesian`.
