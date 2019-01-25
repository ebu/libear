.. cpp:namespace:: ear

Error Handling
==============

Two classes of error are produced by the library: exceptions and warnings.

Exceptions
~~~~~~~~~~

Exceptions are thrown by the library for severe errors which prevent the
requested operation from being completed. All errors thrown by the library are
defined in :ref:`file_ear_exceptions.hpp`

Notably, this library does not yet implement all features of :cite:`bs2127`;
when such a feature is used, a :class:`not_implemented` will be thrown.

Warnings
~~~~~~~~

Warnings are issued in less severe cases, where the requested operation could
still be completed. Warnings are generally issued for user-facing problems,
such as errors in metadata, and so should be visible to the user.

Warnings are returned from the library through the ``warnings_cb`` argument to
``::calculate`` methods on ``GainCalculator`` objects (for example
:func:`GainCalculatorObjects::calculate`). Each time a warning is issued, the
provided callback will be called with a :class:`Warning` structure, containing
the type and message of the warning. By default, :var:`default_warning_cb` is
used for this argument, which will print warnings to stderr.

Warning structures are defines in :ref:`file_ear_warnings.hpp`.
