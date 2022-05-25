import os
import textwrap

project = "libear"
copyright = "2019, libear authors"
author = "EBU"
release = "0.9"

extensions = ["breathe", "exhale", "m2r2", "sphinxcontrib.bibtex", "sphinx.ext.intersphinx"]

intersphinx_mapping = {'ear': ('https://ear.readthedocs.io/en/latest/', None)}

# Setup the breathe extension
breathe_projects = {"libear": "./_build/doxyoutput/xml"}
breathe_default_project = "libear"
breathe_domain_by_extension = {"hpp": "cpp"}

# Setup the exhale extension
exhaleDoxygenStdin = """
INPUT = ../include/ear/ear.hpp
INPUT += ../include/ear/gain_calculators.hpp
INPUT += ../include/ear/metadata.hpp
INPUT += ../include/ear/bs2051.hpp
INPUT += ../include/ear/exceptions.hpp
INPUT += ../include/ear/warnings.hpp
INPUT += ../include/ear/layout.hpp
INPUT += ../include/ear/screen.hpp
INPUT += ../include/ear/common_types.hpp
INPUT += ../include/ear/decorrelate.hpp
INPUT += ../include/ear/fft.hpp
INPUT += ../include/ear/dsp/dsp.hpp
INPUT += ../include/ear/dsp/block_convolver.hpp
EXCLUDE_SYMBOLS += *ZeroTrack
INPUT += ../include/ear/dsp/delay_buffer.hpp
INPUT += ../include/ear/dsp/gain_interpolator.hpp
INPUT += ../include/ear/dsp/ptr_adapter.hpp
INPUT += ../include/ear/dsp/variable_block_size.hpp

EXCLUDE_SYMBOLS += *~*
EXCLUDE_SYMBOLS += block_convolver_impl

EXPAND_AS_DEFINED = EAR_EXPORT
PREDEFINED += EAR_EXPORT=

JAVADOC_AUTOBRIEF = YES
EXTRACT_ALL = YES

STRIP_CODE_COMMENTS = NO
"""
exhale_args = {
    "containmentFolder": "./api",
    "rootFileName": "library_root.rst",
    "rootFileTitle": "API Reference",
    "doxygenStripFromPath": "../include/",
    "createTreeView": True,
    "exhaleExecutesDoxygen": True,
    "exhaleDoxygenStdin": exhaleDoxygenStdin,
    "fullToctreeMaxDepth": 1,
}

primary_domain = "cpp"

highlight_language = "cpp"

templates_path = ["_templates"]

exclude_patterns = ["_build", "Thumbs.db", ".DS_Store", "README.md", "env"]

# html_static_path = ["_static"]

numfig = True

# on_rtd is whether we are on readthedocs.org, this line of code grabbed from
# docs.readthedocs.org
on_rtd = os.environ.get("READTHEDOCS", None) == "True"

if not on_rtd:  # only import and set the theme if we're building docs locally
    import sphinx_rtd_theme

    html_theme = "sphinx_rtd_theme"
    html_theme_path = [sphinx_rtd_theme.get_html_theme_path()]

# configure sphinxcontrib.bibtex
bibtex_bibfiles = ['refs.bib']

# make a pybtex style which uses the bibtex key for labels

from pybtex.style.formatting.unsrt import Style as UnsrtStyle
from pybtex.style.labels.alpha import LabelStyle as AlphaLabelStyle
from pybtex.plugin import register_plugin


class KeyLabelStyle(AlphaLabelStyle):
    def format_label(self, entry):
        return entry.fields["key"]


class KeyStyle(UnsrtStyle):
    default_label_style = "key"


register_plugin("pybtex.style.labels", "key", KeyLabelStyle)
register_plugin("pybtex.style.formatting", "keystyle", KeyStyle)
