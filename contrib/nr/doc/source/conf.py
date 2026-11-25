# Copyright (c) 2024 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
# %SPDX-License-Identifier: GPL-2.0-only

import os
import sys

sys.path.insert(0, os.path.abspath("extensions"))

extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.doctest",
    "sphinx.ext.todo",
    "sphinx.ext.coverage",
    "sphinx.ext.imgmath",
    "sphinx.ext.ifconfig",
    "sphinx.ext.autodoc",
    "sphinx.ext.autosectionlabel",
]

latex_engine = "xelatex"
latex_elements = {"preamble": r"""
                 \usepackage{amsmath}
                 """}
todo_include_todos = True
templates_path = ["_templates"]
source_suffix = ".rst"
master_doc = "nr-module"

# Common ns-3 substitutions (defined globally in the full ns-3 docs via the
# per-book replace.txt; redefined here so the nr manual also builds standalone).
rst_prolog = """
.. |ns3| replace:: *ns-3*
.. |ns2| replace:: *ns-2*
"""

exclude_patterns = []
add_function_parentheses = True
numfig = True
# add_module_names = True
# modindex_common_prefix = []
html_theme = "sphinx_rtd_theme"
html_static_path = ["../static"]
html_css_files = [
    "custom.css",
]
html_logo = "../static/lena_logo.png"
html_theme_options = {
    "logo_only": True,
    "style_nav_header_background": "#433b67",
    "collapse_navigation": False,
    "navigation_depth": 3,
}
project = "NR Module"
copyright = "2022-2026, Centre Tecnològic de Telecomunicacions de Catalunya (CTTC)"
author = "OpenSim CTTC/CERCA"

version = "5.2.0"
release = "5.2.0"
