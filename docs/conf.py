# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
# import os
# import sys
# sys.path.insert(0, os.path.abspath('.'))


# -- Project information -----------------------------------------------------
from datetime import datetime
import os
import re

# Check if we set a specific environment variable to skip the heavy lifting
fast_build = os.getenv('FAST_BUILD') == '1'

project = 'taproot'
copyright = str(datetime.now().year) + ', taproot'
author = 'taproot'

# The full version, including alpha/beta/rc tags
release = '1.0.0'


# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
]

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']


# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = 'sphinx_rtd_theme'

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['_static']


breathe_default_project = "taproot"

extensions = [
    # there may be others here already, e.g. 'sphinx.ext.mathjax'
    'breathe',
    'exhale'
]

# Setup the breathe extension
breathe_projects = {
    "taproot": "./doxyoutput/xml"
}

# Setup the exhale extension
exhale_args = {
    # These arguments are required
    "containmentFolder":     "./api",
    "rootFileName":          "library_root.rst",
    "rootFileTitle":         "Library API",
    "doxygenStripFromPath":  "..",
    # Suggested optional arguments
    "createTreeView":        True,
    # TIP: if using the sphinx-bootstrap-theme, you need
    # "treeViewIsBootstrap": True,
    # "exhaleExecutesDoxygen": True,
    "exhaleExecutesDoxygen": not fast_build,
    "exhaleUseDoxyfile":     True
}

# Tell sphinx what the primary language being documented is.
primary_domain = 'cpp'

# Tell sphinx what the pygments highlight language should be.
highlight_language = 'cpp'


def convert_md_to_rst(source_path, dest_path):
    """
    Converts a basic Markdown file to ReStructuredText.
    Supports H1-H4 headers.
    """
    
    # Mapping: Number of '#' -> The RST underline character
    header_map = {
        1: "-",  # #  -> -----
        2: "^",  # ## -> ^^^^^
        3: "\"", # ### -> """""
        4: "~",  # #### -> ~~~~~
    }

    if os.path.exists(source_path):
        with open(source_path, "r") as f_in, open(dest_path, "w") as f_out:
            # Write the static file header
            f_out.write("Changelog\n")
            f_out.write("=========\n\n")

            for line in f_in:
                # Regex checks for lines starting with 1 or more '#' followed by a space
                # Group 1 captures the hashes, Group 2 captures the text
                match = re.match(r"^(#+)\s+(.*)", line)

                if match:
                    hashes, content = match.groups()
                    level = len(hashes)
                    
                    # Clean up the content (remove trailing newlines or extra # on the right)
                    content = content.strip().rstrip("#").strip()

                    if level in header_map:
                        char = header_map[level]
                        f_out.write(content + "\n")
                        f_out.write(char * len(content) + "\n\n")
                    else:
                        # Fallback for H5+ or unmapped headers: write as bold text
                        f_out.write(f"**{content}**\n\n")
                else:
                    # Simple link support [text](url) -> `text <url>`_
                    line = re.sub(r'\[([^\]]+)\]\(([^)]+)\)', r'`\1 <\2>`_', line)
                    
                    f_out.write(line)
    else:
        print(f"[Sphinx] WARNING: Could not find {source_path}")

def generate_changelog():
    # This assumes conf.py is in taproot/docs/ and CHANGELOG is in taproot/
    curr_dir = os.path.dirname(os.path.abspath(__file__))
    source_path = os.path.join(curr_dir, "../CHANGELOG.md")
    dest_path = os.path.join(curr_dir, "changelog.rst")
    convert_md_to_rst(source_path, dest_path)

# Generates and includes the changelog in the docs
generate_changelog()