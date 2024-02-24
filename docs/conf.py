import os
import sys
sys.path.insert(0, os.path.abspath('../jl'))

project = 'UiAbot Documentation'
copyright = '2023, University of Agder'
author = 'Daniel Hagen, Martin Mæland, Tarjei Skotterud, and Martin Dahlseng Hermansen'
release = '0.0.1'


extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.viewcode',
    'sphinx.ext.napoleon'
]
templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']
html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']

# Added from GitLab Sphinx
html_theme_options = {
    'collapse_navigation': False,
    'sticky_navigation': True,
    'navigation_depth': -1,
}

html_logo = 'src/fig/uiabot_logo.png'
# Adding custom stylesheet
def setup(app):
    app.add_css_file('css/custom.css')
