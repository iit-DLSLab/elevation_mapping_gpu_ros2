import os
import sys

PACKAGE_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", "elevation_mapping_cupy")
)
sys.path.insert(0, PACKAGE_ROOT)

autodoc_mock_imports = [
    "cupy",
    "cupyx",
    "cupyx.scipy",
    "cupyx.scipy.ndimage",
    "cv2",
    "detectron2",
    "geometry_msgs",
    "grid_map_msgs",
    "matplotlib",
    "message_filters",
    "rclpy",
    "ros2_numpy",
    "ruamel",
    "ruamel.yaml",
    "scipy",
    "sensor_msgs",
    "shapely",
    "shapely.geometry",
    "simple_parsing",
    "simple_parsing.helpers",
    "sklearn",
    "sklearn.decomposition",
    "std_msgs",
    "tf2_ros",
    "tf_transformations",
    "torch",
    "torchvision",
]

on_rtd = os.environ.get("READTHEDOCS", None) == "True"
html_theme = "sphinx_rtd_theme"

project = "elevation_mapping_cupy"
copyright = "2022, Takahiro Miki, Gian Erni"
author = "Takahiro Miki, Gian Erni"

extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.coverage",
    "sphinx.ext.napoleon",
    "sphinx_copybutton",
]

templates_path = ["_templates"]
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]

html_theme_options = {
    "analytics_anonymize_ip": False,
    "logo_only": False,
    "prev_next_buttons_location": "bottom",
    "style_external_links": False,
    "vcs_pageview_mode": "",
    "collapse_navigation": True,
    "sticky_navigation": True,
    "navigation_depth": 4,
    "includehidden": True,
    "titles_only": False,
}

STATIC_DIR = os.path.join(os.path.dirname(__file__), "_static")
html_static_path = ["_static"] if os.path.isdir(STATIC_DIR) else []

autodoc_default_options = {
    "members": True,
    "member-order": "bysource",
    "special-members": "__init__",
    "undoc-members": True,
    "exclude-members": "__weakref__",
}
