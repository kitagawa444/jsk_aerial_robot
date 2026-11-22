"""Allow local Bee helpers and catkin-generated bee.msg to share a package."""
from pkgutil import extend_path

__path__ = extend_path(__path__, __name__)
