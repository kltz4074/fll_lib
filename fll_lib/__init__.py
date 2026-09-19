__version__ = "1.2.0"

from fll_lib.core.robot import create_robot
from fll_lib.runtime import detect_platform

__all__ = ["create_robot", "detect_platform", "__version__"]
