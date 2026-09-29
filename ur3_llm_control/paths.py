"""Locate config files both in an installed ROS 2 workspace and in the source tree."""
import os


def config_path(name):
    try:
        from ament_index_python.packages import get_package_share_directory
        p = os.path.join(get_package_share_directory('ur3_llm_control'), 'config', name)
        if os.path.exists(p):
            return p
    except Exception:
        pass
    return os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                         '..', 'config', name))
