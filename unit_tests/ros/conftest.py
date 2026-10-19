# Import robot_base_driver straight from the repository source, so edits are
# tested without rebuilding the ros_tools image.
import sys
from pathlib import Path

import pytest
import rclpy

PACKAGE_DIR = Path(__file__).resolve().parents[2] / 'robot' / 'ros2_ws' / 'src' / 'robot_base_driver'
sys.path.insert(0, str(PACKAGE_DIR))


@pytest.fixture(scope='session', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()
