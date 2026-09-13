"""The suite must never join the robot's ROS domain. See ``conftest.py``.

Cheap insurance: if the conftest is deleted, reordered after an ``rclpy.init()``,
or its override is softened back to ``setdefault``, every other test still
passes — and the only symptom is fabricated log lines in a live robot's
``/rosout``, which reads as a robot fault. Fail loudly here instead.
"""

import os


def test_the_suite_is_not_on_the_robot_domain():
    domain = os.environ.get("ROS_DOMAIN_ID")
    assert domain is not None, "conftest.py did not set ROS_DOMAIN_ID"
    # navi-wall's launch files reject anything outside [1, 19], so a domain
    # beyond that range cannot be one the robot is running on.
    assert not (1 <= int(domain) <= 19), (
        f"tests are on ROS_DOMAIN_ID={domain}, inside the robot's [1, 19] range")
