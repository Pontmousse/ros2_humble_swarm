"""Tests for virtual-spacecraft ROS interface helpers."""

from swarm_spacecraft.virtual_spacecraft_node import namespaced_virtual_frame
from swarm_spacecraft.virtual_spacecraft_node import timestamp_is_fresh


def test_virtual_frame_is_unique_in_robot_namespace():
    assert namespaced_virtual_frame("/RM1") == "RM1/virtual_spacecraft"
    assert namespaced_virtual_frame("/") == "virtual_spacecraft"


def test_localization_timestamp_must_exist_and_be_fresh():
    assert not timestamp_is_fresh(1_000_000_000, None, 0.25)
    assert not timestamp_is_fresh(1_000_000_000, 700_000_000, 0.25)
    assert timestamp_is_fresh(1_000_000_000, 800_000_000, 0.25)
