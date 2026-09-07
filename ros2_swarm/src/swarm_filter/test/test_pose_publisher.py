"""Tests for the standard physical odometry interface."""

import math

from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
import pytest

from swarm_filter.pose_publisher import genuine_gps_update_time
from swarm_filter.pose_publisher import make_localization_odometry
from swarm_filter.pose_publisher import make_map_to_odometry_transform
from swarm_filter.pose_publisher import namespaced_base_frame
from swarm_filter.pose_publisher import source_is_fresh
from swarm_filter.pose_publisher import UNOBSERVED_VARIANCE


def test_gps_freshness_changes_only_with_device_timestamp():
    first_update = genuine_gps_update_time(10, None, 100, None)
    repeated_update = genuine_gps_update_time(10, 10, 200, first_update)
    next_update = genuine_gps_update_time(11, 10, 300, repeated_update)
    assert first_update == 100
    assert repeated_update == 100
    assert next_update == 300
    assert source_is_fresh(200_000_000, 100_000_000, 0.25)
    assert not source_is_fresh(400_000_000, 100_000_000, 0.25)


def test_base_frame_prefers_driver_and_has_namespaced_fallback():
    assert namespaced_base_frame("/RM1", "RM1/chassis") == "RM1/chassis"
    assert namespaced_base_frame("/RM1", "") == "RM1/base_link"
    assert namespaced_base_frame("/", "") == "base_link"


def test_localization_odometry_combines_gps_pose_and_driver_twist():
    source = Odometry()
    source.child_frame_id = "RM1/base_link"
    source.pose.covariance[35] = 0.04
    source.twist.twist.linear.x = 0.3
    source.twist.twist.linear.y = -0.2
    source.twist.twist.angular.z = 0.1
    source.twist.covariance[0] = 0.005

    message = make_localization_odometry(
        x=1.2,
        y=-0.4,
        yaw=math.pi / 2.0,
        source_odometry=source,
        stamp=Time(sec=2),
        global_frame_id="swarm_map",
        namespace="/RM1",
        gps_position_variance=0.01,
    )

    assert message.header.stamp.sec == 2
    assert message.header.frame_id == "swarm_map"
    assert message.child_frame_id == "RM1/base_link"
    assert message.pose.pose.position.x == pytest.approx(1.2)
    assert message.pose.pose.position.y == pytest.approx(-0.4)
    assert message.pose.pose.orientation.z == pytest.approx(math.sqrt(0.5))
    assert message.pose.pose.orientation.w == pytest.approx(math.sqrt(0.5))
    assert message.pose.covariance[0] == pytest.approx(0.01)
    assert message.pose.covariance[7] == pytest.approx(0.01)
    assert message.pose.covariance[14] == UNOBSERVED_VARIANCE
    assert message.pose.covariance[21] == UNOBSERVED_VARIANCE
    assert message.pose.covariance[28] == UNOBSERVED_VARIANCE
    assert message.pose.covariance[35] == pytest.approx(0.04)
    assert message.pose.covariance[1] == 0.0
    assert message.twist.twist.linear.x == pytest.approx(0.3)
    assert message.twist.twist.linear.y == pytest.approx(-0.2)
    assert message.twist.twist.angular.z == pytest.approx(0.1)
    assert message.twist.covariance[0] == pytest.approx(0.005)


def test_map_to_odometry_transform_composes_global_and_driver_poses():
    source = Odometry()
    source.header.frame_id = "/RM1/odom"
    source.pose.pose.position.x = 1.0
    source.pose.pose.position.y = 0.0
    source.pose.pose.orientation.w = 1.0

    transform = make_map_to_odometry_transform(
        x=2.0,
        y=3.0,
        yaw=math.pi / 2.0,
        source_odometry=source,
        stamp=Time(sec=7),
        global_frame_id="swarm_map",
    )

    assert transform.header.frame_id == "swarm_map"
    assert transform.child_frame_id == "RM1/odom"
    assert transform.header.stamp.sec == 7
    assert transform.transform.translation.x == pytest.approx(2.0)
    assert transform.transform.translation.y == pytest.approx(2.0)
    assert transform.transform.rotation.z == pytest.approx(math.sqrt(0.5))
    assert transform.transform.rotation.w == pytest.approx(math.sqrt(0.5))


def test_map_to_odometry_transform_rejects_an_empty_driver_frame():
    source = Odometry()
    source.pose.pose.orientation.w = 1.0
    with pytest.raises(ValueError):
        make_map_to_odometry_transform(
            0.0,
            0.0,
            0.0,
            source,
            Time(),
            "swarm_map",
        )
