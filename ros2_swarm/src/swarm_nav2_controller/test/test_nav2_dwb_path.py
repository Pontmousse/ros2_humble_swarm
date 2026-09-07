from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
import pytest

from swarm_nav2_controller.nav2_dwb_path import controller_id_for_mode
from swarm_nav2_controller.nav2_dwb_path import make_single_pose_path
from swarm_nav2_controller.nav2_dwb_path import Nav2DwbPathNode
from swarm_nav2_controller.nav2_dwb_path import reference_is_fresh


@pytest.mark.parametrize(
    ("mode", "controller_id"),
    [
        ("dwb", "DWBStandard"),
        ("dwb_ff", "DWBVelocityFF"),
        ("dwb_velocity_feedback", "DWBVelocityFeedback"),
    ],
)
def test_controller_mode_mapping(mode, controller_id):
    assert controller_id_for_mode(mode) == controller_id


def test_invalid_controller_mode_is_rejected():
    with pytest.raises(ValueError):
        controller_id_for_mode("pff")


def test_single_pose_path_copies_reference_pose_and_frame():
    reference = Odometry()
    reference.header.frame_id = "/swarm_map"
    reference.pose.pose.position.x = 1.25
    reference.pose.pose.position.y = -0.5
    reference.pose.pose.orientation.z = 0.2
    reference.pose.pose.orientation.w = 0.98
    stamp = Time(sec=4, nanosec=5)

    path = make_single_pose_path(reference, stamp)

    assert path.header.frame_id == "swarm_map"
    assert path.header.stamp == stamp
    assert len(path.poses) == 1
    assert path.poses[0].header == path.header
    assert path.poses[0].pose == reference.pose.pose


def test_reference_freshness_rejects_missing_old_and_future_updates():
    now = 1_000_000_000
    assert not reference_is_fresh(now, None, 0.25)
    assert reference_is_fresh(now, 800_000_000, 0.25)
    assert not reference_is_fresh(now, 700_000_000, 0.25)
    assert not reference_is_fresh(now, 1_100_000_000, 0.25)


def test_reference_validity_requires_a_nonempty_frame():
    class Adapter:
        latest_reference = Odometry()
        last_reference_time = 900_000_000
        reference_timeout = 0.25

    adapter = Adapter()
    assert not Nav2DwbPathNode.reference_is_valid(adapter, 1_000_000_000)
    adapter.latest_reference.header.frame_id = "swarm_map"
    assert Nav2DwbPathNode.reference_is_valid(adapter, 1_000_000_000)


def test_all_accepted_goals_are_cancelled_together():
    class DoneFuture:
        def add_done_callback(self, callback):
            callback(self)

    class GoalHandle:
        def __init__(self):
            self.cancelled = False

        def cancel_goal_async(self):
            self.cancelled = True
            return DoneFuture()

    class Adapter:
        def __init__(self, handles):
            self.active_goal_handles = handles
            self.cancel_requests_in_flight = 0

        def cancel_done_callback(self, future):
            Nav2DwbPathNode.cancel_done_callback(self, future)

    handles = [GoalHandle(), GoalHandle()]
    adapter = Adapter(handles.copy())

    Nav2DwbPathNode.cancel_active_goals(adapter)

    assert all(handle.cancelled for handle in handles)
    assert adapter.active_goal_handles == []
    assert adapter.cancel_requests_in_flight == 0
