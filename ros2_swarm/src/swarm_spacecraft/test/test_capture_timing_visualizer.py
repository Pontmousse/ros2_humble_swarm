"""Check the timing overlay with production initialization, without joining DDS."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy
from rclpy.time import Time
from swarm_interfaces.msg import Readiness
from visualization_msgs.msg import Marker

from swarm_spacecraft.spacecraft_visualizer import SpacecraftVisualizerNode


class OfflineVisualizer(SpacecraftVisualizerNode):
    def __init__(self, **parameters):
        self.parameters = parameters
        self.now_ns = 100_400_000_000
        self.captured_subscriptions, self.published = [], []
        with patch.object(Node, '__init__', lambda *a, **kw: None):
            super().__init__()

    def declare_parameter(self, name, default):
        self.parameters.setdefault(name, default)

    def get_parameter(self, name):
        return SimpleNamespace(value=self.parameters[name])

    def get_namespace(self):
        return '/RM99'

    def get_clock(self):
        return SimpleNamespace(now=lambda: Time(nanoseconds=self.now_ns))

    def get_logger(self):
        return SimpleNamespace(info=lambda *args: None)

    def create_subscription(self, msg_type, topic, callback, qos):
        self.captured_subscriptions.append((topic, qos))

    def create_publisher(self, *args):
        return SimpleNamespace(publish=self.published.append)

    def create_timer(self, *args):
        pass


def proposal(node, rid, action_time=0.0, ready=False, armed=False):
    msg = Readiness(robot_id=rid, action_time=action_time, ready=ready, armed=armed)
    node.readiness_callback(msg)


def text(node):
    return node.capture_timing_marker(node.get_clock().now()).text


def test_overlay_is_optional_and_publishes_without_odometry():
    node = OfflineVisualizer()
    assert not node.show_capture_timing
    assert not any(topic == '/readiness' for topic, _ in node.captured_subscriptions)
    node = OfflineVisualizer(show_capture_timing=True, x_min=-7.0, x_max=-1.0, y_max=2.0)
    assert next(qos for topic, qos in node.captured_subscriptions if topic == '/readiness').reliability \
        == ReliabilityPolicy.BEST_EFFORT
    node.publish_markers()
    markers = node.published[-1].markers
    table = next(m for m in markers if m.id == node.CAPTURE_TIMING_ID)
    assert table.type == Marker.TEXT_VIEW_FACING
    assert table.header.frame_id == 'swarm_map'
    assert (table.pose.position.x, table.pose.position.y) == (-4.0, 1.71)
    assert 'ROS_now=100.400s\nWaiting_for_readiness' == table.text
    assert len({(m.ns, m.id) for m in markers}) == len(markers)


def test_sorted_ids_deadlines_and_countdowns_use_the_same_clock():
    node = OfflineVisualizer(show_capture_timing=True)
    proposal(node, 13)
    proposal(node, 7, 103.0, ready=True)
    proposal(node, 2, 102.0, ready=True, armed=True)
    proposal(node, 4, 102.0, ready=True)
    rows = text(node).splitlines()
    assert rows[0] == 'ROS_now=100.400s'
    assert rows[1] == 'ID2:action=102.000,left=+1.600s,ARMED'
    assert rows[2] == 'ID4:action=102.000,left=+1.600s,READY'
    assert rows[3] == 'ID7:action=103.000,left=+2.600s,READY'
    assert rows[4] == 'ID13:action=--,left=--,WAIT'
    # A continuing readiness stream can advertise a deadline already in the past.
    node.now_ns = 102_100_000_000
    proposal(node, 2, 102.0, ready=True, armed=True)
    assert 'left=-0.100s,ARMED' in text(node)
    assert 'FIRED' not in text(node)


def test_staleness_and_paused_ros_time():
    node = OfflineVisualizer(show_capture_timing=True)
    proposal(node, 2, 102.0, ready=True)
    first = text(node)
    assert text(node) == first  # Wall time does not advance the ROS countdown.
    node.now_ns += 999_999_999
    assert 'STALE' not in text(node)
    node.now_ns += 1
    assert text(node).splitlines()[1] == 'ID2:action=--,left=--,STALE'
    proposal(node, 2)
    assert text(node).splitlines()[1] == 'ID2:action=--,left=--,WAIT'


def test_launch_enables_only_the_first_selected_visualizer(tmp_path):
    root = Path(__file__).resolve().parents[4]
    path = root / 'ros2_swarm/src/swarm_bringup/launch/launch_virtual_spacecraft.py'
    spec = importlib.util.spec_from_file_location('virtual_launch_timing_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    nodes = []
    original = module.Node
    def capture(**kwargs):
        nodes.append(kwargs)
        return original(**kwargs)
    with patch.dict('os.environ', {'ROBOT_NAME': 'RM4,RM2,RM8', 'ROS_LOG_DIR': str(tmp_path)}), \
            patch.object(module, 'Node', side_effect=capture):
        module.generate_launch_description()
    visualizers = [n for n in nodes if n['executable'] == 'spacecraft_visualizer']
    assert [(n['namespace'], n['parameters'][-1]['show_capture_timing']) for n in visualizers] \
        == [('RM4', True), ('RM2', False), ('RM8', False)]
