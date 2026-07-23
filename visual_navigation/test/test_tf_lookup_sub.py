from builtin_interfaces.msg import Time
from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy

from visual_navigation.utils.buffer import MessageBuffer
from visual_navigation.utils.tf_lookup_sub import dynamic_tf_qos


def test_dynamic_tf_qos_matches_unity_and_dlio_publishers():
    qos = dynamic_tf_qos(7)

    assert qos.depth == 7
    assert qos.reliability == ReliabilityPolicy.RELIABLE
    assert qos.durability == DurabilityPolicy.VOLATILE
    assert qos.history == HistoryPolicy.KEEP_LAST


def test_message_buffer_keeps_latest_when_full():
    buffer = MessageBuffer(max_size=1, wait_for_oldest=False)
    buffer.add_msg({"frame": 1}, Time(sec=1))
    buffer.add_msg({"frame": 2}, Time(sec=2))

    assert buffer.get_oldest_msg()["frame"] == 2
