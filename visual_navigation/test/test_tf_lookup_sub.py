from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy

from visual_navigation.utils.tf_lookup_sub import dynamic_tf_qos


def test_dynamic_tf_qos_matches_unity_and_dlio_publishers():
    qos = dynamic_tf_qos(7)

    assert qos.depth == 7
    assert qos.reliability == ReliabilityPolicy.RELIABLE
    assert qos.durability == DurabilityPolicy.VOLATILE
    assert qos.history == HistoryPolicy.KEEP_LAST
