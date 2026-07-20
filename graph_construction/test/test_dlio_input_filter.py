from sensor_msgs.msg import Imu

from graph_construction.dlio_input_filter import (
    is_strictly_new_stamp,
    stamp_nanoseconds,
)


def test_stamp_nanoseconds_combines_ros_stamp_fields():
    msg = Imu()
    msg.header.stamp.sec = 12
    msg.header.stamp.nanosec = 345

    assert stamp_nanoseconds(msg) == 12_000_000_345


def test_strict_stamp_filter_drops_duplicates_and_backwards_time():
    assert is_strictly_new_stamp(100, None)
    assert is_strictly_new_stamp(101, 100)
    assert not is_strictly_new_stamp(100, 100)
    assert not is_strictly_new_stamp(99, 100)
