from builtin_interfaces.msg import Time
from sensor_msgs.msg import CameraInfo

from graph_construction.camera_stamp_adapter import replace_header_stamp


def test_replace_header_stamp_preserves_frame_and_payload():
    msg = CameraInfo()
    msg.header.frame_id = "front_camera"
    msg.header.stamp = Time(sec=10)
    msg.width = 640

    output = replace_header_stamp(msg, Time(sec=1200, nanosec=50))

    assert output is msg
    assert output.header.stamp.sec == 1200
    assert output.header.stamp.nanosec == 50
    assert output.header.frame_id == "front_camera"
    assert output.width == 640
