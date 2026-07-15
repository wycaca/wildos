from typing import List, Sequence

import numpy as np
import torch
import torch.nn.functional as F

from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from object_search_msgs.msg import ObjectMaskWithTf
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header, MultiArrayDimension, UInt8MultiArray
from tf2_msgs.msg import TFMessage


def localize_query(
    text_feats: torch.Tensor,
    spatial_feats: torch.Tensor,
    orig_img_shape: tuple,
    pixel_level_seg: bool,
    mask_threshold: float,
):
    """
    Localizes the text query in the spatial features.

    :param text_feats: (num_queries, D) Text feature vectors.
    :param spatial_feats: (B, D, H, W) Spatial feature map from the image.
    :param orig_img_shape: (H, W) Original image shape.
    :param pixel_level_seg: Whether to perform pixel-level segmentation.
    :param mask_threshold: Threshold for converting similarity maps to binary masks.

    :return: similarity_map, binary_mask
    """
    # Normalize features
    spatial_feats = spatial_feats / spatial_feats.norm(dim=1, keepdim=True)

    # Compute similarity maps
    text_sim_spatial = torch.einsum('qc,bchw->bqhw', text_feats, spatial_feats)

    # Resize similarity maps to match the original image size
    if pixel_level_seg:
        interp_mode = 'bilinear'
    else:
        interp_mode = 'nearest'
    text_sim_spatial = F.interpolate(
        text_sim_spatial, size=(orig_img_shape[0], orig_img_shape[1]), mode=interp_mode
    ).cpu().numpy()  # Shape: (B, num_queries, H, W)

    # Binary mask
    binary_mask = (text_sim_spatial > mask_threshold).astype(np.uint8)

    return text_sim_spatial, binary_mask


def convert_maskmsg_to_multiarray(mask_msg: np.ndarray) -> UInt8MultiArray:
    """
    Converts a binary mask to a UInt8MultiArray message.

    :param mask_msg: (B, 1, H, W) Binary mask of the detected object.
    :return: UInt8MultiArray message.
    """
    multiarray_msg = UInt8MultiArray()
    multiarray_msg.data = mask_msg.flatten().tolist()
    multiarray_msg.layout.data_offset = 0
    b, c, h, w = mask_msg.shape
    multiarray_msg.layout.dim = [
        MultiArrayDimension(label='batch', size=b, stride=b * c * h * w),
        MultiArrayDimension(label='channel', size=c, stride=c * h * w),
        MultiArrayDimension(label='height', size=h, stride=h * w),
        MultiArrayDimension(label='width', size=w, stride=w),
    ]
    return multiarray_msg


def get_objectmask_msg(
    binary_mask: np.ndarray,
    cam_inverted: bool,
    odom_msg: Odometry,
    tf_data: List[TransformStamped],
    cam_info_msgs: List[CameraInfo],
    query: str = "",
    camera_scores: List[float] | None = None,
    measurement_header: Header | None = None,
) -> ObjectMaskWithTf:
    """
    Converts the binary mask and associated data into an ObjectMaskWithTf message.

    :param binary_mask: (3, 1, H, W) Binary mask of the detected object.
    :param cam_inverted: Whether the camera is inverted.
    :param odom_msg: Odometry message for the robot's pose.
    :param tf_data: List of TF data for frame transformations from camera to odom.
    :param cam_info_msgs: List of CameraInfo messages for the cameras.

    :return: ObjectMaskWithTf message.
    """

    if cam_inverted:
        binary_mask = np.rot90(binary_mask, k=2, axes=(2, 3))

    obj_mask_msg = ObjectMaskWithTf()
    obj_mask_msg.header = measurement_header or odom_msg.header

    obj_mask_msg.odom = odom_msg
    obj_mask_msg.cam_infos = cam_info_msgs
    obj_mask_msg.object_mask = convert_maskmsg_to_multiarray(binary_mask)
    obj_mask_msg.cam_transforms = TFMessage(transforms=tf_data)
    obj_mask_msg.query = query
    obj_mask_msg.camera_scores = list(camera_scores or [])

    return obj_mask_msg


def reference_image_stamp(image_msgs: Sequence) -> Time:
    """使用多相机图像中位时间, 避免旧图像错误套用当前 odom 位姿"""
    if not image_msgs:
        raise ValueError("至少需要一条相机图像消息")
    ordered = sorted(image_msgs, key=lambda msg: _stamp_nanoseconds(msg.header.stamp))
    return ordered[len(ordered) // 2].header.stamp


def _stamp_nanoseconds(stamp: Time) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
