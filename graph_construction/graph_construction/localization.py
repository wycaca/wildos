from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping


SUPPORTED_LOCALIZATION_BACKENDS = {"platform", "dlio"}


@dataclass(frozen=True)
class LocalizationWiring:
    backend: str
    odom_input_topic: str
    mapping_pointcloud_topic: str
    dlio_pointcloud_input_topic: str = ""
    dlio_imu_input_topic: str = ""
    dlio_aligned_odom_topic: str = ""
    dlio_reference_odom_topic: str = ""
    dlio_local_frame: str = ""
    dlio_alignment_delay: float = 0.0
    use_pointcloud_axis_adapter: bool = True
    isolate_platform_tf: bool = False


def resolve_localization_wiring(
    profile: Mapping[str, object],
    backend: str,
) -> LocalizationWiring:
    """Resolve the odom, point cloud and TF ownership for one backend"""
    normalized_backend = str(backend).strip().lower()
    if normalized_backend not in SUPPORTED_LOCALIZATION_BACKENDS:
        supported = ", ".join(sorted(SUPPORTED_LOCALIZATION_BACKENDS))
        raise ValueError(
            f"Unsupported localization_backend={backend}, "
            f"expected one of: {supported}"
        )

    if normalized_backend == "platform":
        return LocalizationWiring(
            backend=normalized_backend,
            odom_input_topic=_required(profile, "odom_input_topic"),
            mapping_pointcloud_topic=_required(profile, "aligned_lidar_topic"),
        )

    return LocalizationWiring(
        backend=normalized_backend,
        odom_input_topic=_required(profile, "dlio_odom_topic"),
        mapping_pointcloud_topic=_required(profile, "dlio_deskewed_topic"),
        dlio_pointcloud_input_topic=_required(
            profile,
            "dlio_pointcloud_input_topic",
        ),
        dlio_imu_input_topic=_required(profile, "dlio_imu_input_topic"),
        dlio_aligned_odom_topic=_required(profile, "dlio_aligned_odom_topic"),
        dlio_reference_odom_topic=str(
            profile.get("dlio_reference_odom_topic", "")
        ).strip(),
        dlio_local_frame=_required(profile, "dlio_local_frame"),
        dlio_alignment_delay=float(profile.get("dlio_alignment_delay", 0.0)),
        use_pointcloud_axis_adapter=False,
        isolate_platform_tf=True,
    )


def _required(profile: Mapping[str, object], key: str) -> str:
    value = str(profile.get(key, "")).strip()
    if not value:
        raise ValueError(
            f"Localization profile is missing required key: {key}"
        )
    return value
