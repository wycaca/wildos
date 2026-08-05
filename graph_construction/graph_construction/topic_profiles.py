from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Optional

from ament_index_python.packages import get_package_share_directory
import yaml


PROFILE_CONFIG_NAME = "topic_profiles.yaml"


def load_topic_profile(profile_name: str, profile_file: Optional[str] = None) -> Dict[str, Any]:
    """从安装配置或显式文件加载指定 topic profile"""
    config_path = _resolve_profile_path(profile_file)
    with config_path.open("r", encoding="utf-8") as config_stream:
        config = yaml.safe_load(config_stream) or {}

    profiles = config.get("profiles") or {}
    if profile_name not in profiles:
        available = ", ".join(sorted(profiles)) or "<none>"
        raise ValueError(f"Unknown topic profile '{profile_name}', available profiles: {available}")

    profile = dict(profiles[profile_name])
    profile["name"] = profile_name
    return profile


def _resolve_profile_path(profile_file: Optional[str]) -> Path:
    if profile_file:
        return Path(profile_file).expanduser()

    share_dir = Path(get_package_share_directory("graph_construction"))
    return share_dir / "configs" / PROFILE_CONFIG_NAME
