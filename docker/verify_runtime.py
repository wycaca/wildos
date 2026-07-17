from __future__ import annotations

import os
from pathlib import Path
import platform

import cupy
import rclpy
import torch


def main() -> None:
    """Validate architecture, model assets and both CUDA consumers"""
    expected_arch = os.environ.get("WILDOS_IMAGE_ARCH", "").strip()
    actual_arch = platform.machine()
    if expected_arch and actual_arch != expected_arch:
        raise RuntimeError(
            f"镜像架构不匹配, expected={expected_arch}, actual={actual_arch}"
        )

    repo_root = Path(os.environ.get("WILDOS_REPO_ROOT", "/opt/wildos_ws/src/nebula2-wildos"))
    required_assets = (
        repo_root / "ckpts" / "c-radio_v3-b_half.pth.tar",
        repo_root / "ckpts" / "frontier_ckpt_new.ckpt",
        repo_root / "ckpts" / "traversability_ckpt.ckpt",
        repo_root / "ckpts" / "siglip2",
    )
    missing_assets = [str(path) for path in required_assets if not path.exists()]
    if missing_assets:
        raise RuntimeError(f"缺少模型文件, paths={missing_assets}")

    if not torch.cuda.is_available():
        raise RuntimeError("PyTorch 无法访问 CUDA GPU")
    cupy_devices = int(cupy.cuda.runtime.getDeviceCount())
    if cupy_devices < 1:
        raise RuntimeError("CuPy 无法访问 CUDA GPU")

    print(
        "WildOS 容器运行环境检查通过, "
        f"arch={actual_arch}, torch={torch.__version__}, "
        f"cuda={torch.version.cuda}, cupy_devices={cupy_devices}, "
        f"rclpy={Path(rclpy.__file__).parent}",
        flush=True,
    )


if __name__ == "__main__":
    main()
