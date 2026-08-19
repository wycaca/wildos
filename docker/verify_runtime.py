from __future__ import annotations

import os
from pathlib import Path
import platform

from nvidia_radio.model_assets import verify_model_manifest


def main() -> None:
    """Validate architecture, model assets and both CUDA consumers"""
    expected_arch = os.environ.get("WILDOS_IMAGE_ARCH", "").strip()
    actual_arch = platform.machine()
    if expected_arch and actual_arch != expected_arch:
        raise RuntimeError(
            f"镜像架构不匹配, expected={expected_arch}, actual={actual_arch}"
        )

    repo_root = Path(os.environ.get("WILDOS_REPO_ROOT", "/opt/wildos_ws/src/nebula2-wildos"))
    verified_assets = verify_model_manifest(repo_root)

    import cupy
    import rclpy
    import torch

    if not torch.cuda.is_available():
        raise RuntimeError("PyTorch 无法访问 CUDA GPU")
    cupy_devices = int(cupy.cuda.runtime.getDeviceCount())
    if cupy_devices < 1:
        raise RuntimeError("CuPy 无法访问 CUDA GPU")

    print(
        "WildOS 容器运行环境检查通过, "
        f"arch={actual_arch}, torch={torch.__version__}, "
        f"cuda={torch.version.cuda}, cupy_devices={cupy_devices}, "
        f"rclpy={Path(rclpy.__file__).parent}, models={verified_assets}",
        flush=True,
    )


if __name__ == "__main__":
    main()
