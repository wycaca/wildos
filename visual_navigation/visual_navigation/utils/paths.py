import os
from pathlib import Path


def repository_root() -> Path:
    """Return the source repository root, with an explicit override for installs"""
    override = os.environ.get("WILDOS_REPO_ROOT")
    if override:
        return Path(override).expanduser().resolve()
    return Path(__file__).resolve().parents[3]
