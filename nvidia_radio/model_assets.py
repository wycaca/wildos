from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path


def repository_root() -> Path:
    configured_root = os.environ.get("WILDOS_REPO_ROOT", "").strip()
    if configured_root:
        return Path(configured_root).expanduser().resolve()
    return Path(__file__).resolve().parents[1]


def verify_model_asset(path: str | Path, repo_root: Path | None = None) -> Path:
    root = (repo_root or repository_root()).resolve()
    candidate = Path(path)
    if not candidate.is_absolute():
        candidate = root / candidate
    candidate = Path(os.path.abspath(candidate))
    if not candidate.resolve().is_relative_to(root):
        raise RuntimeError(f"模型路径越出仓库, path={candidate}")

    entries = _manifest_entries(root)
    relative_path = candidate.relative_to(root).as_posix()
    entry = entries.get(relative_path)
    if entry is None:
        raise RuntimeError(f"模型不在授权清单中, path={relative_path}")
    if not candidate.is_file():
        raise RuntimeError(f"缺少模型文件, path={relative_path}")
    expected_size = int(entry["size"])
    actual_size = candidate.stat().st_size
    if actual_size != expected_size:
        raise RuntimeError(
            f"模型大小不匹配, path={relative_path}, "
            f"expected={expected_size}, actual={actual_size}"
        )
    actual_hash = _sha256(candidate)
    if actual_hash != entry["sha256"]:
        raise RuntimeError(
            f"模型 SHA256 不匹配, path={relative_path}, "
            f"expected={entry['sha256']}, actual={actual_hash}"
        )
    return candidate


def verify_model_manifest(repo_root: Path | None = None) -> int:
    root = (repo_root or repository_root()).resolve()
    entries = _manifest_entries(root)
    for relative_path in entries:
        verify_model_asset(root / relative_path, root)
    return len(entries)


def _manifest_entries(repo_root: Path) -> dict[str, dict]:
    manifest_path = repo_root / "ckpts" / "manifest.json"
    if not manifest_path.is_file():
        raise RuntimeError(f"缺少模型清单, path={manifest_path}")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    entries = manifest.get("assets")
    if not isinstance(entries, list) or not entries:
        raise RuntimeError(f"模型清单为空, path={manifest_path}")
    by_path = {entry["path"]: entry for entry in entries}
    if len(by_path) != len(entries):
        raise RuntimeError(f"模型清单包含重复路径, path={manifest_path}")
    return by_path


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()
